/**
 * @file audio.c
 * @brief Nextube audio driver – WAV file playback via DAC continuous driver.
 *
 * Hardware: GPIO25 → LTK8002D amplifier (DAC_CHAN_0).
 *
 * Uses the IDF 5.x dac_continuous driver (driver/dac_continuous.h) to replace
 * the legacy I2S-built-in-DAC path (driver/i2s.h + driver/dac.h).
 *
 * Supports standard PCM WAV files (8-bit or 16-bit, mono or stereo).
 * 16-bit signed samples are down-converted to 8-bit unsigned before writing
 * to the DAC (the DAC is 8-bit; the continuous driver always accepts uint8_t).
 * MP3 files are logged but skipped – the DAC requires raw PCM.
 *
 * Playback runs in a dedicated FreeRTOS task so audio_play_file() returns
 * immediately.  A mutex serialises concurrent play requests.
 *
 * DAC mode lifecycle:
 *   idle    – oneshot channel holds DAC at 0 (low rail).
 *             AC-coupled output: idle voltage doesn't matter; the amp's
 *             internal bias holds its input at VDD/2 regardless.
 *             Idling at 0 makes every mode-switch a 0→0 transition —
 *             matching the DMA ring's zero-initialised state — so no
 *             oneshot stepping ramp is ever needed.
 *   playing – oneshot deleted, continuous channel streams PCM samples;
 *             DMA prime ramp (0→128) and end ramp (128→0) handle the
 *             transitions silently at audio sample rate.
 */

#include "audio.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/dac_continuous.h"
#include "driver/dac_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <strings.h>    /* strcasecmp */
#include "esp_heap_caps.h"
#include "esp_timer.h"

static const char *TAG = "audio";

/* ── Runtime state ─────────────────────────────────────────────────── */
static int               s_volume      = 20;
static volatile bool     s_stop_flag   = false;
static TaskHandle_t      s_audio_task  = NULL;
static SemaphoreHandle_t s_play_mutex  = NULL;

/* DAC handles – only one is active at a time */
static dac_continuous_handle_t s_dac_cont = NULL;   /* during playback  */
static dac_oneshot_handle_t    s_dac_one  = NULL;   /* during idle      */

/* ── Buffer / DMA sizes ─────────────────────────────────────────────── */
#define STREAM_BUF_BYTES   4096   /* file read chunk; also 8-bit output buf */
#define DAC_DESC_NUM          8   /* DMA descriptor count                   */
#define DAC_DMA_BUF_SIZE   2048   /* bytes per DMA descriptor               */

/* ── WAV RIFF header (44 bytes, little-endian) ─────────────────────── */
typedef struct __attribute__((packed)) {
    char     riff_id[4];        /* "RIFF"             */
    uint32_t file_size;         /* total_size - 8     */
    char     wave_id[4];        /* "WAVE"             */
    char     fmt_id[4];         /* "fmt "             */
    uint32_t fmt_size;          /* 16 for PCM         */
    uint16_t audio_format;      /* 1 = PCM            */
    uint16_t num_channels;      /* 1 or 2             */
    uint32_t sample_rate;       /* e.g. 44100         */
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;   /* 8 or 16            */
} wav_riff_hdr_t;

/* ── DAC lifecycle helpers ──────────────────────────────────────────── */

/*
 * Transition from idle (oneshot) to continuous mode for streaming.
 * Oneshot and continuous channels cannot coexist on the same DAC output.
 */
static esp_err_t dac_cont_start(uint32_t sample_rate)
{
    /* Idle strategy: DAC is held at 0 (not 128) between sounds.
     *
     * Why 0 instead of 128 (mid-rail):
     *   Any discrete step of the oneshot DAC is audible through the
     *   AC-coupled LTK8002D, regardless of how slowly it is stepped.
     *   Each step fires through the AC coupling and produces a click;
     *   65 steps at 1 ms intervals sound like a 1 kHz buzz.
     *
     *   Keeping idle at 0 eliminates the oneshot ramp entirely:
     *   - DMA ring is zero-initialised (0x00) by the driver, matching idle (0).
     *   - The oneshot→continuous switch is 0→0 — completely silent.
     *   - When the DAC floats briefly during the mode switch, the AC-coupling
     *     cap was charged to +VDD/2 (bias = VDD/2, DAC = 0), so the LTK8002D
     *     input stays at VDD/2 = silence throughout — no transient at all.
     *   - The DMA prime ramp (0→128, ~100 ms) and end ramp (128→0, 64 ms)
     *     operate at 32 kHz sample rate; each step is ~0.06 DAC units, which
     *     the AC coupling treats as a ~8.7 mV DC signal — inaudible. */
    if (s_dac_one) {
        dac_oneshot_del_channel(s_dac_one);
        s_dac_one = NULL;
    }

    /* Guard: clean up any orphaned continuous handle left by a previous
     * task that was killed (e.g. watchdog) before reaching dac_cont_stop().
     * Without this, dac_continuous_new_channels() returns an error on every
     * subsequent call and audio never works again until reboot. */
    if (s_dac_cont) {
        ESP_LOGW(TAG, "Orphaned dac_cont handle detected — cleaning up before restart");
        dac_continuous_disable(s_dac_cont);
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
    }

    dac_continuous_config_t cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num  = DAC_DESC_NUM,
        .buf_size  = DAC_DMA_BUF_SIZE,
        .freq_hz   = sample_rate,
        .clk_src   = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode  = DAC_CHANNEL_MODE_SIMUL,
    };
    esp_err_t err = dac_continuous_new_channels(&cfg, &s_dac_cont);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dac_continuous_new_channels: %s", esp_err_to_name(err));
        /* Reclaim oneshot so idle silence is restored */
        dac_oneshot_config_t one_cfg = { .chan_id = DAC_CHAN_0 };
        if (dac_oneshot_new_channel(&one_cfg, &s_dac_one) == ESP_OK)
            dac_oneshot_output_voltage(s_dac_one, 0);   /* idle at 0 */
        return err;
    }

    err = dac_continuous_enable(s_dac_cont);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dac_continuous_enable: %s", esp_err_to_name(err));
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
        dac_oneshot_config_t one_cfg = { .chan_id = DAC_CHAN_0 };
        if (dac_oneshot_new_channel(&one_cfg, &s_dac_one) == ESP_OK)
            dac_oneshot_output_voltage(s_dac_one, 0);   /* idle at 0 */
    }
    return err;
}

/*
 * Transition from continuous mode back to idle (oneshot).
 *
 * Idle strategy: DAC is held at 0 (low rail) between sounds.
 * The DMA end ramp in task_cleanup already fades the ring to 0, so
 * del_channels() fires while the ring output is 0.  The RTC register
 * also holds 0 (never changed since init), so the digi-bypass switch is
 * 0→0 — completely silent.  No oneshot ramp is needed. */
static void dac_cont_stop(void)
{
    if (s_dac_cont) {
        dac_continuous_disable(s_dac_cont);
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
    }

    /* Restore oneshot and hold at 0 (low rail = idle). */
    if (!s_dac_one) {
        dac_oneshot_config_t one_cfg = { .chan_id = DAC_CHAN_0 };
        if (dac_oneshot_new_channel(&one_cfg, &s_dac_one) == ESP_OK)
            dac_oneshot_output_voltage(s_dac_one, 0);
    }
}

/* ── Volume scaling ─────────────────────────────────────────────────── */
/*
 * In-place volume attenuation applied before bit-depth conversion.
 * 16-bit: samples are signed (centre = 0).
 * 8-bit:  samples are unsigned (centre = 128).
 */
static void apply_volume(uint8_t *buf, int len_bytes,
                         uint16_t bits_per_sample, int vol_pct)
{
    if (vol_pct >= 100) return;
    const float scale = vol_pct / 100.0f;

    if (bits_per_sample == 16) {
        int16_t *s = (int16_t *)(void *)buf;
        int      n = len_bytes / 2;
        for (int i = 0; i < n; i++)
            s[i] = (int16_t)roundf((float)s[i] * scale);
    } else {
        for (int i = 0; i < len_bytes; i++)
            buf[i] = (uint8_t)(128 + (int)roundf(((int)buf[i] - 128) * scale));
    }
}

/*
 * Convert 16-bit signed PCM to 8-bit unsigned PCM in-place.
 * Works by taking the high byte and re-centering: val = (s16 >> 8) + 128.
 * Returns the number of 8-bit output bytes produced (= len_bytes / 2).
 */
static int pcm16_to_pcm8(uint8_t *buf, int len_bytes)
{
    int16_t *s16    = (int16_t *)(void *)buf;
    int      samples = len_bytes / 2;
    for (int i = 0; i < samples; i++)
        buf[i] = (uint8_t)((s16[i] >> 8) + 128);
    return samples;
}

/* ── Playback task ──────────────────────────────────────────────────── */
typedef struct { char path[128]; } play_arg_t;

static void audio_play_task(void *arg)
{
    play_arg_t *a = (play_arg_t *)arg;
    char path[128];
    strncpy(path, a->path, sizeof(path) - 1);
    path[sizeof(path) - 1] = '\0';
    free(a);

    ESP_LOGI(TAG, "task start: internal_free=%u  total_free=%u  stack_hwm=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)esp_get_free_heap_size(),
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    uint8_t *buf = NULL;   /* internal-SRAM DMA window */
    /* Declared here (not inside the streaming block) so task_cleanup can
     * read total_bytes_out regardless of which goto reaches it. */
    uint32_t frame = 0, write_stalls = 0, total_bytes_out = 0;

    /* ── Open file ── */
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open audio file: %s", path);
        goto task_exit;
    }

    /* ── Parse RIFF/WAVE header ── */
    wav_riff_hdr_t hdr;
    if (fread(&hdr, 1, sizeof(hdr), f) < (int)sizeof(hdr)) {
        ESP_LOGE(TAG, "Short read on WAV header: %s", path);
        goto task_close;
    }
    if (memcmp(hdr.riff_id, "RIFF", 4) != 0 ||
        memcmp(hdr.wave_id, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Not a WAV file: %s", path);
        goto task_close;
    }
    if (hdr.audio_format != 1) {
        ESP_LOGE(TAG, "Only PCM WAV supported (format=%u)", hdr.audio_format);
        goto task_close;
    }

    /* ── Find the 'data' sub-chunk (skip 'LIST', 'fact', etc.) ── */
    {
        long data_start = -1;
        fseek(f, 12, SEEK_SET);   /* skip RIFF/WAVE preamble */
        while (!feof(f)) {
            char     cid[4];
            uint32_t csz;
            if (fread(cid, 1, 4, f) < 4) break;
            if (fread(&csz, 1, 4, f) < 4) break;
            if (memcmp(cid, "data", 4) == 0) {
                data_start = ftell(f);
                break;
            }
            fseek(f, (long)(csz + (csz & 1)), SEEK_CUR);
        }
        if (data_start < 0) {
            ESP_LOGE(TAG, "No 'data' chunk in: %s", path);
            goto task_close;
        }
        fseek(f, data_start, SEEK_SET);
    }

    ESP_LOGI(TAG, "WAV play: %s  %u Hz  %u ch  %u-bit  vol=%d%%",
             path, (unsigned)hdr.sample_rate, hdr.num_channels,
             hdr.bits_per_sample, s_volume);

    /* ── Upsample factor for low sample-rate files ────────────────────
     * The ESP32 DAC DMA uses I2S0 internally.  The I2S clock source is
     * D2PLL (160 MHz), with a 32× multiplier (2 ch × 16-bit slots).
     * mclk_div = 160 MHz / (sample_rate × 32); the register is 8-bit (max 255).
     * Minimum rate = 160 000 000 / (255 × 32) ≈ 19 608 Hz.
     * 8 000 Hz (divider 625) and 16 000 Hz (divider 312) both exceed 255 and
     * return ESP_ERR_INVALID_ARG from dac_continuous_new_channels().
     * Fix: integer upsample until dac_rate ≥ 20 000 Hz. */
    uint32_t upsample = 1;
    uint32_t dac_rate = hdr.sample_rate;
    while (dac_rate < 20000) { upsample <<= 1; dac_rate <<= 1; }
    if (upsample > 1)
        ESP_LOGI(TAG, "Upsampling x%u: %u Hz → %u Hz (DAC min ≈19608 Hz)",
                 (unsigned)upsample, (unsigned)hdr.sample_rate, (unsigned)dac_rate);

    /* ── DMA window in internal SRAM ──────────────────────────────────
     * The DAC DMA controller cannot access PSRAM.  All PCM is streamed
     * directly from SPIFFS into this internal-SRAM window (no pre-buffer).
     * 8-bit mono files at 8 000 Hz are small (≤ ~32 KB) so SPIFFS read
     * latency is negligible; fread() timing is logged for diagnostics. */
    buf = (uint8_t *)heap_caps_malloc(STREAM_BUF_BYTES,
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "OOM for DMA window (internal SRAM)");
        goto task_cleanup;
    }
    ESP_LOGI(TAG, "DMA window @%p (internal SRAM)", buf);

    /* ── Start DAC continuous at the (possibly upsampled) rate ── */
    if (dac_cont_start(dac_rate) != ESP_OK)
        goto task_cleanup;   /* buf allocated; task_cleanup frees it */

    /* ── Prime DMA ring with a short silence burst ─────────────────────
     * dac_continuous_enable() zeros the DMA descriptors (0x00 = full
     * negative rail).  A step from oneshot idle (128 = mid-rail) down to
     * 0x00 is amplified as a pop.  Writing ~100 ms of 0x80 before the
     * first PCM data smooths the transition without causing a noticeable
     * delay.
     *
     * Priming = sample_rate / 10 bytes (100 ms at any sample rate).
     * At 8 000 Hz this is 800 bytes — far less than the old full-ring fill
     * of 16 384 bytes which caused a ~2-second silence before audio started. */
    {
        size_t prime_bytes = dac_rate / 10;   /* 100 ms of 8-bit mono at dac_rate */
        if (prime_bytes > STREAM_BUF_BYTES) prime_bytes = STREAM_BUF_BYTES;
        /* Ramp 0→128 over the priming window.
         * The DMA ring starts at 0x00 (zero-filled by the driver) which
         * matches the idle voltage (0).  A flat jump to 128 would be a hard
         * 0→128 step through the AC coupling; a slow linear ramp is treated
         * as a near-DC signal (fundamental ≈ 10 Hz) attenuated by the high-
         * pass characteristic to ~8.7 mV — inaudible. */
        for (size_t i = 0; i < prime_bytes; i++)
            buf[i] = (uint8_t)(128 * i / prime_bytes);
        size_t _w;
        dac_continuous_write(s_dac_cont, buf, prime_bytes, &_w, pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "DMA ring primed with %u-byte ramp 0→128 (~100 ms)",
                 (unsigned)prime_bytes);
    }

    /* ── Stream PCM data directly from SPIFFS ── */
    {
        int64_t  t_start = esp_timer_get_time();

        /* When upsampling, read fewer raw bytes so the expanded output still
         * fits inside the STREAM_BUF_BYTES DMA window.
         *   8-bit  : read STREAM_BUF_BYTES/upsample → expand → STREAM_BUF_BYTES
         *   16-bit : read STREAM_BUF_BYTES/upsample → pcm16_to_pcm8 halves it
         *            → expand × upsample → STREAM_BUF_BYTES/2  (always fits)  */
        const size_t read_size = STREAM_BUF_BYTES / upsample;

        while (!s_stop_flag) {
            int64_t t_rd = esp_timer_get_time();
            int rd = (int)fread(buf, 1, read_size, f);
            int64_t rd_us = esp_timer_get_time() - t_rd;
            if (rd <= 0) break;
            /* Warn only if SPIFFS read latency exceeds half the DMA ring's
             * capacity (~256 ms at 32 kHz).  Shorter spikes are absorbed
             * silently by the ring and don't affect audio continuity. */
            if (rd_us > 250000)
                ESP_LOGW(TAG, "fread slow: %lld ms (frame %u)",
                         (long long)(rd_us / 1000), frame);

            /* Volume attenuation (operates on native bit depth) */
            apply_volume(buf, rd, hdr.bits_per_sample, s_volume);

            /* DAC continuous only accepts 8-bit unsigned PCM */
            int out_bytes = rd;
            if (hdr.bits_per_sample == 16)
                out_bytes = pcm16_to_pcm8(buf, rd);

            /* Integer nearest-neighbour upsample ─────────────────────────
             * Expand 8-bit samples in-place, back-to-front, so we never
             * overwrite a source sample before it has been duplicated. */
            if (upsample > 1) {
                for (int i = out_bytes - 1; i >= 0; i--) {
                    uint8_t s = buf[i];
                    for (uint32_t j = 0; j < upsample; j++)
                        buf[(uint32_t)i * upsample + j] = s;
                }
                out_bytes *= (int)upsample;
            }

            size_t written = 0;
            int64_t t_wr = esp_timer_get_time();
            esp_err_t werr = dac_continuous_write(s_dac_cont, buf, (size_t)out_bytes,
                                                  &written, pdMS_TO_TICKS(1000));
            int64_t wr_us = esp_timer_get_time() - t_wr;

            /* Stall detection: only flag if the write blocked for longer than
             * a full ring drain (ring_bytes/dac_rate ≈ 512 ms at 32 kHz) plus
             * margin.  Normal back-pressure for short files that nearly fill
             * the ring (e.g. a 500 ms click in a 512 ms ring) can legitimately
             * block for ~330 ms — well below 700 ms — and produces no gap.
             * A real problem (DMA stopped, descriptor queue wedged) would
             * approach or exceed the 1000 ms write timeout. */
            if (wr_us > 700000) {
                write_stalls++;
                ESP_LOGW(TAG, "DAC write stall: %lld ms (frame %u, written=%u/%d)",
                         (long long)(wr_us / 1000), frame,
                         (unsigned)written, out_bytes);
            }
            if (werr != ESP_OK) {
                ESP_LOGW(TAG, "DAC write error %s — stopping playback",
                         esp_err_to_name(werr));
                break;
            }
            total_bytes_out += (uint32_t)written;
            frame++;
        }

        int64_t elapsed_ms = (esp_timer_get_time() - t_start) / 1000;
        ESP_LOGI(TAG, "playback done: %u frames  %u bytes  %lld ms  %u stalls",
                 frame, total_bytes_out, (long long)elapsed_ms, write_stalls);
    }

task_cleanup:
    if (buf && s_dac_cont) {
        /* ── End-of-playback fade ──────────────────────────────────────────
         *
         * Append a 128→0 linear ramp to the DMA ring so the last sample
         * output is 0, matching the RTC register (idle = 0).
         *
         * dac_continuous_del_channels() switches DAC from digi-bypass to the
         * RTC register.  Because both are 0, the switch is 0→0 — silent.
         *
         * Ramp = 2048 samples = 64 ms at 32 kHz.  Each sample step is
         * 128/2047 ≈ 0.06 DAC units.  Through the AC-coupled LTK8002D the
         * ramp's fundamental (≈ 15 Hz) is attenuated to ~8.7 mV — inaudible.
         *
         * Drain timing: ring_occ = min(total_bytes_out, ring_bytes).
         * Wait until the ring (plus the appended ramp) has fully played out
         * before issuing del_channels. */
        const size_t   RAMP_BYTES = 2048;                    /* 64 ms at 32 kHz */
        const uint32_t ring_bytes = (uint32_t)DAC_DESC_NUM * DAC_DMA_BUF_SIZE;

        /* Build the 128 → 0 ramp (linear, last sample exactly 0). */
        for (size_t i = 0; i < RAMP_BYTES; i++)
            buf[i] = (uint8_t)(128 * (RAMP_BYTES - 1 - i) / (RAMP_BYTES - 1));

        size_t _fw;
        dac_continuous_write(s_dac_cont, buf, RAMP_BYTES, &_fw, pdMS_TO_TICKS(500));

        uint32_t ring_occ = (total_bytes_out < ring_bytes) ? total_bytes_out : ring_bytes;
        vTaskDelay(pdMS_TO_TICKS(ring_occ * 1000 / dac_rate + 150));
        /* DAC is now at 0; dac_cont_stop() switches 0→0, restores oneshot at 0. */
    }
    free(buf);
    dac_cont_stop();

task_close:
    fclose(f);
task_exit:
    ESP_LOGI(TAG, "task exit: stack_hwm=%u  internal_free=%u",
             (unsigned)uxTaskGetStackHighWaterMark(NULL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    xSemaphoreGive(s_play_mutex);
    s_audio_task = NULL;
    vTaskDelete(NULL);
}

/* ════════════════════════════════════════════════════════════════════ */
/*  Public API                                                          */
/* ════════════════════════════════════════════════════════════════════ */

void audio_init(void)
{
    /* Pin this component's log level to INFO at runtime.
     * Prevents a global esp_log_level_set("*", WARN) elsewhere from
     * silencing the diagnostic logs in audio_play_task. */
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Audio init – DAC GPIO%d", PIN_AUDIO_DAC);

    /* Start in oneshot mode.  Idle at 0 (low rail).
     * AC-coupling means idle voltage has no bearing on silence: the cap
     * blocks DC and the LTK8002D biases its input to VDD/2 regardless.
     * Idling at 0 means mode-switch transients (0→float→0) produce no
     * voltage change at the amp input — no pop at start or end of sound. */
    dac_oneshot_config_t one_cfg = { .chan_id = DAC_CHAN_0 };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&one_cfg, &s_dac_one));
    dac_oneshot_output_voltage(s_dac_one, 0);

    /* Binary semaphore (no task-ownership enforcement).
     * A mutex requires that the same task which called xSemaphoreTake also
     * calls xSemaphoreGive — but audio_play_file() takes in the caller's task
     * while audio_play_task gives on exit, which is a different task.
     * That cross-task give triggers the FreeRTOS assert in xTaskPriorityDisinherit.
     * A binary semaphore has identical counting behaviour with no ownership check. */
    s_play_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(s_play_mutex);   /* start in "available" state */
}

void audio_play_file(const char *path)
{
    if (!path || path[0] == '\0') {
        ESP_LOGW(TAG, "audio_play_file: empty path — ignoring");
        return;
    }
    ESP_LOGI(TAG, "audio_play_file: %s", path);

    /* Only PCM WAV is supported via the built-in DAC */
    const char *ext = strrchr(path, '.');
    if (!ext || strcasecmp(ext, ".wav") != 0) {
        ESP_LOGW(TAG, "Skipping non-WAV file (DAC only supports PCM WAV): %s",
                 path);
        return;
    }

    if (!s_play_mutex) {
        ESP_LOGE(TAG, "audio_play_file: mutex NULL — audio_init() not called?");
        return;
    }

    /* Orphaned-mutex recovery: if the playback task was killed externally
     * (watchdog, stack overflow) it never reached xSemaphoreGive().
     * Detect this by checking that the task handle is NULL (task is gone)
     * while the mutex is still taken (count=0).  Force-release so the next
     * play request is not permanently blocked. */
    if (s_audio_task == NULL && uxSemaphoreGetCount(s_play_mutex) == 0) {
        ESP_LOGW(TAG, "Orphaned play mutex detected (task dead) — force-releasing");
        xSemaphoreGive(s_play_mutex);
    }

    /* Non-blocking: drop immediately if a sound is already playing.
     * audio_play_file() is called from the touch-handler and HTTP-handler
     * tasks.  The old audio_stop() + 300 ms mutex wait blocked those tasks
     * for up to 600 ms; during that window the touch poll task queued new
     * events that fired immediately on unblock, producing cascading mode
     * changes ("cycling screens").
     *
     * For click / notification sounds, dropping a concurrent request is
     * the correct behaviour.  To interrupt a playing file and start a new
     * one, call audio_stop() explicitly before audio_play_file(). */
    if (xSemaphoreTake(s_play_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "audio busy — dropping %s", path);
        return;
    }

    s_stop_flag = false;

    play_arg_t *a = (play_arg_t *)malloc(sizeof(play_arg_t));
    if (!a) {
        xSemaphoreGive(s_play_mutex);
        return;
    }
    strncpy(a->path, path, sizeof(a->path) - 1);
    a->path[sizeof(a->path) - 1] = '\0';

    BaseType_t rc = xTaskCreate(audio_play_task, "audio_play",
                                16384, a, 5, &s_audio_task);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio_play task");
        free(a);
        xSemaphoreGive(s_play_mutex);
    }
}

void audio_set_volume(int vol)
{
    if (vol < 0)   vol = 0;
    if (vol > 100) vol = 100;
    s_volume = vol;
    ESP_LOGD(TAG, "Volume set to %d%%", s_volume);
}

void audio_stop(void)
{
    s_stop_flag = true;
    /* Poll briefly for playback task to finish (it gives the mutex on exit) */
    for (int i = 0; i < 30 && s_audio_task != NULL; i++)
        vTaskDelay(pdMS_TO_TICKS(10));
    /* Only touch DAC state if the task exited cleanly.  If it timed out, it
     * is still tearing down the DAC hardware and owns the oneshot handle;
     * touching it concurrently would race with dac_cont_stop(). */
    if (s_audio_task == NULL && s_dac_one)
        dac_oneshot_output_voltage(s_dac_one, 0);
}
