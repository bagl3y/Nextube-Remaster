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
 *             voltage step occurs at the amp input on mode change.
 *   playing – oneshot deleted, continuous channel streams PCM samples;
 *             DMA prime ramp (0→128) and end ramp (128→0) handle the
 *             transitions at audio sample rate.
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
#include <math.h>

static const char *TAG = "audio";

/* ── Runtime state ─────────────────────────────────────────────────── */
static int               s_volume      = 20;
static volatile bool     s_stop_flag   = false;
static TaskHandle_t      s_audio_task  = NULL;
static SemaphoreHandle_t s_play_mutex  = NULL;

/* DAC handles – only one is active at a time */
static dac_continuous_handle_t s_dac_cont = NULL;   /* during playback */
static dac_oneshot_handle_t    s_dac_one  = NULL;   /* during idle     */

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
    if (s_dac_one) {
        dac_oneshot_del_channel(s_dac_one);
        s_dac_one = NULL;
    }

    /* Guard: clean up any orphaned continuous handle left by a previous
     * task that was killed before reaching dac_cont_stop(). */
    if (s_dac_cont) {
        ESP_LOGW(TAG, "Orphaned dac_cont handle — cleaning up");
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
        dac_oneshot_config_t one_cfg = { .chan_id = DAC_CHAN_0 };
        if (dac_oneshot_new_channel(&one_cfg, &s_dac_one) == ESP_OK)
            dac_oneshot_output_voltage(s_dac_one, 0);
        return err;
    }

    err = dac_continuous_enable(s_dac_cont);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dac_continuous_enable: %s", esp_err_to_name(err));
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
        dac_oneshot_config_t one_cfg = { .chan_id = DAC_CHAN_0 };
        if (dac_oneshot_new_channel(&one_cfg, &s_dac_one) == ESP_OK)
            dac_oneshot_output_voltage(s_dac_one, 0);
        return err;
    }

    /* No prime ramp.
     *
     * The DMA ring is zero-initialised.  With oneshot idling at 0,
     * V_cap = −VDD/2.  The DMA starting at 0 is the same voltage as the
     * oneshot — V_amp_in = 0 − (−VDD/2) = VDD/2 = silence, no transient.
     *
     * The first audio sample (≈128 for a click WAV that starts at silence)
     * creates a single-sample step (31 µs at 32 kHz, fundamental ≈ 16 kHz).
     * At that frequency the ear is far less sensitive than the 7 Hz chirp
     * produced by the old 64 ms ramp, and the transient lands on the click's
     * own attack — perceptually masked by the sound itself. */
    return ESP_OK;
}

/*
 * Transition from continuous mode back to idle (oneshot at 0).
 */
static void dac_cont_stop(void)
{
    if (s_dac_cont) {
        dac_continuous_disable(s_dac_cont);
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
    }
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

    uint8_t *buf     = NULL;   /* internal-SRAM DMA window              */
    uint8_t *preload = NULL;   /* PSRAM pre-load buffer (NULL = stream) */
    size_t   preload_n = 0;    /* processed byte count in preload buf   */
    /* Declared here so task_cleanup can access them via any goto path. */
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

    /* ── DMA window: internal SRAM transfer buffer ──────────────────────
     * The DAC DMA controller cannot access PSRAM.  PCM data is moved from
     * the PSRAM pre-load buffer (or directly from SPIFFS) into this
     * STREAM_BUF_BYTES internal-SRAM window before each dac_continuous_write
     * call.  DMA-capability flag is set for safety even though the driver
     * copies data into its own DMA buffers. */
    buf = (uint8_t *)heap_caps_malloc(STREAM_BUF_BYTES,
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "OOM for DMA window (internal SRAM)");
        goto task_cleanup;
    }

    /* ── PSRAM pre-buffer ───────────────────────────────────────────────
     * Root cause of DMA ring starvation (heard as static/pops):
     *   SPIFFS cold-cache reads can take 500+ ms per chunk (first access
     *   after mounting or a long idle).  The DMA ring at 32 kHz holds only
     *   ~512 ms of audio.  A 577 ms fread stall drains the ring completely,
     *   causing the DAC to output undefined values → audible pop/static.
     *
     * Fix: load the entire data chunk into PSRAM before writing anything
     * to the DMA ring.  All SPIFFS I/O happens before DAC output begins,
     * so no mid-stream stall can drain the ring.
     *
     * Applies to files ≤ PSRAM_PRELOAD_MAX (256 KB).  click.wav and other
     * notification sounds are typically ≤ 64 KB and always pre-buffered.
     * Larger files fall back to the streaming path below.
     *
     * alloc_size = max(raw_bytes, expanded_bytes) to ensure the buffer is
     * large enough to hold the raw data before in-place conversion:
     *   8-bit  source: expanded = raw_bytes × upsample
     *   16-bit source: raw pcm16→pcm8 halves size, then × upsample
     *                  expanded = (raw_bytes/2) × upsample
     * The raw_bytes > expanded case only occurs for 16-bit files at rates
     * ≥ 20 kHz where upsample = 1 and expanded = raw_bytes/2. */
#define PSRAM_PRELOAD_MAX  (256 * 1024)

    {
        long cur = ftell(f);
        fseek(f, 0, SEEK_END);
        long eof = ftell(f);
        fseek(f, cur, SEEK_SET);
        size_t raw_bytes = (eof > cur) ? (size_t)(eof - cur) : 0;

        if (raw_bytes > 0 && raw_bytes <= PSRAM_PRELOAD_MAX) {
            size_t post_conv   = (hdr.bits_per_sample == 16)
                                 ? raw_bytes / 2 : raw_bytes;
            size_t expanded    = post_conv * upsample;
            size_t alloc_size  = (raw_bytes > expanded) ? raw_bytes : expanded;

            preload = (uint8_t *)heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM);
            if (preload) {
                /* ── Load entire data chunk from SPIFFS ── */
                int64_t t_load = esp_timer_get_time();
                size_t  got    = fread(preload, 1, raw_bytes, f);
                ESP_LOGI(TAG, "PSRAM pre-load: %zu bytes in %lld ms",
                         got, (long long)((esp_timer_get_time() - t_load) / 1000));

                /* Volume attenuation before bit-depth conversion */
                apply_volume(preload, (int)got, hdr.bits_per_sample, s_volume);

                /* 16-bit → 8-bit unsigned in-place */
                int out8 = (int)got;
                if (hdr.bits_per_sample == 16)
                    out8 = pcm16_to_pcm8(preload, (int)got);

                /* Nearest-neighbour upsample in-place (back-to-front) */
                if (upsample > 1) {
                    for (int i = out8 - 1; i >= 0; i--) {
                        uint8_t sv = preload[i];
                        for (uint32_t j = 0; j < upsample; j++)
                            preload[(uint32_t)i * upsample + j] = sv;
                    }
                    out8 *= (int)upsample;
                }
                preload_n = (size_t)out8;
            } else {
                ESP_LOGW(TAG, "PSRAM pre-load OOM (%zu bytes) — streaming", alloc_size);
            }
        }
    }

    /* ── Ensure DAC continuous is running at the (possibly upsampled) rate ──
     * Common case: driver already at 32 kHz → no-op, no transient.
     * Rate change → brief restart with 0→128 ramp (handled inside dac_cont_start). */
    if (dac_cont_start(dac_rate) != ESP_OK)
        goto task_cleanup;

    /* ── Stream PCM to DMA ──────────────────────────────────────────────
     * Primary path: serve from PSRAM pre-buffer (no SPIFFS during output).
     * Fallback: stream directly from SPIFFS (large files or PSRAM OOM). */
    {
        int64_t t_start = esp_timer_get_time();

        if (preload) {
            /* ── PSRAM path ── */
            size_t pos = 0;
            while (!s_stop_flag && pos < preload_n) {
                size_t chunk = preload_n - pos;
                if (chunk > STREAM_BUF_BYTES) chunk = STREAM_BUF_BYTES;
                /* Copy PSRAM → internal SRAM window (DMA reads from here) */
                memcpy(buf, preload + pos, chunk);
                pos += chunk;

                size_t written = 0;
                int64_t t_wr = esp_timer_get_time();
                esp_err_t werr = dac_continuous_write(s_dac_cont, buf, chunk,
                                                      &written, pdMS_TO_TICKS(1000));
                int64_t wr_us = esp_timer_get_time() - t_wr;
                if (wr_us > 700000) {
                    write_stalls++;
                    ESP_LOGW(TAG, "DAC write stall: %lld ms (frame %u)",
                             (long long)(wr_us / 1000), frame);
                }
                if (werr != ESP_OK) {
                    ESP_LOGW(TAG, "DAC write error %s", esp_err_to_name(werr));
                    break;
                }
                total_bytes_out += (uint32_t)written;
                frame++;
            }
            free(preload);
            preload = NULL;
        } else {
            /* ── SPIFFS streaming fallback (large files) ── */
            const size_t read_size = STREAM_BUF_BYTES / upsample;
            while (!s_stop_flag) {
                int64_t t_rd = esp_timer_get_time();
                int rd = (int)fread(buf, 1, read_size, f);
                int64_t rd_us = esp_timer_get_time() - t_rd;
                if (rd <= 0) break;
                if (rd_us > 250000)
                    ESP_LOGW(TAG, "fread slow: %lld ms (frame %u)",
                             (long long)(rd_us / 1000), frame);

                apply_volume(buf, rd, hdr.bits_per_sample, s_volume);

                int out_bytes = rd;
                if (hdr.bits_per_sample == 16)
                    out_bytes = pcm16_to_pcm8(buf, rd);

                if (upsample > 1) {
                    for (int i = out_bytes - 1; i >= 0; i--) {
                        uint8_t sv = buf[i];
                        for (uint32_t j = 0; j < upsample; j++)
                            buf[(uint32_t)i * upsample + j] = sv;
                    }
                    out_bytes *= (int)upsample;
                }

                size_t written = 0;
                int64_t t_wr = esp_timer_get_time();
                esp_err_t werr = dac_continuous_write(s_dac_cont, buf,
                                                      (size_t)out_bytes,
                                                      &written, pdMS_TO_TICKS(1000));
                int64_t wr_us = esp_timer_get_time() - t_wr;
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
        }

        int64_t elapsed_ms = (esp_timer_get_time() - t_start) / 1000;
        ESP_LOGI(TAG, "playback done: %u frames  %u bytes  %lld ms  %u stalls",
                 frame, total_bytes_out, (long long)elapsed_ms, write_stalls);
    }

task_cleanup:
    if (preload) { free(preload); preload = NULL; }

    if (buf && s_dac_cont) {
        /* ── End-of-playback fade ─────────────────────────────────────────
         * Append a 128→0 ramp so the last output value is 0, matching the
         * RTC register (idle = 0).  dac_cont_stop() then switches DAC from
         * digi-bypass to the RTC register — a 0→0 transition, silent.
         * Ramp = DAC_DMA_BUF_SIZE samples = one full descriptor. */
        const uint32_t ring_bytes = (uint32_t)DAC_DESC_NUM * DAC_DMA_BUF_SIZE;
        for (size_t i = 0; i < DAC_DMA_BUF_SIZE; i++)
            buf[i] = (uint8_t)(128 * (DAC_DMA_BUF_SIZE - 1 - i) / (DAC_DMA_BUF_SIZE - 1));
        size_t _fw;
        dac_continuous_write(s_dac_cont, buf, DAC_DMA_BUF_SIZE, &_fw,
                             pdMS_TO_TICKS(500));

        uint32_t ring_occ = (total_bytes_out < ring_bytes) ? total_bytes_out : ring_bytes;
        vTaskDelay(pdMS_TO_TICKS(ring_occ * 1000 / dac_rate + 150));
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
     * voltage change at the amp input — no pop at start or end of sound.
     * Crucially, the I2S/DMA hardware is completely off at idle, so there
     * is no switching noise coupling into the DAC analog output. */
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
    for (int i = 0; i < 30 && s_audio_task != NULL; i++)
        vTaskDelay(pdMS_TO_TICKS(10));
    if (s_audio_task == NULL && s_dac_one)
        dac_oneshot_output_voltage(s_dac_one, 0);
}
