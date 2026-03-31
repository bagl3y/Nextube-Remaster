/**
 * @file audio.c
 * @brief Nextube audio driver – WAV file playback via DAC continuous driver.
 *
 * Hardware: GPIO25 → LTK8002D amplifier (DAC_CHAN_0).
 *
 * Uses the IDF 5.x dac_continuous driver (driver/dac_continuous.h).
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
 *   always-on – the dac_continuous driver runs at DAC_ALWAYS_ON_HZ at all
 *               times, outputting 128 (mid-rail = 8-bit silence) when idle.
 *               This keeps the AC-coupling cap pre-charged to V_cap = 0
 *               (equilibrium for V_DAC = VDD/2), so when audio starts the
 *               first sample (also near 128) produces no voltage step and
 *               therefore no pop or chirp.
 *
 *   playing   – the silence_feed_task pauses; audio_play_task writes PCM
 *               data at the same 32 kHz rate.  All files are upsampled to
 *               32 kHz before writing, so the DAC rate never changes.
 *               At end-of-audio a silence flush (128) drains the ring back
 *               to idle, then silence_feed_task resumes seamlessly.
 */

#include "audio.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/dac_continuous.h"
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

/* Fixed DAC output rate.  All WAV files are upsampled to this rate so the
 * driver never needs to stop/restart (which would cause a transient pop). */
#define DAC_ALWAYS_ON_HZ   32000U

/* ── Runtime state ─────────────────────────────────────────────────── */
static int               s_volume      = 20;
static volatile bool     s_stop_flag   = false;
static TaskHandle_t      s_audio_task  = NULL;
static SemaphoreHandle_t s_play_mutex  = NULL;

/* Set true while audio_play_task owns the DMA ring; silence_feed_task
 * checks this flag and backs off to prevent interleaving. */
static volatile bool     s_audio_active = false;
static TaskHandle_t      s_silence_task = NULL;

static dac_continuous_handle_t s_dac_cont = NULL;

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

/* ── Silence feed task ──────────────────────────────────────────────── */
/*
 * Keeps the DMA ring fed with mid-rail silence (128) whenever no audio
 * task is writing to it.  This serves two purposes:
 *
 *  1. Prevents DMA ring stall: an empty ring causes a ~900 ms write-stall
 *     on the next audio request.  Feeding silence continuously keeps the
 *     ring in motion so the ISR stays active.
 *
 *  2. Maintains V_cap = 0: the AC-coupling cap between GPIO25 and the
 *     LTK8002D amplifier charges to V_cap = V_DAC − VDD/2.  With the DAC
 *     always outputting 128 (= VDD/2), V_cap stays at 0, so when audio
 *     starts (first sample ≈ 128) there is no voltage step at the amp
 *     input and therefore no pop or chirp.
 *
 * Uses non-blocking writes (timeout = 0) so audio_play_task can always
 * queue its data without competing for ring space.  The 10 ms sleep rate
 * matches the DMA drain rate (one 2048-byte descriptor ≈ 64 ms at 32 kHz),
 * keeping at most ~1 descriptor of silence ahead.
 */
static void silence_feed_task(void *arg)
{
    uint8_t *buf = (uint8_t *)heap_caps_malloc(DAC_DMA_BUF_SIZE,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "silence_feed_task: OOM");
        vTaskDelete(NULL);
        return;
    }
    memset(buf, 128, DAC_DMA_BUF_SIZE);

    while (1) {
        if (s_dac_cont && !s_audio_active) {
            size_t w = 0;
            /* Non-blocking: if ring has space, write instantly.
             * If ring is full (audio just finished filling it), skip this
             * cycle — audio data is already queued and plays correctly. */
            dac_continuous_write(s_dac_cont, buf, DAC_DMA_BUF_SIZE,
                                 &w, /*timeout_ms=*/0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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

    /* ── Claim the DMA ring ─────────────────────────────────────────────
     * Set s_audio_active BEFORE writing any audio data so silence_feed_task
     * backs off immediately.  The flag is cleared AFTER the end-of-audio
     * silence flush and drain wait (in task_cleanup) so the silence task
     * never interleaves with audio or the drain tail. */
    s_audio_active = true;

    if (!s_dac_cont) {
        ESP_LOGE(TAG, "DAC not running — audio_init() not called?");
        goto task_cleanup;
    }

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
        /* ── End-of-audio silence flush ──────────────────────────────────
         * Write one descriptor of mid-rail silence (128) to let the audio
         * tail drain cleanly, then wait for the ring to empty.
         * s_audio_active stays true until AFTER the drain so silence_feed_task
         * cannot interleave silence into the still-draining audio tail. */
        const uint32_t ring_bytes = (uint32_t)DAC_DESC_NUM * DAC_DMA_BUF_SIZE;
        memset(buf, 128, DAC_DMA_BUF_SIZE);
        size_t _fw;
        dac_continuous_write(s_dac_cont, buf, DAC_DMA_BUF_SIZE, &_fw,
                             pdMS_TO_TICKS(500));

        uint32_t ring_occ = (total_bytes_out < ring_bytes) ? total_bytes_out : ring_bytes;
        vTaskDelay(pdMS_TO_TICKS(ring_occ * 1000 / dac_rate + 150));
    }
    free(buf);

    /* Release the ring back to silence_feed_task.  Done AFTER the drain
     * wait so the silence task never touches the ring mid-drain. */
    s_audio_active = false;

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
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Audio init – DAC GPIO%d  always-on @ %u Hz",
             PIN_AUDIO_DAC, (unsigned)DAC_ALWAYS_ON_HZ);

    /* Start the DAC in continuous mode immediately and never stop it.
     *
     * The AC-coupling cap between GPIO25 and the LTK8002D amplifier charges
     * to V_cap = V_DAC − VDD/2.  With the DAC always outputting 128 (VDD/2),
     * V_cap stays at 0 (equilibrium).  When audio starts, the first sample
     * is also near 128, so V_amp_input = 128/255×VDD − 0 = VDD/2 = silence
     * — no voltage step, no pop or chirp.
     *
     * silence_feed_task keeps the DMA ring fed with 128 at idle, preventing
     * the ring-stall that causes ~900 ms write latency on the next write. */
    dac_continuous_config_t cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num  = DAC_DESC_NUM,
        .buf_size  = DAC_DMA_BUF_SIZE,
        .freq_hz   = DAC_ALWAYS_ON_HZ,
        .clk_src   = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode  = DAC_CHANNEL_MODE_SIMUL,
    };
    ESP_ERROR_CHECK(dac_continuous_new_channels(&cfg, &s_dac_cont));
    ESP_ERROR_CHECK(dac_continuous_enable(s_dac_cont));

    /* Immediately prime the ring with one silence descriptor so the DAC
     * outputs 128 from the first moment and V_cap begins settling. */
    {
        uint8_t *prime = (uint8_t *)heap_caps_malloc(DAC_DMA_BUF_SIZE,
                                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (prime) {
            memset(prime, 128, DAC_DMA_BUF_SIZE);
            size_t w;
            dac_continuous_write(s_dac_cont, prime, DAC_DMA_BUF_SIZE,
                                 &w, pdMS_TO_TICKS(200));
            free(prime);
        }
    }

    /* Start silence_feed_task at priority 4 (below audio task priority 5). */
    xTaskCreate(silence_feed_task, "audio_silence", 2048, NULL, 4, &s_silence_task);

    /* Binary semaphore – see note in audio_play_file(). */
    s_play_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(s_play_mutex);
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
    /* DAC stays running; silence_feed_task resumes once s_audio_active
     * is cleared by the play task's cleanup path. */
}
