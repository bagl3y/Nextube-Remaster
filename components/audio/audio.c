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
 *
 * Playback runs in a dedicated FreeRTOS task so audio_play_file() returns
 * immediately.  A mutex serialises concurrent play requests.
 *
 * DAC mode lifecycle:
 *   idle    – oneshot channel holds DAC at 128 (mid-rail = 8-bit silence).
 *             I2S/DMA hardware is completely off.  This is critical: the
 *             WS2812 RMT driver fires at 10 MHz on GPIO32 every ~50 ms
 *             (breathing effect).  With I2S/DMA off the RMT cannot cause
 *             DMA bus contention or couple noise into the DAC output.
 *
 *   playing – oneshot deleted, continuous channel streams PCM.
 *             On entry the DMA ring is primed with one full descriptor of
 *             128 (mid-rail silence) before any audio data is written.
 *             The oneshot was also at 128, so the DAC output stays at
 *             128 through the mode switch — V_cap = 0 throughout — and
 *             there is no voltage step at the amp input (no pop/chirp).
 *             On exit a 128-silence flush drains the ring to 128, then
 *             oneshot resumes at 128 — again a 128→128 non-event.
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

/* DAC handles – only one active at a time */
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
 * Transition from idle (oneshot at 128) to continuous mode.
 *
 * The ring is primed with one full descriptor of 128 (mid-rail silence)
 * before this function returns.  The hardware begins outputting 128
 * immediately — matching the oneshot level — so V_cap stays at 0 and
 * the amplifier input sees no voltage step.  Audio data queued by the
 * caller immediately follows the prime in the ring.
 */
static esp_err_t dac_cont_start(uint32_t sample_rate)
{
    if (s_dac_one) {
        dac_oneshot_del_channel(s_dac_one);
        s_dac_one = NULL;
    }

    /* Guard: clean up any orphaned continuous handle. */
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
        goto fail_restore_oneshot;
    }

    err = dac_continuous_enable(s_dac_cont);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dac_continuous_enable: %s", esp_err_to_name(err));
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
        goto fail_restore_oneshot;
    }

    /* Prime the ring with mid-rail silence so the DAC outputs 128 from
     * the very first sample — matching the oneshot idle level.
     * V_cap was 0 at idle (oneshot at 128 = VDD/2), and stays 0 now.
     * The first real audio sample (also near 128 for a silent-start WAV)
     * creates no voltage step → no pop. */
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
    return ESP_OK;

fail_restore_oneshot:
    {
        dac_oneshot_config_t one_cfg = { .chan_id = DAC_CHAN_0 };
        if (dac_oneshot_new_channel(&one_cfg, &s_dac_one) == ESP_OK)
            dac_oneshot_output_voltage(s_dac_one, 128);
    }
    return err;
}

/*
 * Transition from continuous mode back to idle (oneshot at 128).
 * Call only after the 128-silence flush + drain wait in task_cleanup,
 * so the last DAC output was 128 — the oneshot takes over at 128.
 * No voltage step, no pop.
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
            dac_oneshot_output_voltage(s_dac_one, 128);
    }
}

/* ── Volume scaling ─────────────────────────────────────────────────── */
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

    uint8_t *buf     = NULL;
    uint8_t *preload = NULL;
    size_t   preload_n = 0;
    uint32_t frame = 0, write_stalls = 0, total_bytes_out = 0;
    uint32_t dac_rate = 32000;   /* set properly after header parse */

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

    /* ── Find the 'data' sub-chunk ── */
    {
        long data_start = -1;
        fseek(f, 12, SEEK_SET);
        while (!feof(f)) {
            char     cid[4];
            uint32_t csz;
            if (fread(cid, 1, 4, f) < 4) break;
            if (fread(&csz, 1, 4, f) < 4) break;
            if (memcmp(cid, "data", 4) == 0) { data_start = ftell(f); break; }
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

    /* ── Upsample factor for low sample-rate files ─────────────────────
     * ESP32 DAC DMA minimum rate ≈ 19 608 Hz (160 MHz / (255 × 32)).
     * 8 kHz and 16 kHz files are integer-upsampled to ≥ 20 kHz. */
    uint32_t upsample = 1;
    dac_rate = hdr.sample_rate;
    while (dac_rate < 20000) { upsample <<= 1; dac_rate <<= 1; }
    if (upsample > 1)
        ESP_LOGI(TAG, "Upsampling x%u: %u Hz → %u Hz",
                 (unsigned)upsample, (unsigned)hdr.sample_rate, (unsigned)dac_rate);

    /* ── DMA window: internal SRAM ── */
    buf = (uint8_t *)heap_caps_malloc(STREAM_BUF_BYTES,
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "OOM for DMA window");
        goto task_cleanup;
    }

    /* ── PSRAM pre-buffer ───────────────────────────────────────────────
     * Load the entire WAV data chunk into PSRAM before starting the DAC.
     * Prevents SPIFFS cold-read stalls (can be 500+ ms) from draining the
     * DMA ring mid-playback and causing pops/static. */
#define PSRAM_PRELOAD_MAX  (256 * 1024)
    {
        long cur = ftell(f);
        fseek(f, 0, SEEK_END);
        long eof = ftell(f);
        fseek(f, cur, SEEK_SET);
        size_t raw_bytes = (eof > cur) ? (size_t)(eof - cur) : 0;

        if (raw_bytes > 0 && raw_bytes <= PSRAM_PRELOAD_MAX) {
            size_t post_conv  = (hdr.bits_per_sample == 16) ? raw_bytes/2 : raw_bytes;
            size_t expanded   = post_conv * upsample;
            size_t alloc_size = (raw_bytes > expanded) ? raw_bytes : expanded;

            preload = (uint8_t *)heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM);
            if (preload) {
                int64_t t_load = esp_timer_get_time();
                size_t  got    = fread(preload, 1, raw_bytes, f);
                ESP_LOGI(TAG, "PSRAM pre-load: %zu bytes in %lld ms",
                         got, (long long)((esp_timer_get_time() - t_load) / 1000));

                apply_volume(preload, (int)got, hdr.bits_per_sample, s_volume);

                int out8 = (int)got;
                if (hdr.bits_per_sample == 16)
                    out8 = pcm16_to_pcm8(preload, (int)got);

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
                ESP_LOGW(TAG, "PSRAM OOM (%zu bytes) — streaming", alloc_size);
            }
        }
    }

    /* ── Start DAC continuous (oneshot → continuous + prime ring) ── */
    if (dac_cont_start(dac_rate) != ESP_OK)
        goto task_cleanup;

    /* ── Stream PCM to DMA ── */
    {
        int64_t t_start = esp_timer_get_time();

        if (preload) {
            /* PSRAM path */
            size_t pos = 0;
            while (!s_stop_flag && pos < preload_n) {
                size_t chunk = preload_n - pos;
                if (chunk > STREAM_BUF_BYTES) chunk = STREAM_BUF_BYTES;
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
            /* SPIFFS streaming fallback */
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
                    ESP_LOGW(TAG, "DAC write error %s — stopping",
                             esp_err_to_name(werr));
                    break;
                }
                total_bytes_out += (uint32_t)written;
                frame++;
            }
        }

        ESP_LOGI(TAG, "playback done: %u frames  %u bytes  %lld ms  %u stalls",
                 frame, total_bytes_out,
                 (long long)((esp_timer_get_time() - t_start) / 1000),
                 write_stalls);
    }

task_cleanup:
    if (preload) { free(preload); preload = NULL; }

    if (buf && s_dac_cont) {
        /* Flush ring with 128 silence so the last DAC output is mid-rail.
         * dac_cont_stop() then restores oneshot at 128 — a 128→128
         * transition, no voltage step, no pop. */
        const uint32_t ring_bytes = (uint32_t)DAC_DESC_NUM * DAC_DMA_BUF_SIZE;
        memset(buf, 128, DAC_DMA_BUF_SIZE);
        size_t _fw;
        dac_continuous_write(s_dac_cont, buf, DAC_DMA_BUF_SIZE,
                             &_fw, pdMS_TO_TICKS(500));
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
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Audio init – DAC GPIO%d", PIN_AUDIO_DAC);

    /* Idle in oneshot mode at 128 (mid-rail = 8-bit silence).
     *
     * Keeping I2S/DMA hardware completely off at idle is essential:
     * the WS2812 accent LEDs use the RMT peripheral at 10 MHz, which
     * causes DMA bus contention and/or EMI coupling if I2S is also
     * running.  With I2S off, RMT transmissions are inaudible.
     *
     * Idling at 128 (not 0) means V_cap = V_DAC − VDD/2 = 0.
     * dac_cont_start() primes the ring with 128 before writing audio,
     * so the mode switch and first audio sample create no voltage step
     * at the amplifier input — no pop or chirp. */
    dac_oneshot_config_t one_cfg = { .chan_id = DAC_CHAN_0 };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&one_cfg, &s_dac_one));
    dac_oneshot_output_voltage(s_dac_one, 128);

    /* Binary semaphore – cross-task give/take pattern (play_file takes
     * in caller's task, play_task gives on exit). */
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

    const char *ext = strrchr(path, '.');
    if (!ext || strcasecmp(ext, ".wav") != 0) {
        ESP_LOGW(TAG, "Skipping non-WAV file: %s", path);
        return;
    }

    if (!s_play_mutex) {
        ESP_LOGE(TAG, "audio_play_file: mutex NULL — audio_init() not called?");
        return;
    }

    /* Orphaned-mutex recovery */
    if (s_audio_task == NULL && uxSemaphoreGetCount(s_play_mutex) == 0) {
        ESP_LOGW(TAG, "Orphaned play mutex — force-releasing");
        xSemaphoreGive(s_play_mutex);
    }

    /* Non-blocking drop if already playing */
    if (xSemaphoreTake(s_play_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "audio busy — dropping %s", path);
        return;
    }

    s_stop_flag = false;

    play_arg_t *a = (play_arg_t *)malloc(sizeof(play_arg_t));
    if (!a) { xSemaphoreGive(s_play_mutex); return; }
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
        dac_oneshot_output_voltage(s_dac_one, 128);
}
