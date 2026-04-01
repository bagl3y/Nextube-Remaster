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
 *   idle    – GPIO25 configured as Hi-Z input (GPIO_MODE_INPUT).
 *             The DAC analog output buffer is completely powered down.
 *             WS2812 current spikes on the shared 3.3 V rail have no
 *             coupling path into the amp — only thermal noise remains.
 *             The amp's internal bias resistors hold its input at VDD/2
 *             through the AC cap; V_cap = 0.
 *
 *   playing – leds_set_audio_active(true) pauses WS2812 RMT first.
 *             A 15 ms oneshot-at-128 pre-charges the AC cap to V_cap = 0
 *             while the rail is quiet.  GPIO25 is then reclaimed by
 *             dac_continuous_new_channels().  A flat-128 prime buffer
 *             (pre-filled before enable) is written immediately after
 *             dac_continuous_enable() to minimise the DMA zero-init
 *             window.  With V_cap = 0 and DAC at 128, V_amp_in = VDD/2
 *             = silence throughout the transition — no pop, no chirp.
 *             On exit the ring is flushed with 128, DAC stopped, GPIO25
 *             returns to Hi-Z, and LEDs resume.
 */

#include "audio.h"
#include "leds.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/dac_continuous.h"
#include "driver/dac_oneshot.h"
#include "driver/gpio.h"
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
    if (s_dac_cont) {
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
    if (err != ESP_OK) return err;

    err = dac_continuous_enable(s_dac_cont);
    if (err != ESP_OK) {
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
        return err;
    }

    /* Smooth fade-in from 0 to 128 (VDD/2). */
    size_t fade_samples = (sample_rate * 20) / 1000; 
    fade_samples = (fade_samples + 3) & ~3; /* Force 4-byte alignment */
    
    uint8_t *fade_buf = (uint8_t *)heap_caps_malloc(fade_samples, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (fade_buf) {
        for (size_t i = 0; i < fade_samples; i++) {
            fade_buf[i] = (uint8_t)((i * 128) / fade_samples);
        }
        size_t w;
        dac_continuous_write(s_dac_cont, fade_buf, fade_samples, &w, pdMS_TO_TICKS(200));
        free(fade_buf);
    }

    return ESP_OK;
}

static void dac_cont_stop(void)
{
    if (s_dac_cont) {
        dac_continuous_disable(s_dac_cont);
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
    }
    
    /* Drive pin LOW (0V) instead of Hi-Z. 
     * This provides a low-impedance path to ground that rejects 
     * WS2812/WiFi EMI crosstalk, killing the static floor. */
    gpio_reset_pin(PIN_AUDIO_DAC);
    gpio_set_direction(PIN_AUDIO_DAC, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_AUDIO_DAC, 0);
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

    /* ── Pause LED RMT before starting DAC ─────────────────────────────
     * WS2812 current spikes on the 3.3 V rail couple into the DAC output.
     * Pausing RMT stops all transmissions; LEDs hold their last colour. */
    leds_set_audio_active(true);

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
                
                chunk &= ~3; 
                if (chunk == 0) break;
                
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
                
                out_bytes &= ~3;
                if (out_bytes == 0) break;

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
        /* Fade-out from 128 to 0 to gracefully discharge the AC cap */
        size_t fade_samples = (dac_rate * 20) / 1000;
        fade_samples = (fade_samples + 3) & ~3; /* Force 4-byte alignment */
        uint8_t *fade_buf = (uint8_t *)heap_caps_malloc(fade_samples, MALLOC_CAP_INTERNAL);
        if (fade_buf) {
            for (size_t i = 0; i < fade_samples; i++) {
                fade_buf[i] = (uint8_t)(128 - ((i * 128) / fade_samples));
            }
            size_t w;
            dac_continuous_write(s_dac_cont, fade_buf, fade_samples, &w, pdMS_TO_TICKS(200));
            free(fade_buf);
        }
        
        /* Wait briefly for DMA to physically output the fade buffer */
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    free(buf);
    dac_cont_stop();

    /* Resume LED RMT now that DAC is back to 0V driven idle. */
    leds_set_audio_active(false);

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

    /* Idle with DAC completely off — GPIO25 Hi-Z (input).
     *
     * With the DAC analog output buffer powered down, WS2812 current
     * spikes on the shared 3.3 V rail have no path through the DAC
     * buffer into the LTK8002D amplifier.  The amp's internal bias
     * resistors hold its input at VDD/2 through the AC coupling cap,
     * so the amp output is silent.
     *
     * dac_cont_start() reconfigures GPIO25 for DAC use and primes the
     * DMA ring with 128 (VDD/2) before writing audio.  V_cap = 0 at
     * idle (both sides of the cap at VDD/2), so the first DAC sample
     * at 128 creates no voltage step — no pop. */
    gpio_reset_pin(PIN_AUDIO_DAC);
    gpio_set_direction(PIN_AUDIO_DAC, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_AUDIO_DAC, 0);

    /* Binary semaphore – cross-task give/take pattern (play_file takes
     * in caller's task, play_task gives on exit). */
    s_play_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(s_play_mutex);

    /* ── DAC warm-up ────────────────────────────────────────────────────
     * The first call to dac_continuous_new_channels() + enable() pays a
     * one-time cost: I2S peripheral reset, APLL lock, DMA init (~1.5 s
     * on ESP32).  Doing a dummy start/stop here at boot hides that cost
     * at startup (silent, inaudible) rather than on the first button press
     * where it would delay audio and cause a prolonged DMA write stall. */
    ESP_LOGI(TAG, "DAC warm-up...");
    dac_continuous_handle_t warmup = NULL;
    dac_continuous_config_t wcfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num  = 2,
        .buf_size  = 512,
        .freq_hz   = 32000,
        .clk_src   = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode  = DAC_CHANNEL_MODE_SIMUL,
    };
    if (dac_continuous_new_channels(&wcfg, &warmup) == ESP_OK) {
        if (dac_continuous_enable(warmup) == ESP_OK) {
            uint8_t silence[512];
            memset(silence, 128, sizeof(silence));
            size_t w;
            dac_continuous_write(warmup, silence, sizeof(silence), &w, pdMS_TO_TICKS(2000));
            dac_continuous_disable(warmup);
        }
        dac_continuous_del_channels(warmup);
    }
    /* Return GPIO25 to actively driven LOW after warm-up */
    gpio_reset_pin(PIN_AUDIO_DAC);
    gpio_set_direction(PIN_AUDIO_DAC, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_AUDIO_DAC, 0);
    ESP_LOGI(TAG, "DAC warm-up done");
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
    /* Ensure DAC is off if task exited without reaching dac_cont_stop() */
    if (s_audio_task == NULL && !s_dac_cont && !s_dac_one)
        gpio_set_direction(PIN_AUDIO_DAC, GPIO_MODE_INPUT);
}
