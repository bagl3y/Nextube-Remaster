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
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <strings.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "audio";

/* ── Runtime state ─────────────────────────────────────────────────── */
static int               s_volume      = 20;
static volatile bool     s_stop_flag   = false;
static TaskHandle_t      s_audio_task  = NULL;
static SemaphoreHandle_t s_play_mutex  = NULL;

/* DAC handle - Always On */
static dac_continuous_handle_t s_dac_cont = NULL;

/* ── Buffer / DMA sizes ─────────────────────────────────────────────── */
#define FIXED_DAC_RATE     32000
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

    uint8_t *buf     = NULL;
    uint8_t *preload = NULL;
    size_t   preload_n = 0;
    uint32_t frame = 0, total_bytes_out = 0;

    FILE *f = fopen(path, "rb");
    if (!f) goto task_exit;

    wav_riff_hdr_t hdr;
    if (fread(&hdr, 1, sizeof(hdr), f) < (int)sizeof(hdr)) goto task_close;
    if (memcmp(hdr.riff_id, "RIFF", 4) != 0 || memcmp(hdr.wave_id, "WAVE", 4) != 0) goto task_close;
    if (hdr.audio_format != 1) goto task_close;

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
        if (data_start < 0) goto task_close;
        fseek(f, data_start, SEEK_SET);
    }

    /* ── Upsample factor for low sample-rate files ─────────────────────
     * ESP32 DAC DMA minimum rate ≈ 19 608 Hz (160 MHz / (255 × 32)).
     * 8 kHz and 16 kHz files are integer-upsampled to ≥ 20 kHz. */
    uint32_t upsample = 1;
    if (hdr.sample_rate > 0 && FIXED_DAC_RATE >= hdr.sample_rate) {
        upsample = FIXED_DAC_RATE / hdr.sample_rate;
    }
    if (upsample < 1) upsample = 1;

    /* ── DMA window: internal SRAM ── */
    buf = (uint8_t *)heap_caps_malloc(STREAM_BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!buf) goto task_cleanup;

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
                size_t got = fread(preload, 1, raw_bytes, f);
                apply_volume(preload, (int)got, hdr.bits_per_sample, s_volume);

                int out8 = (int)got;
                if (hdr.bits_per_sample == 16) out8 = pcm16_to_pcm8(preload, (int)got);

                if (upsample > 1) {
                    for (int i = out8 - 1; i >= 0; i--) {
                        uint8_t sv = preload[i];
                        for (uint32_t j = 0; j < upsample; j++)
                            preload[(uint32_t)i * upsample + j] = sv;
                    }
                    out8 *= (int)upsample;
                }
                preload_n = (size_t)out8;
            }
        }
    }

    /* ── Pause LED RMT before starting DAC ─────────────────────────────
     * WS2812 current spikes on the 3.3 V rail couple into the DAC output.
     * Pausing RMT stops all transmissions; LEDs hold their last colour. */
    leds_set_audio_active(true);

    /* Stream PCM to the perpetually running DMA */
    {
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
                esp_err_t werr = dac_continuous_write(s_dac_cont, buf, chunk,
                                                      &written, pdMS_TO_TICKS(1000));
                if (werr != ESP_OK) break;
                total_bytes_out += (uint32_t)written;
                frame++;
            }
            free(preload);
            preload = NULL;
        } else {
            /* SPIFFS streaming fallback */
            const size_t read_size = STREAM_BUF_BYTES / upsample;
            while (!s_stop_flag) {
                int rd = (int)fread(buf, 1, read_size, f);
                if (rd <= 0) break;

                apply_volume(buf, rd, hdr.bits_per_sample, s_volume);

                int out_bytes = rd;
                if (hdr.bits_per_sample == 16) out_bytes = pcm16_to_pcm8(buf, rd);

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
                esp_err_t werr = dac_continuous_write(s_dac_cont, buf,
                                                      (size_t)out_bytes,
                                                      &written, pdMS_TO_TICKS(1000));
                if (werr != ESP_OK) break;
                total_bytes_out += (uint32_t)written;
                frame++;
            }
        }
    }

task_cleanup:
    if (preload) { free(preload); preload = NULL; }

    if (buf && s_dac_cont) {
        /* Flush ring with pure silence (128) to safely drain audio 
         * and leave the DMA perfectly resting at mid-rail. */
        memset(buf, 128, STREAM_BUF_BYTES);
        size_t w;
        for (int i = 0; i < DAC_DESC_NUM; i++) {
            dac_continuous_write(s_dac_cont, buf, DAC_DMA_BUF_SIZE, &w, pdMS_TO_TICKS(200));
        }
    }
    
    free(buf);
    leds_set_audio_active(false);

task_close:
    fclose(f);
task_exit:
    xSemaphoreGive(s_play_mutex);
    s_audio_task = NULL;
    vTaskDelete(NULL);
}

/* ════════════════════════════════════════════════════════════════════ */
/* Public API                                                          */
/* ════════════════════════════════════════════════════════════════════ */

void audio_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Audio init – DAC Always-On (32kHz)");

    s_play_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(s_play_mutex);

    dac_continuous_config_t cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num  = DAC_DESC_NUM,
        .buf_size  = DAC_DMA_BUF_SIZE,
        .freq_hz   = FIXED_DAC_RATE,
        .clk_src   = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode = DAC_CHANNEL_MODE_SIMUL,
    };
    
    if (dac_continuous_new_channels(&cfg, &s_dac_cont) == ESP_OK) {
        dac_continuous_enable(s_dac_cont);
        
        /* Anti-Pop Boot Fade: Smoothly shift the DC bias from 0V to 1.65V
         * over 500ms using a sub-audible 1Hz S-Curve. */
        size_t fade_samples = (FIXED_DAC_RATE * 500) / 1000;
        fade_samples = (fade_samples + 3) & ~3;
        
        uint8_t *boot_fade = (uint8_t *)calloc(1, fade_samples);
        if (boot_fade) {
            for (size_t i = 0; i < fade_samples; i++) {
                float t = (float)i / (float)fade_samples;
                boot_fade[i] = (uint8_t)(64.0f * (1.0f - cosf(t * (float)M_PI)));
            }
            size_t w;
            dac_continuous_write(s_dac_cont, boot_fade, fade_samples, &w, portMAX_DELAY);
            free(boot_fade);
        }
        
        /* Pre-fill the rest of the DMA ring with silence */
        uint8_t silence[DAC_DMA_BUF_SIZE];
        memset(silence, 128, sizeof(silence));
        size_t w;
        for (int i = 0; i < DAC_DESC_NUM; i++) {
            dac_continuous_write(s_dac_cont, silence, sizeof(silence), &w, portMAX_DELAY);
        }
    }
}

void audio_play_file(const char *path)
{
    if (!path || path[0] == '\0') return;

    const char *ext = strrchr(path, '.');
    if (!ext || strcasecmp(ext, ".wav") != 0) return;

    if (!s_play_mutex) return;

    if (s_audio_task == NULL && uxSemaphoreGetCount(s_play_mutex) == 0) {
        xSemaphoreGive(s_play_mutex);
    }

    if (xSemaphoreTake(s_play_mutex, 0) != pdTRUE) return;

    s_stop_flag = false;

    play_arg_t *a = (play_arg_t *)malloc(sizeof(play_arg_t));
    if (!a) { xSemaphoreGive(s_play_mutex); return; }
    strncpy(a->path, path, sizeof(a->path) - 1);
    a->path[sizeof(a->path) - 1] = '\0';

    if (xTaskCreate(audio_play_task, "audio_play", 16384, a, 5, &s_audio_task) != pdPASS) {
        free(a);
        xSemaphoreGive(s_play_mutex);
    }
}

void audio_set_volume(int vol)
{
    if (vol < 0)   vol = 0;
    if (vol > 100) vol = 100;
    s_volume = vol;
}

void audio_stop(void)
{
    s_stop_flag = true;
    for (int i = 0; i < 30 && s_audio_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
