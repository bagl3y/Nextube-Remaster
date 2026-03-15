/**
 * @file audio.c
 * @brief Nextube audio driver – WAV file playback via I2S built-in DAC
 *
 * Hardware: GPIO25 → LTK8002D amplifier (DAC_CHANNEL_1 / I2S_DAC_CHANNEL_RIGHT_EN).
 *
 * Supports standard PCM WAV files (8-bit or 16-bit, mono or stereo).
 * MP3 files are logged but skipped – the built-in DAC requires raw PCM.
 *
 * Playback runs in a dedicated FreeRTOS task so audio_play_file() returns
 * immediately.  A mutex serialises concurrent play requests.
 */

#include "audio.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/dac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <strings.h>    /* strcasecmp */

static const char *TAG = "audio";

/* ── Runtime state ─────────────────────────────────────────────────── */
static int               s_volume     = 20;
static volatile bool     s_stop_flag  = false;
static TaskHandle_t      s_audio_task = NULL;
static SemaphoreHandle_t s_play_mutex = NULL;

/* ── I2S configuration ─────────────────────────────────────────────── */
#define I2S_PORT        I2S_NUM_0
#define DMA_BUF_COUNT   8
#define DMA_BUF_LEN     512     /* samples per DMA buffer */
#define STREAM_BUF_BYTES 4096   /* file read chunk size   */

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
    uint32_t byte_rate;         /* sample_rate * block_align */
    uint16_t block_align;       /* channels * bits/8  */
    uint16_t bits_per_sample;   /* 8 or 16            */
} wav_riff_hdr_t;

/* ── I2S / DAC lifecycle ────────────────────────────────────────────── */
static esp_err_t i2s_dac_init(uint32_t sample_rate, uint16_t bits)
{
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX |
                                             I2S_MODE_DAC_BUILT_IN),
        .sample_rate          = sample_rate,
        .bits_per_sample      = (bits == 8) ? I2S_BITS_PER_SAMPLE_8BIT
                                            : I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,
        .intr_alloc_flags     = 0,
    };
    esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_driver_install: %s", esp_err_to_name(err));
        return err;
    }
    /* Route I2S right channel to DAC1 (GPIO25) */
    i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
    i2s_zero_dma_buffer(I2S_PORT);
    return ESP_OK;
}

static void i2s_dac_deinit(void)
{
    i2s_zero_dma_buffer(I2S_PORT);
    i2s_driver_uninstall(I2S_PORT);
    /* Leave DAC output at mid-rail to avoid click/pop */
    dac_output_voltage(DAC_CHANNEL_1, 128);
}

/* ── Volume scaling ─────────────────────────────────────────────────── */
/*
 * In-place volume attenuation.
 * 16-bit samples are signed; 8-bit WAV samples are unsigned (centre = 128).
 */
static void apply_volume(uint8_t *buf, int len_bytes,
                         uint16_t bits_per_sample, int vol_pct)
{
    if (vol_pct >= 100) return;
    const float scale = vol_pct / 100.0f;

    if (bits_per_sample == 16) {
        int16_t *s = (int16_t *)(void *)buf;
        int      n = len_bytes / 2;
        for (int i = 0; i < n; i++) {
            s[i] = (int16_t)((float)s[i] * scale);
        }
    } else {
        /* 8-bit unsigned PCM */
        for (int i = 0; i < len_bytes; i++) {
            buf[i] = (uint8_t)(128 + (int)(((int)buf[i] - 128) * scale));
        }
    }
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

    /* ── Open file ── */
    FILE *f = fopen(path, "r");
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
    /*
     * Position just after the fixed fmt chunk (12 bytes RIFF+size+WAVE,
     * then 8 bytes chunk header + fmt_size bytes fmt body).
     */
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
        /* Skip non-data chunk (word-aligned) */
        fseek(f, (long)(csz + (csz & 1)), SEEK_CUR);
    }
    if (data_start < 0) {
        ESP_LOGE(TAG, "No 'data' chunk found in: %s", path);
        goto task_close;
    }
    fseek(f, data_start, SEEK_SET);

    ESP_LOGI(TAG, "WAV play: %s  %u Hz  %u ch  %u-bit  vol=%d%%",
             path, (unsigned)hdr.sample_rate, hdr.num_channels,
             hdr.bits_per_sample, s_volume);

    /* ── Configure I2S/DAC ── */
    if (i2s_dac_init(hdr.sample_rate, hdr.bits_per_sample) != ESP_OK)
        goto task_close;

    /* ── Stream PCM data ── */
    uint8_t *buf = (uint8_t *)malloc(STREAM_BUF_BYTES);
    if (!buf) {
        ESP_LOGE(TAG, "OOM for audio stream buffer");
        i2s_dac_deinit();
        goto task_close;
    }

    while (!s_stop_flag) {
        int rd = fread(buf, 1, STREAM_BUF_BYTES, f);
        if (rd <= 0) break;

        apply_volume(buf, rd, hdr.bits_per_sample, s_volume);

        size_t written = 0;
        /* i2s_write blocks until DMA accepts the data */
        i2s_write(I2S_PORT, buf, (size_t)rd, &written,
                  pdMS_TO_TICKS(1000));
    }

    free(buf);
    i2s_dac_deinit();

task_close:
    fclose(f);
task_exit:
    xSemaphoreGive(s_play_mutex);
    s_audio_task = NULL;
    vTaskDelete(NULL);
}

/* ════════════════════════════════════════════════════════════════════ */
/*  Public API                                                          */
/* ════════════════════════════════════════════════════════════════════ */

void audio_init(void)
{
    ESP_LOGI(TAG, "Audio init – DAC GPIO%d", PIN_AUDIO_DAC);
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, 128);   /* silence */
    s_play_mutex = xSemaphoreCreateMutex();
}

void audio_play_file(const char *path)
{
    if (!path || path[0] == '\0') return;

    /* Only PCM WAV is supported via the built-in DAC */
    const char *ext = strrchr(path, '.');
    if (!ext || strcasecmp(ext, ".wav") != 0) {
        ESP_LOGW(TAG, "Skipping non-WAV file (DAC only supports PCM WAV): %s",
                 path);
        return;
    }

    /* Stop any running playback first */
    audio_stop();

    /* Acquire play lock (previous task releases it on exit) */
    if (xSemaphoreTake(s_play_mutex, pdMS_TO_TICKS(300)) != pdTRUE) {
        ESP_LOGW(TAG, "audio busy – play request dropped");
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
                                4096, a, 5, &s_audio_task);
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
    /* Poll briefly for task to finish (it gives the mutex on exit) */
    for (int i = 0; i < 30 && s_audio_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    /* Guarantee silence on DAC output */
    dac_output_voltage(DAC_CHANNEL_1, 128);
}
