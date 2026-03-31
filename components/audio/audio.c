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
 * DAC lifecycle – always-on continuous driver:
 *   The dac_continuous driver is started once in audio_init() and NEVER
 *   stopped.  Between sounds the DMA ring loops silence (value 128 = mid-rail).
 *
 *   Why always-on?
 *     The LTK8002D is AC-coupled.  The coupling cap charges to:
 *       V_cap = V_DAC_avg − VDD/2
 *     Idling the DAC at 128 (= VDD/2) keeps V_cap ≈ 0.  When audio starts
 *     (PCM samples centred on 128) or stops, V_amp_in = V_DAC − V_cap ≈ VDD/2
 *     throughout — no stored charge to release as a transient → no chirp.
 *
 *     The previous approach (oneshot at 0 ↔ continuous) charged the cap to
 *     V_cap = −VDD/2 at idle.  Switching to continuous (which starts at 0 then
 *     ramps to 128) produced V_amp_in = 128_step − (−VDD/2) = full-rail spike,
 *     audible as a chirp through the amplifier on every button press.
 *
 *   Silence = 128 streams at the current sample rate between sounds.
 *   On playback start: if the driver is already running at the correct sample
 *     rate (common case) — zero reconfiguration, zero transient.
 *   On playback end: a short silence flush replaces residual audio samples in
 *     the DMA ring, then the driver keeps running.
 *   Rate change (rare): driver is briefly recreated; a startup ramp minimises
 *     the single transient that occurs on the very first enable.
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

/* ── Runtime state ─────────────────────────────────────────────────── */
static int               s_volume      = 20;
static volatile bool     s_stop_flag   = false;
static TaskHandle_t      s_audio_task  = NULL;
static SemaphoreHandle_t s_play_mutex  = NULL;

/* DAC continuous handle – runs at all times (silence between sounds) */
static dac_continuous_handle_t s_dac_cont = NULL;
static uint32_t                s_dac_rate = 0;   /* current driver sample rate */

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
 * Write exactly one full DMA descriptor (DAC_DMA_BUF_SIZE bytes) of
 * silence (128 = mid-rail) into the DMA ring.
 *
 * IMPORTANT: sub-descriptor writes (e.g. 512 bytes into a 2048-byte slot)
 * leave the driver's internal accumulation buffer in a partial state.
 * The hardware may never schedule that partial descriptor, so the DMA
 * keeps looping old audio data rather than transitioning to silence.
 * Writing exactly DAC_DMA_BUF_SIZE bytes guarantees one complete
 * descriptor is queued and the ISR picks it up on the next cycle.
 */
static void dac_write_silence(void)
{
    if (!s_dac_cont) return;
    uint8_t sil[DAC_DMA_BUF_SIZE];   /* 2048 bytes = one full descriptor */
    memset(sil, 128, sizeof(sil));
    size_t w;
    dac_continuous_write(s_dac_cont, sil, sizeof(sil), &w, pdMS_TO_TICKS(200));
}

/*
 * Ensure the DAC continuous driver is running at `sample_rate`.
 *
 * Common case (rate unchanged): driver already running — no-op, returns OK.
 *   No mode switch → no transient → no chirp.
 *
 * Rate change (rare): briefly recreates the driver at the new rate.
 *   A 0→128 startup ramp is written to smooth the initial DMA-zero → mid-rail
 *   transition through the AC coupling cap.  A transient is unavoidable on
 *   the very first enable, but this code path is only hit when WAV files with
 *   different sample rates are played back-to-back.
 */
static esp_err_t dac_cont_start(uint32_t sample_rate)
{
    if (s_dac_cont && s_dac_rate == sample_rate) {
        /* Already running at the correct rate — nothing to reconfigure. */
        return ESP_OK;
    }

    if (s_dac_cont) {
        /* Rate change: tear down and recreate at the new rate. */
        ESP_LOGI(TAG, "DAC rate change %u → %u Hz — brief restart",
                 (unsigned)s_dac_rate, (unsigned)sample_rate);
        dac_continuous_disable(s_dac_cont);
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
        s_dac_rate = 0;
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
        return err;
    }

    err = dac_continuous_enable(s_dac_cont);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dac_continuous_enable: %s", esp_err_to_name(err));
        dac_continuous_del_channels(s_dac_cont);
        s_dac_cont = NULL;
        return err;
    }
    s_dac_rate = sample_rate;

    /* Prime new ring: 0→128 ramp spread across exactly one DMA descriptor
     * (DAC_DMA_BUF_SIZE = 2048 bytes) so the hardware always schedules a
     * complete descriptor.  Then follow with one full silence descriptor at 128.
     * DMA ring starts zero-filled; without this ramp the 0→128 step through
     * the AC coupling cap would produce a single start-up transient. */
    {
        uint8_t ramp[DAC_DMA_BUF_SIZE];
        for (size_t i = 0; i < DAC_DMA_BUF_SIZE; i++)
            ramp[i] = (uint8_t)(128 * i / DAC_DMA_BUF_SIZE);
        size_t w;
        dac_continuous_write(s_dac_cont, ramp, sizeof(ramp), &w, pdMS_TO_TICKS(300));
    }
    dac_write_silence();   /* settle ring at 128 before audio begins */
    return ESP_OK;
}

/*
 * End-of-playback: keep the continuous driver running.
 *
 * The driver is intentionally NOT stopped here.  Stopping and restarting the
 * driver on each sound is the root cause of the chirp: it recharges the AC
 * coupling cap from an unpredictable voltage to the new idle level, producing
 * a transient through the LTK8002D amplifier.
 *
 * Instead, the caller (task_cleanup) writes silence (128) back into the DMA
 * ring BEFORE calling dac_cont_stop(), so the ring loops silence when audio
 * ends.  dac_cont_stop() is now a logging-only no-op that documents intent.
 */
static void dac_cont_stop(void)
{
    /* Intentional no-op: driver keeps running with silence (128) in the ring.
     * See file header for the AC-coupling rationale. */
    ESP_LOGD(TAG, "dac_cont_stop: driver left running at %u Hz", (unsigned)s_dac_rate);
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
        /* ── End-of-playback: drain ring, then restore silence ─────────────
         *
         * Wait for all queued audio frames to play out, then write one full
         * silence descriptor (128 = mid-rail).  The driver stays running;
         *
         * Drain timing: ring_occ = min(total_bytes_out, ring_bytes).
         * Writing silence AFTER the wait avoids pushing the silence bytes
         * behind un-played audio (which would delay the next sound). */
        const uint32_t ring_bytes = (uint32_t)DAC_DESC_NUM * DAC_DMA_BUF_SIZE;
        uint32_t ring_occ = (total_bytes_out < ring_bytes) ? total_bytes_out : ring_bytes;
        vTaskDelay(pdMS_TO_TICKS(ring_occ * 1000 / dac_rate + 150));
        dac_write_silence();   /* one full 2048-byte descriptor at 128 */
    }
    free(buf);
    dac_cont_stop();   /* intentional no-op: driver keeps running */

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

    ESP_LOGI(TAG, "Audio init – DAC GPIO%d (always-on continuous, idle at 128)",
             PIN_AUDIO_DAC);

    /* Start the DAC continuous driver immediately at a 32 kHz default rate.
     * It runs permanently; sounds are written into the DMA ring on demand and
     * silence (128 = mid-rail) fills the ring between sounds.
     *
     * Keeping the driver always running means the AC coupling cap stays
     * charged to V_cap ≈ 0 (DAC avg = VDD/2 = 128).  Every playback start
     * and stop is therefore a 128→audio and audio→128 transition with V_cap≈0,
     * producing V_amp_in = VDD/2 throughout — no chirps, no pops. */
    dac_continuous_config_t cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num  = DAC_DESC_NUM,
        .buf_size  = DAC_DMA_BUF_SIZE,
        .freq_hz   = 32000,
        .clk_src   = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode  = DAC_CHANNEL_MODE_SIMUL,
    };
    if (dac_continuous_new_channels(&cfg, &s_dac_cont) != ESP_OK ||
        dac_continuous_enable(s_dac_cont) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start DAC continuous driver — audio disabled");
        if (s_dac_cont) {
            dac_continuous_del_channels(s_dac_cont);
            s_dac_cont = NULL;
        }
    } else {
        s_dac_rate = 32000;
        /* Startup ramp: 0→128 spread across exactly one DMA descriptor
         * (DAC_DMA_BUF_SIZE = 2048 bytes) so a complete descriptor is always
         * scheduled.  Sub-descriptor writes can leave the TX queue in a partial
         * state, preventing the ISR from firing and stalling subsequent writes.
         * Follow with one full silence descriptor at 128 to settle the ring. */
        {
            uint8_t ramp[DAC_DMA_BUF_SIZE];
            for (size_t i = 0; i < DAC_DMA_BUF_SIZE; i++)
                ramp[i] = (uint8_t)(128 * i / DAC_DMA_BUF_SIZE);
            size_t w;
            dac_continuous_write(s_dac_cont, ramp, sizeof(ramp), &w,
                                 pdMS_TO_TICKS(300));
        }
        dac_write_silence();   /* ring now holds 128; DMA ISR active */
        ESP_LOGI(TAG, "DAC continuous started at 32 kHz — ring primed with silence");

    }

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
    /* Poll briefly for playback task to finish (it gives the mutex on exit).
     * The task writes silence into the ring before exiting, so the DAC
     * output is already at 128 (mid-rail) when it returns. */
    for (int i = 0; i < 30 && s_audio_task != NULL; i++)
        vTaskDelay(pdMS_TO_TICKS(10));
}
