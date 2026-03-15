/**
 * @file touch_input.c
 * @brief Capacitive touch input – uses IDF 5.x touch_sens driver.
 *
 * Migrated from the legacy driver/touch_pad.h API to driver/touch_sens.h
 * (available for the original ESP32 since IDF 5.5).
 *
 * Three channels:
 *   TOUCH_LEFT   – GPIO 2  (touch pad channel 2)
 *   TOUCH_MIDDLE – GPIO 4  (touch pad channel 0)
 *   TOUCH_RIGHT  – GPIO 15 (touch pad channel 3)
 *
 * Strategy: continuous scanning with software IIR filter.  A poll task
 * (CPU 0, 50 ms) compares SMOOTH data against the hardware BENCHMARK
 * (adaptive baseline) and fires a task-notification to a handler task on
 * rising-edge press (smooth < benchmark × 80 %).  The handler calls the
 * user callback, keeping the poll loop unblocked during flash writes.
 */

#include "touch_input.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/touch_sens.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "touch";

/* ── Touch pad channel numbers (match GPIO from board_pins.h) ───────── */
/*   PIN_TOUCH_LEFT=2→pad2, PIN_TOUCH_MIDDLE=4→pad0, PIN_TOUCH_RIGHT=15→pad3 */
static const int touch_channels[] = { 2, 0, 3 };  /* pad IDs, not GPIO numbers */

/* ── Driver state ───────────────────────────────────────────────────── */
static touch_sensor_handle_t  s_sens = NULL;
static touch_channel_handle_t s_chan[3] = { NULL, NULL, NULL };

static touch_callback_t user_cb = NULL;
static TaskHandle_t touch_task_handle = NULL;

/* ── Poll task ──────────────────────────────────────────────────────── */
/*
 * Detection uses the hardware BENCHMARK (long-term baseline) rather than a
 * one-shot startup calibration.  This prevents drift as the MCU warms up.
 * A press is detected when smooth < benchmark * 80/100 (20 % drop required).
 *
 * Running on CPU 0 so JPEG decoding on CPU 1 (display task) cannot starve it.
 *
 * The user callback (which calls config_set_json / save_to_flash) is
 * dispatched via a task notification to a separate handler task, keeping
 * the 50 ms polling loop unblocked while flash writes occur.
 */

static TaskHandle_t s_handler_task = NULL;

static void touch_handler_task(void *arg)
{
    while (1) {
        uint32_t notif = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &notif, portMAX_DELAY) == pdTRUE) {
            if (user_cb) user_cb((touch_pad_id_t)notif);
        }
    }
}

static void touch_poll_task(void *arg)
{
    bool was_pressed[3] = { false, false, false };
    int  stuck_count[3] = { 0, 0, 0 };   /* detect stuck-pressed */

    while (1) {
        for (int i = 0; i < 3; i++) {
            uint32_t smooth[1]    = { 0 };
            uint32_t benchmark[1] = { 0 };

            if (touch_channel_read_data(s_chan[i], TOUCH_CHAN_DATA_TYPE_SMOOTH,
                                        smooth) != ESP_OK ||
                touch_channel_read_data(s_chan[i], TOUCH_CHAN_DATA_TYPE_BENCHMARK,
                                        benchmark) != ESP_OK) {
                continue;
            }

            /* Dynamic threshold: 20 % drop below long-term baseline */
            uint32_t threshold = benchmark[0] * 80 / 100;
            bool pressed = (smooth[0] < threshold);

            if (pressed) {
                stuck_count[i]++;
                /* Force-release after 1 second of continuous press (20 × 50 ms)
                 * to recover if the filter drifts or a write stalls detection. */
                if (stuck_count[i] > 20) {
                    was_pressed[i] = false;
                    stuck_count[i] = 0;
                }
            } else {
                stuck_count[i] = 0;
            }

            if (pressed && !was_pressed[i]) {
                /* Dispatch to handler task so flash write doesn't block poll */
                if (s_handler_task)
                    xTaskNotify(s_handler_task, (uint32_t)i, eSetValueWithOverwrite);
            }
            was_pressed[i] = pressed;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ════════════════════════════════════════════════════════════════════ */
/*  Public API                                                          */
/* ════════════════════════════════════════════════════════════════════ */

void touch_input_init(void)
{
    ESP_LOGI(TAG, "Initialising touch pads: GPIO%d, GPIO%d, GPIO%d",
             PIN_TOUCH_LEFT, PIN_TOUCH_MIDDLE, PIN_TOUCH_RIGHT);

    /* ── Configure sensor controller ── */
    /* Voltage levels match original firmware:
     *   old: TOUCH_HVOLT_2V7 with TOUCH_HVOLT_ATTEN_1V → effective 1.7 V
     *   old: TOUCH_LVOLT_0V5 → 0.5 V
     * In the new API charge_volt_lim_h is absolute (no attenuation field). */
    touch_sensor_sample_config_t sample_cfg =
        TOUCH_SENSOR_V1_DEFAULT_SAMPLE_CONFIG(
            1.0f,
            TOUCH_VOLT_LIM_L_0V5,
            TOUCH_VOLT_LIM_H_1V7);

    touch_sensor_config_t sens_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(1, &sample_cfg);

    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &s_sens));

    /* ── Register channels ── */
    /* abs_active_thresh = 0: hardware interrupt never fires (polling only) */
    touch_channel_config_t chan_cfg = {
        .abs_active_thresh = { 0 },
        .charge_speed      = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt  = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group             = TOUCH_CHAN_TRIG_GROUP_BOTH,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(touch_sensor_new_channel(s_sens, touch_channels[i],
                                                 &chan_cfg, &s_chan[i]));
    }

    /* ── Configure software (IIR) filter ── */
    /* Required before enable; SMOOTH data type won't work without it.
     * The default config uses a reasonable IIR weight for capacitive pads. */
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_ERROR_CHECK(touch_sensor_config_filter(s_sens, &filter_cfg));

    /* ── Enable and start continuous scanning ── */
    ESP_ERROR_CHECK(touch_sensor_enable(s_sens));
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(s_sens));

    /* Allow sensor readings and benchmark to settle */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Log initial baselines (benchmark tracks drift automatically at runtime) */
    for (int i = 0; i < 3; i++) {
        uint32_t bm[1] = { 0 };
        touch_channel_read_data(s_chan[i], TOUCH_CHAN_DATA_TYPE_BENCHMARK, bm);
        ESP_LOGI(TAG, "Touch%d (pad%d) benchmark=%u threshold(80%%)=%u",
                 i, touch_channels[i], (unsigned)bm[0],
                 (unsigned)(bm[0] * 80 / 100));
    }

    /* Handler task: receives notifications from poll task and calls user_cb.
     * Keeps the 50 ms poll loop unblocked during slow flash writes.
     * Pinned to CPU 0 where other app tasks live. */
    xTaskCreatePinnedToCore(touch_handler_task, "touch_hdl", 3072, NULL,
                            4, &s_handler_task, 0);

    /* Poll task on CPU 0 – away from the display task's JPEG decoding on CPU 1 */
    xTaskCreatePinnedToCore(touch_poll_task, "touch", 2048, NULL,
                            5, &touch_task_handle, 0);
}

void touch_input_register_callback(touch_callback_t cb) { user_cb = cb; }
