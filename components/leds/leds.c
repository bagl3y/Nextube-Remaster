#include "leds.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "leds";

static uint8_t led_data[LED_COUNT][3];  /* GRB */
static uint8_t brightness = 60;
static TaskHandle_t led_task_handle = NULL;

/* RMT-based WS2812 driver */
static rmt_channel_t rmt_ch = RMT_CHANNEL_0;

static void ws2812_write(void)
{
    /* Simplified: real implementation uses RMT items for WS2812 timing.
       Each bit: T0H=0.4us T0L=0.85us, T1H=0.8us T1L=0.45us */
    rmt_item32_t items[LED_COUNT * 24];
    int idx = 0;
    for (int led = 0; led < LED_COUNT; led++) {
        for (int c = 0; c < 3; c++) {
            uint8_t val = (led_data[led][c] * brightness) / 100;
            for (int bit = 7; bit >= 0; bit--) {
                if (val & (1 << bit)) {
                    items[idx].duration0 = 16;  items[idx].level0 = 1;
                    items[idx].duration1 = 9;   items[idx].level1 = 0;
                } else {
                    items[idx].duration0 = 8;   items[idx].level0 = 1;
                    items[idx].duration1 = 17;  items[idx].level1 = 0;
                }
                idx++;
            }
        }
    }
    rmt_write_items(rmt_ch, items, idx, true);
}

void leds_init(void)
{
    ESP_LOGI(TAG, "Initialising WS2812 LEDs on GPIO%d", PIN_LED_DATA);

    rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(PIN_LED_DATA, rmt_ch);
    cfg.clk_div = 2;  /* 40MHz -> 25ns per tick */
    rmt_config(&cfg);
    rmt_driver_install(rmt_ch, 0, 0);

    memset(led_data, 0, sizeof(led_data));
    ws2812_write();
}

void leds_set_color(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx < 0 || idx >= LED_COUNT) return;
    led_data[idx][0] = g;
    led_data[idx][1] = r;
    led_data[idx][2] = b;
}

void leds_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LED_COUNT; i++)
        leds_set_color(i, r, g, b);
}

void leds_set_brightness(uint8_t pct)
{
    brightness = pct > 100 ? 100 : pct;
}

void leds_update(void) { ws2812_write(); }

void leds_off(void)
{
    memset(led_data, 0, sizeof(led_data));
    ws2812_write();
}

void leds_effect_breath(void)
{
    static float phase = 0;
    phase += 0.05f;
    float val = (sinf(phase) + 1.0f) / 2.0f;
    uint8_t v = (uint8_t)(val * 200);
    leds_set_all(v/3, v/2, v);
    leds_update();
}

void leds_effect_rainbow(void)
{
    static int hue_offset = 0;
    hue_offset = (hue_offset + 4) % 360;
    for (int i = 0; i < LED_COUNT; i++) {
        int hue = (hue_offset + i * 60) % 360;
        /* Simple HSV->RGB at S=1, V=200 */
        int sector = hue / 60;
        int f = (hue % 60) * 200 / 60;
        uint8_t r, g, b;
        switch (sector) {
            case 0: r=200; g=f;   b=0;     break;
            case 1: r=200-f; g=200; b=0;   break;
            case 2: r=0;   g=200; b=f;     break;
            case 3: r=0;   g=200-f; b=200; break;
            case 4: r=f;   g=0;   b=200;   break;
            default: r=200; g=0;  b=200-f; break;
        }
        leds_set_color(i, r, g, b);
    }
    leds_update();
}
