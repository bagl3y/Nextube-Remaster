#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void leds_init(void);
void leds_task_start(void);      /* start the FreeRTOS effect task */
void leds_set_color(int idx, uint8_t r, uint8_t g, uint8_t b);
void leds_set_all(uint8_t r, uint8_t g, uint8_t b);
void leds_set_brightness(uint8_t pct);
void leds_update(void);
void leds_effect_breath(void);
void leds_effect_rainbow(void);
void leds_off(void);
/* Pause/resume RMT transmissions during audio playback.
 * WS2812 LEDs hold their last colour without refresh, so visually
 * nothing changes.  Pausing stops RMT 10 MHz bursts and the resulting
 * current spikes on the 3.3 V rail from coupling into the DAC output. */
void leds_set_audio_active(bool active);
#ifdef __cplusplus
}
#endif
