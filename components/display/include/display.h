#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void display_init(void);
void display_set_brightness(uint8_t pct);
void display_show_digit(int tube, const uint8_t *rgb565_data, int w, int h);
void display_fill(int tube, uint16_t color);
void display_show_time(int h, int m, int s, const char *theme);
#ifdef __cplusplus
}
#endif
