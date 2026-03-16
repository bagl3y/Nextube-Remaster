#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#ifdef __cplusplus
extern "C" {
#endif
void pcf8563_init(void);
bool rtc_get_time(struct tm *t);
bool rtc_set_time(const struct tm *t);
#ifdef __cplusplus
}
#endif
