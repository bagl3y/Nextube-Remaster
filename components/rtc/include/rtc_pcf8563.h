#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "driver/i2c_master.h"
#ifdef __cplusplus
extern "C" {
#endif
void pcf8563_init(void);
bool rtc_get_time(struct tm *t);
bool rtc_set_time(const struct tm *t);
/* Returns the I²C master bus handle so other drivers (e.g. SHT30) can
 * register themselves on the same shared bus without re-initialising it. */
i2c_master_bus_handle_t pcf8563_get_bus_handle(void);
#ifdef __cplusplus
}
#endif
