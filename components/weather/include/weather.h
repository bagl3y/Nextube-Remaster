#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float temp_c;
    float humidity;
    char  condition[32];
    char  icon[16];
    bool  valid;
} weather_data_t;
void weather_start(void);
const weather_data_t *weather_get(void);
#ifdef __cplusplus
}
#endif
