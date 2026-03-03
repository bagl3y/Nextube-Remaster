#pragma once
#include <time.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void ntp_time_start(void);
bool ntp_time_synced(void);
void ntp_get_local(struct tm *t);
#ifdef __cplusplus
}
#endif
