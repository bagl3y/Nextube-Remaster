#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void wifi_manager_start(void);
bool wifi_manager_is_connected(void);
const char *wifi_manager_get_ip(void);
void wifi_manager_scan_start(void);
#ifdef __cplusplus
}
#endif
