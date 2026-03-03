#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t subscriber_count;
    bool     valid;
} sub_count_t;
void youtube_bili_start(void);
const sub_count_t *youtube_bili_get(void);
#ifdef __cplusplus
}
#endif
