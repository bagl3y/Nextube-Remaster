#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { TOUCH_LEFT, TOUCH_MIDDLE, TOUCH_RIGHT } touch_pad_id_t;
typedef void (*touch_callback_t)(touch_pad_id_t pad);
void touch_input_init(void);
void touch_input_register_callback(touch_callback_t cb);
#ifdef __cplusplus
}
#endif
