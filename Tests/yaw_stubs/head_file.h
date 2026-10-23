#ifndef YAW_TEST_HEAD_H
#define YAW_TEST_HEAD_H
#include "main.h"
#include <math.h>
typedef struct { struct { int16_t ch[5]; char s[2]; } rc; } RC_ctrl_t;
const RC_ctrl_t *get_remote_control_point(void);
void vTaskDelay(uint32_t ticks);
#endif
