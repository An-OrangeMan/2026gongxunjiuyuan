#ifndef YAW_TEST_MAIN_H
#define YAW_TEST_MAIN_H
#include <stddef.h>
#include "struct_typedef.h"
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
#endif
