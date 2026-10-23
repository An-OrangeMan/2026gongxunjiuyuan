#ifndef YAW_TEST_TIM_H
#define YAW_TEST_TIM_H
#include "main.h"
typedef struct { uint32_t compare[4]; } TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim2, htim4, htim6;
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->compare[channel] = (value))
void HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
#endif
