#ifndef __SERVO_H__
#define __SERVO_H__


#include "tim.h"


#define GRIPPER_RC_CHANNEL    4
#define GRIPPER_RC_THRESHOLD  200      
#define GRIPPER_OPEN_ANGLE    90
#define GRIPPER_CLOSE_ANGLE   30        
#define GRIPPER_INVERT        1        
#define GRIPPER_MIRROR        1        /* 1 = 二号舵机镜像安装 */


#define SERVO_NUM        2

#define SERVO1_TIM       (&htim1)
#define SERVO1_CHANNEL   TIM_CHANNEL_1     //PA8

#define SERVO2_TIM       (&htim1)
#define SERVO2_CHANNEL   TIM_CHANNEL_4     //PE14


#define SERVO_MIN_PULSE  500
#define SERVO_MAX_PULSE  2500
#define SERVO_MAX_ANGLE  180



#define SERVO_TASK_TIMEOUT   20     //任务周期 20ms（tick=1kHz）
#define SERVO_STEP_DEG       5      //每 20ms 最多转 5°


extern volatile uint8_t servo_target[SERVO_NUM];


void Servo_Init(void);
void Servo_SetAngle(uint8_t idx, uint8_t angle);
void Servo_task(void const *argument);

#endif
