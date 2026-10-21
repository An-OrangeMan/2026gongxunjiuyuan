#ifndef __CONTROL_H
#define __CONTROL_H

#include "tim.h"
#include "pid.h"

#define MOTOR_NUM   4

/* TB6612 双输入 PWM（IN1/IN2），PWMA/PWMB 接高。
 * 正转：IN1=PWM, IN2=0；反转：IN1=0, IN2=PWM；刹车：IN1=IN2=0 */
#define PWM_MAX     999     /* TIM2/TIM4 Period(ARR)=999，占空比 0~999 */

/* 电机物理参数、速度环 PID 增益、控制周期都在 chassis.h 里，
 * 换电机只改 chassis.h 顶部那一段。 */

typedef struct
{
    pid_type_def speed_pid;   /* pid.c 的速度环 */
    int16_t cur_speed;        /* 实测速度（编码器计数/周期） */
    int16_t target;           /* 目标速度（编码器计数/周期） */
    int16_t out;              /* PID 输出（PWM 占空比） */
} motor_t;

extern motor_t motor[MOTOR_NUM];
/* 底盘任务写目标速度，TIM6 中断读（int16 在 Cortex-M4 上原子访问） */
extern volatile int16_t motor_speed_set[MOTOR_NUM];

void DC_Motor_Init(void);
void Set_Pwm(const int16_t pwms[MOTOR_NUM]);
void Speed_Control_Loop(void);   /* 在 TIM6 周期中断回调里调用 */

#endif
