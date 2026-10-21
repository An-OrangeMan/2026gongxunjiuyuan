#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "head_file.h"


#define MOTOR_GEAR_RATIO        30.0f    
#define MOTOR_ENCODER_PPR       11.0f    
#define MOTOR_NOLOAD_RPM        320.0f   
#define MOTOR_RATED_RPM         180.0f   
#define MOTOR_VOLTAGE           12.0f
#define MOTOR_RATED_CURRENT_A   1.2f
#define MOTOR_STALL_CURRENT_A   3.5f     
#define WHEEL_DIAMETER_M        0.080f   
#define SPEED_LOOP_BASE_HZ      1000.0f  
#define SPEED_PID_DECIMATION    4u       
#define SPEED_LOOP_HZ           (SPEED_LOOP_BASE_HZ / SPEED_PID_DECIMATION)
#define ENCODER_MULT            4.0f     
#define ENC_COUNTS_PER_REV      (MOTOR_ENCODER_PPR * MOTOR_GEAR_RATIO * ENCODER_MULT)
#define SPEED_MAX_CNT           (ENC_COUNTS_PER_REV * MOTOR_NOLOAD_RPM / 60.0f / SPEED_LOOP_HZ)
#define WHEEL_MAX_V_MPS         (3.14159265f * WHEEL_DIAMETER_M * MOTOR_NOLOAD_RPM / 60.0f)
#define SPEED_KP                ((fp32)PWM_MAX * 0.60f / SPEED_MAX_CNT)
#define SPEED_KI                (SPEED_KP / 20.0f)
#define SPEED_KD                0.0f
#define SPEED_MAX_OUT           ((fp32)PWM_MAX)
#define SPEED_MAX_IOUT          ((fp32)PWM_MAX * 0.40f)
#define CHASSIS_MAX_SPEED       SPEED_MAX_CNT
#define CHASSIS_TASK_TIMEOUT    5      
#define CHASSIS_X_CHANNEL       3       
#define CHASSIS_Y_CHANNEL       2       
#define CHASSIS_YAW_CHANNEL     0       
#define CHASSIS_RC_RANGE        660.0f

void Chassis_task(void const *argument);

#endif
