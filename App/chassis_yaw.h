#ifndef CHASSIS_YAW_H
#define CHASSIS_YAW_H

#include "struct_typedef.h"

extern uint8_t CHASSIS_YAW_INPUT_RC;
#define CHASSIS_YAW_RC_DEADBAND     20
#define CHASSIS_YAW_RC_RATE_DPS     150.0f // 满杆时目标每秒变化的度数 
#define CHASSIS_YAW_RC_MAX_LEAD     90.0f // 车身跟不上时限制目标领先量 
#define CHASSIS_YAW_ANGLE_DEADBAND  2.0f
#define CHASSIS_YAW_KP              0.3f
#define CHASSIS_YAW_KI              0.0f // 按秒积分 
#define CHASSIS_YAW_MAX_IOUT        3.0f // 积分项输出限幅 
#define CHASSIS_YAW_KD              0.02f // 对实测角速度做阻尼，时间单位为秒 
#define CHASSIS_YAW_RATE_FILTER_S   0.05f
#define CHASSIS_YAW_MAX_WZ          8.0f
#define CHASSIS_YAW_FEEDBACK_MS     500u

#define CHASSIS_YAW_OUTPUT_SIGN    (-1.0f)

extern fp32 chassis_yaw_target_deg;
extern fp32 chassis_yaw_error_deg;
extern fp32 chassis_yaw_rate_dps;
extern fp32 chassis_yaw_iout;

void Chassis_Yaw_Init(void);

void Chassis_Yaw_Feedback(fp32 yaw_deg);

 // 在底盘任务中每收到一条新指令调用一次；无新鲜反馈时返回 0 
uint8_t Chassis_Yaw_SetOffset(fp32 offset_deg);
// 每个底盘周期调用，返回原轮速分配使用的 wz
fp32 Chassis_Yaw_Control(int16_t rc_yaw);

#endif
