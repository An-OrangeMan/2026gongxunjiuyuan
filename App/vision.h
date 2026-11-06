#ifndef __VISION_H__
#define __VISION_H__

#include "main.h"
#include "chassis.h"

//帧头
#define V_HEAD0  0xAA
#define V_HEAD1  0x55

//命令码
#define V_CMD_STOP    0x01
#define V_CMD_MOVE    0x02
#define V_CMD_TURN    0x03
#define V_CMD_GRIP    0x04
#define V_CMD_SPIN    0x05
#define V_CMD_RESULT  0x81
#define V_CMD_TEAM    0x82
#define V_CMD_START   0x83

//本方00 蓝 01 红 
#define V_TEAM_COLOR   0x00

//转弯方向符号：左转(正)要 yaw 减小，取 -1；上车反了就改 +1 
#define V_TURN_SIGN    (-1.0f)

//运动速度（编码器计数/4ms；CHASSIS_MAX_SPEED≈28）
#define V_MOVE_SPEED   (CHASSIS_MAX_SPEED * 0.5f)
#define V_SPIN_SPEED   (CHASSIS_MAX_SPEED * 0.5f)

//反馈状态
#define V_STAT_RUNNING 0x00
#define V_STAT_DONE    0x01

//自动模式标志：1 = 视觉接管底盘与夹爪 
extern volatile uint8_t vision_active;

void Vision_task(void const *argument);
void Vision_FeedByte(uint8_t b);   

#endif
