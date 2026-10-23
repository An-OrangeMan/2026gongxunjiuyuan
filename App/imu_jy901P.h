#ifndef __IMU_JY901P_H
#define __IMU_JY901P_H

#include "main.h"
#include "usart.h"

void imu_ReceData(uint8_t RxData);
	
extern float Roll_t, Pitch_t, Yaw_t;
extern uint8_t rece_data;
extern int a;

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Attitude_t;

#endif
