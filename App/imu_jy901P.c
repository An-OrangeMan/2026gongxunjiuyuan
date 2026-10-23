#include "imu_jy901P.h"
#include "chassis_yaw.h"

uint8_t RxBuffer[11];  
static volatile uint8_t RxState = 0;  
uint8_t RxIndex = 0;
int cnt;
int cnt1;
int cnt2;
int cnt3;

uint8_t RX_Buffer[12] = {0};
uint8_t RX_Index = 0;
uint8_t last_byte = 0;
uint8_t is_receiving = 0;
Attitude_t current_attitude;

//extern volatile int a;

float Roll_t, Pitch_t, Yaw_t;
uint8_t rece_data;

void imu_ReceData(uint8_t RxData)
{
	uint8_t sum_t = 0;
	if (RxState == 0)
	{
		if (RxData == 0x55)
		{
		  RxBuffer[RxIndex] = RxData;
		
		  RxState = 1;
		  RxIndex = 1;
		}
	}
	else if (RxState == 1)
	{
		if(RxData == 0x53)
		{
			RxBuffer[RxIndex] = RxData;
			
			RxState = 2;
	  	RxIndex = 2;
		}
	}
	else if (RxState == 2)
	{
		RxBuffer[RxIndex++] = RxData;
		
		if (RxIndex == 11)
		{
			for (int i=0 ; i<10 ; i++)
			{
				sum_t = sum_t + RxBuffer[i];
			}
			if (sum_t == RxBuffer[10])
			{
				/* 取反模块原始 yaw：左转减小，右转增大。 */
				Yaw_t = -((int16_t)((int16_t)RxBuffer[7] << 8 | (int16_t)RxBuffer[6])) / 32768.0f * 180.0f;
				Chassis_Yaw_Feedback(Yaw_t);
			}
			RxState = 0;
	  	RxIndex = 0;
		}
	}
}





void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6)
	{
		
		
		
		
	imu_ReceData(rece_data);

			HAL_UART_Receive_IT(&huart6, &rece_data, 1);
	}
}
