#include "chassis.h"
#include "control.h"
#include "chassis_yaw.h"

volatile int aa;

static void chassis_move_forward_decomposition(fp32 rate[4], fp32 vx, fp32 vy, fp32 wz)
{
    
    rate[0] = -vx + vy + wz;       
    rate[1] = -(vx + vy + wz);     
    rate[2] = -vx - vy + wz;       
    rate[3] = -vx + vy - wz;       
}

void Chassis_task(void const *argument)
{
    const RC_ctrl_t *rc = get_remote_control_point();
    fp32 vx, vy, wz;
    fp32 rate[4];
    uint8_t previous_mode = 0;
    uint8_t mode;

    Motor_SetEnabled(0);
    Chassis_Yaw_Init();

    for (;;)
    {
        mode = (uint8_t)rc->rc.s[1];
        if (mode != previous_mode)
        {
            Motor_SetEnabled(0);
            Chassis_Yaw_Init(); 
            previous_mode = mode;
        }
        if (mode != 3 && mode != 2) 
        {
            Motor_SetEnabled(0);
            vTaskDelay(CHASSIS_TASK_TIMEOUT);
            continue;
        }
        CHASSIS_YAW_INPUT_RC = (mode == 3);

        vx =  (fp32)rc->rc.ch[CHASSIS_X_CHANNEL] / CHASSIS_RC_RANGE * CHASSIS_MAX_SPEED;
        vy = -(fp32)rc->rc.ch[CHASSIS_Y_CHANNEL] / CHASSIS_RC_RANGE * CHASSIS_MAX_SPEED;
        wz = -(fp32)rc->rc.ch[CHASSIS_YAW_CHANNEL] / CHASSIS_RC_RANGE * CHASSIS_MAX_SPEED;
			aa=rc->rc.ch[CHASSIS_YAW_CHANNEL];

        if (!CHASSIS_YAW_INPUT_RC)
        {
            vx = 0.0f;
            vy = 0.0f;
           
        }
        wz = Chassis_Yaw_Control(rc->rc.ch[CHASSIS_YAW_CHANNEL]);

        chassis_move_forward_decomposition(rate, vx, vy, wz);

        {
            fp32 max_rate = 0.0f;
            for (int i = 0; i < MOTOR_NUM; i++)
            {
                fp32 a = fabsf(rate[i]);
                if (a > max_rate) max_rate = a;
            }
            if (max_rate > CHASSIS_MAX_SPEED)
            {
                fp32 scale = CHASSIS_MAX_SPEED / max_rate;
                for (int i = 0; i < MOTOR_NUM; i++) rate[i] *= scale;
            }
        }

        for (int i = 0; i < MOTOR_NUM; i++)
        {
            motor_speed_set[i] = (int16_t)rate[i];
        }
        Motor_SetEnabled(1);

        vTaskDelay(CHASSIS_TASK_TIMEOUT);
    }
}
