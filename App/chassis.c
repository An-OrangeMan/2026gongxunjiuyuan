#include "chassis.h"
#include "control.h"


static void chassis_move_forward_decomposition(fp32 rate[4], fp32 vx, fp32 vy, fp32 wz)
{
    rate[0] = -vx + vy + wz;
    rate[1] =  vx + vy + wz;
    rate[2] =  vx - vy + wz;
    rate[3] = -vx - vy + wz;
	
//    rate[0] = 500;
//    rate[1] = 500;
//    rate[2] = 500;
//    rate[3] = 500;
}

void Chassis_task(void const *argument)
{
    const RC_ctrl_t *rc = get_remote_control_point();
    fp32 vx, vy, wz;
    fp32 rate[4];

    for (;;)
    {
        vx =  (fp32)rc->rc.ch[CHASSIS_X_CHANNEL]   / CHASSIS_RC_RANGE * CHASSIS_MAX_SPEED;
        vy = -(fp32)rc->rc.ch[CHASSIS_Y_CHANNEL]   / CHASSIS_RC_RANGE * CHASSIS_MAX_SPEED;
        wz = -(fp32)rc->rc.ch[CHASSIS_YAW_CHANNEL] / CHASSIS_RC_RANGE * CHASSIS_MAX_SPEED;

        chassis_move_forward_decomposition(rate, vx, vy, wz);


        {
            fp32 max_rate = 0.0f;
            for (int i = 0; i < MOTOR_NUM; i++)
            {
                fp32 a = (rate[i] > 0.0f) ? rate[i] : -rate[i];
                if (a > max_rate)
                {
                    max_rate = a;
                }
            }
            if (max_rate > CHASSIS_MAX_SPEED)
            {
                fp32 scale = CHASSIS_MAX_SPEED / max_rate;
                for (int i = 0; i < MOTOR_NUM; i++)
                {
                    rate[i] *= scale;
                }
            }
        }

        for (int i = 0; i < MOTOR_NUM; i++)
        {
            motor_speed_set[i] = (int16_t)rate[i];
        }

        vTaskDelay(CHASSIS_TASK_TIMEOUT);
    }
}
