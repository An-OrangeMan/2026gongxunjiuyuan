#include "servo.h"
#include "vision.h"
#include "head_file.h"



volatile uint8_t servo_target[SERVO_NUM] = {80, 90};


volatile uint8_t servo_now[SERVO_NUM] = {80, 90};


static TIM_HandleTypeDef * const servo_tim[SERVO_NUM] =
{
    SERVO1_TIM,
    SERVO2_TIM
};

static const uint32_t servo_ch[SERVO_NUM] =
{
    SERVO1_CHANNEL,
    SERVO2_CHANNEL
};


void Servo_Init(void)
{
    for (uint8_t i = 0; i < SERVO_NUM; i++)
    {
        Servo_SetAngle(i, servo_now[i]);
        HAL_TIM_PWM_Start(servo_tim[i], servo_ch[i]);
    }
}


void Servo_SetAngle(uint8_t idx, uint8_t angle)
{
    uint16_t ccr;

    if (idx >= SERVO_NUM)
        return;

    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    ccr = SERVO_MIN_PULSE + (uint16_t)((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / SERVO_MAX_ANGLE);

    __HAL_TIM_SET_COMPARE(servo_tim[idx], servo_ch[idx], ccr);
}

static void Gripper_Update(void)
{
    const RC_ctrl_t *rc = get_remote_control_point();
    int16_t wheel;

    if (rc == 0)
        return;

    wheel = rc->rc.ch[GRIPPER_RC_CHANNEL];

#if GRIPPER_INVERT
    wheel = -wheel;
#endif

    /* 回中（|wheel| ≤ 阈值）→ 什么都不写，servo_target 保持不变 = 锁定当前开口 */
    if (wheel >= -GRIPPER_RC_THRESHOLD && wheel <= GRIPPER_RC_THRESHOLD)
        return;

    if (wheel > 0)                          /* 向上拨 → 关闭 */
        servo_target[0] = GRIPPER_CLOSE_ANGLE;
    else                                    /* 向下拨 → 打开 */
        servo_target[0] = GRIPPER_OPEN_ANGLE;

#if GRIPPER_MIRROR
    servo_target[1] = (uint8_t)(SERVO_MAX_ANGLE - servo_target[0]);
#else
    servo_target[1] = servo_target[0];
#endif
}


void Servo_task(void const *argument)
{
    Servo_Init();

    for (;;)
    {
        if (!vision_active)
            Gripper_Update();
        for (uint8_t i = 0; i < SERVO_NUM; i++)
        {
            int16_t diff = (int16_t)servo_target[i] - (int16_t)servo_now[i];

            if (diff > SERVO_STEP_DEG)
                diff = SERVO_STEP_DEG;
            else if (diff < -SERVO_STEP_DEG)
                diff = -SERVO_STEP_DEG;

            if (diff != 0)
            {
                servo_now[i] = (uint8_t)((int16_t)servo_now[i] + diff);
                Servo_SetAngle(i, servo_now[i]);
            }
        }

        vTaskDelay(SERVO_TASK_TIMEOUT);
    }
}
