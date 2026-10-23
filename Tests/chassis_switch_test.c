#include "chassis.h"
#include "chassis_yaw.h"
#include "control.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>

TIM_HandleTypeDef htim2, htim4, htim6;
static RC_ctrl_t remote;
static uint32_t now, irq_mask;
static unsigned int step;
static jmp_buf task_exit;
uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
void HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) { }
void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer) { }
void Software_Encoder_Init(void) { }
int Read_Encoder(uint8_t idx) { return motor_enabled ? 0 : 100; }
const RC_ctrl_t *get_remote_control_point(void) { return &remote; }

static void stopped(void)
{
    for (unsigned int i = 0; i < MOTOR_NUM; i++)
    {
        assert(motor[i].out == 0);
        assert(motor[i].speed_pid.Iout == 0);
        assert(htim2.compare[i] == 0 && htim4.compare[i] == 0);
    }
}

static void zero_targets(void)
{
    for (unsigned int i = 0; i < MOTOR_NUM; i++) assert(motor_speed_set[i] == 0);
}

void vTaskDelay(uint32_t ticks)
{
    assert(ticks == CHASSIS_TASK_TIMEOUT);
    now += ticks;
    /* Assert the task cleared live PWM before waiting for another speed IRQ. */
    if (!motor_enabled) { stopped(); zero_targets(); }
    for (unsigned int i = 0; i < SPEED_PID_DECIMATION; i++) Speed_Control_Loop();
    for (unsigned int i = 0; i < MOTOR_NUM; i++) assert(motor[i].cur_speed == 0);
    switch (step++)
    {
    case 0: /* No valid switch received at power-on. */
        assert(!motor_enabled);
        remote.rc.s[1] = 1;
        break;
    case 1:
        assert(!motor_enabled);
        for (unsigned int i = 0; i < MOTOR_NUM; i++) motor_speed_set[i] = 99;
        for (unsigned int i = 0; i < 12; i++) Speed_Control_Loop();
        stopped(); /* Even a nonzero target cannot drive disabled motors. */
        Motor_SetEnabled(0);
        remote.rc.s[1] = 3;
        remote.rc.ch[CHASSIS_X_CHANNEL] = 330;
        break;
    case 2:
        assert(motor_enabled && CHASSIS_YAW_INPUT_RC == 1);
        for (unsigned int i = 0; i < MOTOR_NUM; i++)
        {
            assert(motor_speed_set[i] == -14 && motor[i].out < 0);
            assert(motor[i].speed_pid.Iout < 0);
        }
        assert(fabsf(chassis_yaw_target_deg - 30) < 0.001f);
        remote.rc.ch[CHASSIS_YAW_CHANNEL] = 100;
        break;
    case 3:
        assert(chassis_yaw_target_deg > 30 && chassis_yaw_iout > 0);
        remote.rc.s[1] = 1;
        Chassis_Yaw_Feedback(80);
        break;
    case 4:
        assert(!motor_enabled && chassis_yaw_iout == 0);
        assert(Chassis_Yaw_SetOffset(20)); /* A command during stop must not survive re-entry. */
        remote.rc.s[1] = 2;
        remote.rc.ch[CHASSIS_X_CHANNEL] = 660;
        remote.rc.ch[CHASSIS_Y_CHANNEL] = 660;
        remote.rc.ch[CHASSIS_YAW_CHANNEL] = 660;
        break;
    case 5:
        assert(motor_enabled && CHASSIS_YAW_INPUT_RC == 0);
        zero_targets();
        assert(fabsf(chassis_yaw_target_deg - 80) < 0.001f);
        assert(chassis_yaw_iout == 0);
        assert(Chassis_Yaw_SetOffset(20));
        break;
    case 6:
        assert(fabsf(chassis_yaw_target_deg - 100) < 0.001f);
        assert(motor_speed_set[0] < 0 && motor_speed_set[1] > 0);
        assert(motor_speed_set[0] == motor_speed_set[2]);
        assert(motor_speed_set[1] == motor_speed_set[3]);
        remote.rc.s[1] = 3;
        remote.rc.ch[CHASSIS_X_CHANNEL] = 0;
        remote.rc.ch[CHASSIS_Y_CHANNEL] = 0;
        remote.rc.ch[CHASSIS_YAW_CHANNEL] = 0;
        Chassis_Yaw_Feedback(110);
        break;
    case 7:
        assert(motor_enabled && CHASSIS_YAW_INPUT_RC == 1);
        zero_targets();
        assert(fabsf(chassis_yaw_target_deg - 110) < 0.001f);
        assert(chassis_yaw_iout == 0);
        remote.rc.ch[CHASSIS_YAW_CHANNEL] = 330;
        break;
    case 8:
        assert(chassis_yaw_target_deg > 110 && chassis_yaw_iout > 0);
        remote.rc.s[1] = 1;
        break;
    case 9:
        assert(!motor_enabled && chassis_yaw_iout == 0);
        remote.rc.s[1] = 2;
        break;
    case 10:
        assert(motor_enabled && CHASSIS_YAW_INPUT_RC == 0);
        zero_targets();
        assert(fabsf(chassis_yaw_target_deg - 110) < 0.001f);
        remote.rc.s[1] = 0;
        break;
    case 11:
        assert(!motor_enabled);
        longjmp(task_exit, 1);
    default: assert(0);
    }
}

int main(void)
{
    DC_Motor_Init();
    irq_mask = 1;
    Motor_SetEnabled(0);
    assert(irq_mask == 1); /* Stop preserves the caller's interrupt state. */
    irq_mask = 0;
    Chassis_Yaw_Feedback(30);
    if (setjmp(task_exit) == 0) Chassis_task(NULL);
    assert(step == 12);
    puts("PASS: actual chassis + motor PI, switch 0/1/3/2, immediate PWM disable, gated IRQ, mode isolation and clean re-entry");
    return 0;
}
