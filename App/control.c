#include "control.h"
#include "encoder.h"
#include "chassis.h"    

motor_t motor[MOTOR_NUM];
volatile int16_t motor_speed_set[MOTOR_NUM] = {0, 0, 0, 0};
volatile uint8_t motor_enabled = 0;


static void set_motor_pwm(TIM_HandleTypeDef *htim, uint32_t ch_in1, uint32_t ch_in2, int16_t pwm)
{
    int16_t duty = pwm;
    if (duty >  PWM_MAX) duty =  PWM_MAX;
    if (duty < -PWM_MAX) duty = -PWM_MAX;

    if (duty > 0)
    {
        __HAL_TIM_SET_COMPARE(htim, ch_in1, (uint32_t)duty);
        __HAL_TIM_SET_COMPARE(htim, ch_in2, 0);
    }
    else if (duty < 0)
    {
        __HAL_TIM_SET_COMPARE(htim, ch_in1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch_in2, (uint32_t)(-duty));
    }
    else
    {
        __HAL_TIM_SET_COMPARE(htim, ch_in1, 0);
        __HAL_TIM_SET_COMPARE(htim, ch_in2, 0);
    }
}

void Set_Pwm(const int16_t pwms[MOTOR_NUM])
{
    // 电机1 左前：TIM2 CH1/CH2 
    set_motor_pwm(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, pwms[0]);
    // 电机2 右前：TIM2 CH3/CH4 
    set_motor_pwm(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, pwms[1]);
    // 电机3 左后：TIM4 CH1/CH2 
    set_motor_pwm(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, pwms[2]);
    // 电机4 右后：TIM4 CH3/CH4 
    set_motor_pwm(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, pwms[3]);
}

void Motor_SetEnabled(uint8_t enabled)
{
    uint32_t mask = __get_PRIMASK();
    const int16_t zero[MOTOR_NUM] = {0, 0, 0, 0};
    __disable_irq();
    motor_enabled = enabled != 0;
    if (!motor_enabled)
    {
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            motor_speed_set[i] = 0;
            motor[i].target = 0;
            motor[i].out = 0;
            PID_clear(&motor[i].speed_pid);
        }
        Set_Pwm(zero);
    }
    __set_PRIMASK(mask);
}

void DC_Motor_Init(void)
{
    fp32 pid_param[3] = {SPEED_KP, SPEED_KI, SPEED_KD};
    motor_enabled = 0;

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    Software_Encoder_Init();

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        PID_init(&motor[i].speed_pid, PID_POSITION, pid_param, SPEED_MAX_OUT, SPEED_MAX_IOUT);
    }


    {
        int16_t zero[MOTOR_NUM] = {0, 0, 0, 0};
        Set_Pwm(zero);
    }

    HAL_TIM_Base_Start_IT(&htim6);
}


void Speed_Control_Loop(void)
{
    static int32_t enc_acc[MOTOR_NUM] = {0};
    static uint8_t tick = 0;
    int16_t pwm[MOTOR_NUM];

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        enc_acc[i] += Read_Encoder(i);  
    }

    if (!motor_enabled)
    {
        for (int i = 0; i < MOTOR_NUM; i++) enc_acc[i] = 0;
        tick = 0;
        return;
    }

    if (++tick < SPEED_PID_DECIMATION)
    {
        return;
    }
    tick = 0;

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        motor[i].cur_speed = (int16_t)enc_acc[i];
        enc_acc[i] = 0;
        motor[i].target = motor_speed_set[i];

        if (motor[i].target == 0)
        {

            PID_clear(&motor[i].speed_pid);
            motor[i].out = 0;
        }
        else
        {
            motor[i].out = (int16_t)PID_calc(&motor[i].speed_pid,
                                             (fp32)motor[i].cur_speed,
                                             (fp32)motor[i].target);
        }
        pwm[i] = motor[i].out;
    }

    Set_Pwm(pwm);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        Speed_Control_Loop();
    }
}
