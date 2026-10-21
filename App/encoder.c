#include "encoder.h"

/* ===================== 软件正交解码（M1 / M3） =====================
 * 对应关系：sw 索引 0 -> 电机1 左前 (PC2/PC3)
 *           sw 索引 1 -> 电机3 左后 (PE4/PE12)
 *
 * 原理：A、B 两相都开双边沿 EXTI，每次中断读 A/B 当前电平，
 *       用 (上一状态<<2 | 当前状态) 查 16 项状态表得到 +1 / 0 / -1。
 *       一个正交周期共 4 次跳变，正好和硬件 TIM 编码器的 TI12 模式
 *       （4 倍频）计数完全一致。
 */
static volatile int32_t sw_count[2];
static uint8_t          sw_last[2];

/* index = (上状态 << 2) | 当前状态，状态 = (A << 1) | B */
static const int8_t quad_table[16] =
{
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static void quad_update(uint8_t idx, GPIO_TypeDef *portA, uint16_t pinA,
                        GPIO_TypeDef *portB, uint16_t pinB)
{
    uint8_t a    = (HAL_GPIO_ReadPin(portA, pinA) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t b    = (HAL_GPIO_ReadPin(portB, pinB) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t curr = (uint8_t)((a << 1) | b);

    sw_count[idx] += quad_table[(sw_last[idx] << 2) | curr];
    sw_last[idx]  = curr;
}

void Software_Encoder_Init(void)
{
    uint8_t a, b;

    a = (HAL_GPIO_ReadPin(ENC1_A_Port, ENC1_A_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    b = (HAL_GPIO_ReadPin(ENC1_B_Port, ENC1_B_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    sw_last[0]  = (uint8_t)((a << 1) | b);
    sw_count[0] = 0;

    a = (HAL_GPIO_ReadPin(ENC3_A_Port, ENC3_A_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    b = (HAL_GPIO_ReadPin(ENC3_B_Port, ENC3_B_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    sw_last[1]  = (uint8_t)((a << 1) | b);
    sw_count[1] = 0;
}

/* HAL 的 EXTI 统一回调，由 EXTI2/EXTI3/EXTI4/EXTI15_10 的 IRQHandler 调用 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ENC1_A_Pin || GPIO_Pin == ENC1_B_Pin)
    {
        quad_update(0, ENC1_A_Port, ENC1_A_Pin, ENC1_B_Port, ENC1_B_Pin);
    }
    else if (GPIO_Pin == ENC3_A_Pin || GPIO_Pin == ENC3_B_Pin)
    {
        quad_update(1, ENC3_A_Port, ENC3_A_Pin, ENC3_B_Port, ENC3_B_Pin);
    }
}

/* 取走软件计数并清零。
 * 本函数只在 TIM6 中断（优先级 5）里被调用，
 * EXTI 中断优先级同为 5，同级不能互相抢占，所以读+清零是原子的。 */
static int16_t sw_take(uint8_t idx)
{
    int32_t v = sw_count[idx];
    sw_count[idx] = 0;
    return (int16_t)v;
}

/* ===================== 统一读取接口 ===================== */
int Read_Encoder(uint8_t idx)
{
    int cnt = 0;

    switch (idx)
    {
        case 0:   /* M1 左前：软件（PC2/PC3） */
            cnt = sw_take(0);
            break;

        case 1:   /* M2 右前：TIM8 硬件编码器 */
            cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
            __HAL_TIM_SET_COUNTER(&htim8, 0);
            break;

        case 2:   /* M3 左后：软件（PE4/PE12） */
            cnt = sw_take(1);
            break;

        case 3:   /* M4 右后：TIM5 硬件编码器 */
            cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
            __HAL_TIM_SET_COUNTER(&htim5, 0);
            break;

        default:
            break;
    }

    return cnt;
}
