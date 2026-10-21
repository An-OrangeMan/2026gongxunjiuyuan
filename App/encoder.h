#ifndef __ENCODER_H
#define __ENCODER_H

#include "tim.h"

/* ===== 4 个 GM520 编码器映射 =====
 * idx  电机     编码器类型      引脚
 *  0   左前 M1  软件正交解码    PC2(A) / PC3(B)
 *  1   右前 M2  TIM8 硬件编码器 PI5(A) / PI6(B)
 *  2   左后 M3  软件正交解码    PE4(A) / PE12(B)
 *  3   右后 M4  TIM5 硬件编码器 PH10(A) / PH11(B)
 *
 * 软件解码用 EXTI 双边沿 + 状态表，和硬件 TIM 编码器的 TI12 模式一样是 4 倍频，
 * 所以 4 个电机的计数量纲完全一致。
 */
#define ENC1_A_Pin      GPIO_PIN_2
#define ENC1_A_Port     GPIOC
#define ENC1_B_Pin      GPIO_PIN_3
#define ENC1_B_Port     GPIOC

#define ENC3_A_Pin      GPIO_PIN_4
#define ENC3_A_Port     GPIOE
#define ENC3_B_Pin      GPIO_PIN_12
#define ENC3_B_Port     GPIOE

/* 记录上电瞬间 A/B 相电平，必须在 MX_GPIO_Init() 之后调用 */
void Software_Encoder_Init(void);

/* 读取本周期编码器增量并清零（M 法测速），返回带符号计数值 */
int Read_Encoder(uint8_t idx);

#endif
