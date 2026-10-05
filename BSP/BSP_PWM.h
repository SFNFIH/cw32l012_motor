/**
 * @file    BSP_PWM.h
 * @brief   三相六步互补 PWM 输出 (ATIM, 中心对齐 15 kHz)
 *
 * 原理图 (Doc/SCH_Schematic1_1-P1, P3):
 *   MCU_AH = PB05 / ATIM_CH1     MCU_AL = PA15 / ATIM_CH1N
 *   MCU_BH = PB06 / ATIM_CH2     MCU_BL = PB03 / ATIM_CH2N
 *   MCU_CH = PB07 / ATIM_CH3     MCU_CL = PB04 / ATIM_CH3N
 *   栅极驱动 FD6288T, HIN/LIN 高有效, 硬件无死区, 由 ATIM 插入死区。
 *
 * 六步换相约定 (step 0..5):
 *   0: A+ B- (C 浮空)   1: A+ C- (B 浮空)   2: B+ C- (A 浮空)
 *   3: B+ A- (C 浮空)   4: C+ A- (B 浮空)   5: C+ B- (A 浮空)
 */
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_PWM_ARR             3199U     /* 96 MHz / (2*(3199+1)) = 15 kHz */
#define BSP_PWM_FREQ_HZ         15000U
#define BSP_PWM_DEADTIME        48U       /* 死区 ~0.5 us */
/* ADC 触发点: 谷底后 40 计数, 导通窗内采样 (浮空相 = Vbus/2 + 1.5*eX,
   不会被负电压削波; 过零参考由峰值包络自动跟踪) */
#define BSP_PWM_TRIG_LEVEL      40U       /* 谷底后 40 计数: 导通窗采样 */
#define BSP_PWM_DUTY_MIN        320U      /* 10% */
#define BSP_PWM_DUTY_MAX        2400U     /* 75% */

void BSP_PWM_Init(void);
void BSP_PWM_EnableOutputs(void);
void BSP_PWM_DisableOutputs(void);
void BSP_PWM_FloatAll(void);
void BSP_PWM_ApplyStep(uint8_t step);
void BSP_PWM_SetDuty(uint16_t duty);
uint16_t BSP_PWM_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
