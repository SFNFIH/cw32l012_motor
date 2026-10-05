/**
 * @file    BSP_ADC.h
 * @brief   母线电压 / 调速电位器 / 母线电流 采样 (ADC2, 软件触发轮询)
 *
 * 原理图 P3:
 *   PA08 = ADC2_CH5    母线电压 (100K/5.1K 分压, 与 BEMF 分压同比例)
 *   PA09 = ADC2_CH6    调速电位器 RV1 (5V 分压)
 *   PB02 = ADC2_CH11   母线电流 ISOUT (片内 OPA1 放大 10 倍, 2.5V 偏置)
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_ADC_Init(void);
/** 触发一轮转换并等待完成 (~15 us). 建议 1 ms 调用一次 */
void BSP_ADC_Update(void);

uint16_t BSP_ADC_GetVbusRaw(void);
uint16_t BSP_ADC_GetPotRaw(void);
uint16_t BSP_ADC_GetIsoutRaw(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
