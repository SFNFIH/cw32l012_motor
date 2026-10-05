/**
 * @file    BSP_BEMF_ADC.h
 * @brief   反电势采样: PA00/PA01/PA02 (ADC1_CH0/1/2), ATIM_CH4 触发
 *
 * 原理图 P1: 相电压经 100K/5.1K 分压 + 470pF 滤波到 MCU_EA/EB/EC;
 * 与母线电压分压 (100K/5.1K) 比例一致, 可用同一 ADC 尺度比较过零点。
 * 每次序列转换完成 (EOS) 产生中断, 由回调通知无感控制层。
 */
#ifndef BSP_BEMF_ADC_H
#define BSP_BEMF_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_BEMF_PHASE_A    0U
#define BSP_BEMF_PHASE_B    1U
#define BSP_BEMF_PHASE_C    2U

/* [0]=EA(PA00) [1]=EB(PA01) [2]=EC(PA02), 中断内更新 */
extern volatile uint16_t g_bemf_raw[3];

typedef void (*BSP_BEMF_Callback_t)(void);

void BSP_BEMF_Init(BSP_BEMF_Callback_t on_sample);
uint16_t BSP_BEMF_GetRaw(uint8_t phase);
void BSP_BEMF_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BEMF_ADC_H */
