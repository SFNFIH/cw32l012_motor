/**
 * @file    interrupts_cw32l012.c
 * @brief   中断向量: ADC1 = BEMF 序列完成; BTIM3 = 换相延时; BTIM1 = 1ms 任务
 */
#include "main.h"
#include "cw32l012_btim.h"
#include "BSP_BEMF_ADC.h"
#include "BSP_SIXSTEP.h"
#include "BSP_KEY.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    /* 异常时立即封锁六路 PWM 输出, 防止功率级失控 */
    CW_ATIM->BDTR_f.MOE = 0U;
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void ADC1_IRQHandler(void)
{
    BSP_BEMF_IRQHandler();
}

void BTIM1_IRQHandler(void)
{
    if (BTIM_GetITStatus(CW_BTIM1, BTIM_IT_UPDATE) != RESET)
    {
        BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_UPDATE);
        BSP_KEY_Tick1ms();
        BSP_SIXSTEP_Tick1ms();
    }
}

void BTIM3_HALLTIM_IRQHandler(void)
{
    BSP_SIXSTEP_OnDelayTimerIRQ();
}
