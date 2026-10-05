/**
 * @file    BSP_BEMF_ADC.c
 * @brief   三相端电压 ADC 采样 (ADC1, 硬件定时触发, EOS 中断)
 */
#include "BSP_BEMF_ADC.h"
#include "cw32l012_adc.h"
#include "cw32l012_gpio.h"
#include "cw32l012_sysctrl.h"

volatile uint16_t g_bemf_raw[3];

static BSP_BEMF_Callback_t s_callback = 0;

void BSP_BEMF_Init(BSP_BEMF_Callback_t on_sample)
{
    ADC_InitTypeDef adc  = {0};
    GPIO_InitTypeDef gpio = {0};

    s_callback = on_sample;

    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_ADC_CLK_ENABLE();

    gpio.IT   = GPIO_IT_NONE;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_Init(CW_GPIOA, &gpio);
    PA00_ANALOG_ENABLE();
    PA01_ANALOG_ENABLE();
    PA02_ANALOG_ENABLE();

    adc.ADC_ClkDiv      = ADC_Clk_Div2;          /* 96/2 = 48 MHz */
    adc.ADC_ConvertMode = ADC_ConvertMode_Once;
    adc.ADC_SlaveMod    = ADC_SlaveMode_Disable;
    adc.ADC_SQREns      = ADC_SqrEns0to2;

    adc.ADC_IN0.ADC_InputChannel = ADC_InputCH0; /* PA00 = EA */
    adc.ADC_IN0.ADC_SampTime     = ADC_SampTime54Clk;
    adc.ADC_IN1.ADC_InputChannel = ADC_InputCH1; /* PA01 = EB */
    adc.ADC_IN1.ADC_SampTime     = ADC_SampTime54Clk;
    adc.ADC_IN2.ADC_InputChannel = ADC_InputCH2; /* PA02 = EC */
    adc.ADC_IN2.ADC_SampTime     = ADC_SampTime54Clk;

    ADC_Init(CW_ADC1, &adc);
    ADC_ClearITPendingAll(CW_ADC1);
    ADC_ITConfig(CW_ADC1, ADC_IT_EOS, ENABLE);
    ADC_Enable(CW_ADC1);
    ADC_ExtTrigCfg(CW_ADC1, ADC_TRIG_ATIMOC4REFC, ENABLE);

    NVIC_SetPriority(ADC1_IRQn, 1U);
    NVIC_EnableIRQ(ADC1_IRQn);
}

uint16_t BSP_BEMF_GetRaw(uint8_t phase)
{
    if (phase > BSP_BEMF_PHASE_C)
    {
        phase = BSP_BEMF_PHASE_C;
    }
    return g_bemf_raw[phase];
}

void BSP_BEMF_IRQHandler(void)
{
    if (ADC_GetITStatus(CW_ADC1, ADC_IT_EOS) == RESET)
    {
        return;
    }

    ADC_ClearITPendingBit(CW_ADC1, ADC_IT_EOS);
    ADC_GetSqr0Result(CW_ADC1, (uint16_t *)&g_bemf_raw[0]);
    ADC_GetSqr1Result(CW_ADC1, (uint16_t *)&g_bemf_raw[1]);
    ADC_GetSqr2Result(CW_ADC1, (uint16_t *)&g_bemf_raw[2]);

    if (s_callback != 0)
    {
        s_callback();
    }
}
