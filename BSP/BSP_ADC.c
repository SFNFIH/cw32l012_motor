/**
 * @file    BSP_ADC.c
 * @brief   ADC2 三通道: 母线电压 / 电位器 / 母线电流
 *
 * 母线电流 ISOUT 来自片内 OPA1 (原理图 P2): 外部电阻网络构成差分放大,
 * OPA1_INP2=PA06, OPA1_INN2=PA07, OPA1_OUT=PB00, 输出经 R45/C26 到 PB02。
 */
#include "BSP_ADC.h"
#include "cw32l012_adc.h"
#include "cw32l012_gpio.h"
#include "cw32l012_opa.h"
#include "cw32l012_sysctrl.h"

static volatile uint16_t s_vbus_raw  = 0U;
static volatile uint16_t s_pot_raw   = 0U;
static volatile uint16_t s_isout_raw = 0U;

/** 使能 OPA1 独立运放模式, 否则 ISOUT 悬空、电流采样无效 */
static void isout_amp_init(void)
{
    OPA_InitTypeDef opa = {0};

    __SYSCTRL_OPA_CLK_ENABLE();

    AFx_OPA1INP2_PA06();
    AFx_OPA1INN2_PA07();
    AFx_OPA1OUT_PB00();

    opa.InputP   = OPA_INPUT_INP2;
    opa.InputN   = OPA_INPUT_INN2;
    opa.Bias     = OPA_BIAS_8UA_4US;
    opa.PgaGain  = OPA_PGA_GAIN2;           /* 独立模式不使用 */
    opa.WorkMode = OPA_WORKMODE_STANDALONE;
    OPA_Init(CW_OPA1, &opa);
    OPA_Start(CW_OPA1);
}

void BSP_ADC_Init(void)
{
    ADC_InitTypeDef  adc  = {0};
    GPIO_InitTypeDef gpio = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();
    __SYSCTRL_ADC_CLK_ENABLE();

    isout_amp_init();

    gpio.IT   = GPIO_IT_NONE;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pins = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_Init(CW_GPIOA, &gpio);
    gpio.Pins = GPIO_PIN_2;
    GPIO_Init(CW_GPIOB, &gpio);
    PA08_ANALOG_ENABLE();
    PA09_ANALOG_ENABLE();
    PB02_ANALOG_ENABLE();

    adc.ADC_ClkDiv      = ADC_Clk_Div2;
    adc.ADC_ConvertMode = ADC_ConvertMode_Once;
    adc.ADC_SlaveMod    = ADC_SlaveMode_Disable;
    adc.ADC_SQREns      = ADC_SqrEns0to2;

    adc.ADC_IN0.ADC_InputChannel = ADC_InputCH5;   /* PA08 母线电压 */
    adc.ADC_IN0.ADC_SampTime     = ADC_SampTime54Clk;
    adc.ADC_IN1.ADC_InputChannel = ADC_InputCH6;   /* PA09 电位器 */
    adc.ADC_IN1.ADC_SampTime     = ADC_SampTime54Clk;
    adc.ADC_IN2.ADC_InputChannel = ADC_InputCH11;  /* PB02 母线电流 */
    adc.ADC_IN2.ADC_SampTime     = ADC_SampTime54Clk;

    ADC_Init(CW_ADC2, &adc);
    ADC_ClearITPendingAll(CW_ADC2);
    ADC_Enable(CW_ADC2);
}

void BSP_ADC_Update(void)
{
    uint32_t guard = 4000U;

    ADC_ClearITPendingBit(CW_ADC2, ADC_IT_EOS);
    ADC_SoftwareStartConvCmd(CW_ADC2, ENABLE);

    while ((ADC_GetITStatus(CW_ADC2, ADC_IT_EOS) == RESET) && (guard > 0U))
    {
        guard--;
    }
    if (guard == 0U)
    {
        return;
    }

    ADC_ClearITPendingBit(CW_ADC2, ADC_IT_EOS);
    s_vbus_raw  = ADC_GetConversionValue(CW_ADC2, ADC_RESULT_0);
    s_pot_raw   = ADC_GetConversionValue(CW_ADC2, ADC_RESULT_1);
    s_isout_raw = ADC_GetConversionValue(CW_ADC2, ADC_RESULT_2);
}

uint16_t BSP_ADC_GetVbusRaw(void)
{
    return s_vbus_raw;
}

uint16_t BSP_ADC_GetPotRaw(void)
{
    return s_pot_raw;
}

uint16_t BSP_ADC_GetIsoutRaw(void)
{
    return s_isout_raw;
}
