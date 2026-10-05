/**
 * @file    BSP_LED.c
 * @brief   PC13 LED, 低电平点亮
 */
#include "BSP_LED.h"
#include "cw32l012.h"
#include "cw32l012_gpio.h"
#include "cw32l012_sysctrl.h"

void BSP_LED_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __SYSCTRL_GPIOC_CLK_ENABLE();

    g.IT   = GPIO_IT_NONE;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pins = GPIO_PIN_13;
    GPIO_Init(CW_GPIOC, &g);

    BSP_LED_Off();
}

void BSP_LED_On(void)
{
    PC13_SETLOW();
}

void BSP_LED_Off(void)
{
    PC13_SETHIGH();
}
