/**
 * @file    BSP_LED.h
 * @brief   运行指示灯 PC13 (原理图 P2: +5V -> R64 10K -> LED1 -> MCU_LED)
 *          低电平点亮
 */
#ifndef BSP_LED_H
#define BSP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

void BSP_LED_Init(void);
void BSP_LED_On(void);
void BSP_LED_Off(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LED_H */
