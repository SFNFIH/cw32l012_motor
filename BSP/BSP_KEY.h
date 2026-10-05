/**
 * @file    BSP_KEY.h
 * @brief   按键输入 PB10 (原理图 P3: MCU_EN, 板上有 3.3K 上拉, 按下为低)
 *
 * 1 ms 周期消抖; 事件: 单击 / 双击 (双击用于换向)。
 */
#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_KEY_EVENT_NONE     0U
#define BSP_KEY_EVENT_CLICK    1U
#define BSP_KEY_EVENT_DOUBLE   2U

void BSP_KEY_Init(void);
/** 1 ms 周期调用 (BTIM1 中断内) */
void BSP_KEY_Tick1ms(void);
uint8_t BSP_KEY_GetEvent(void);
uint8_t BSP_KEY_IsPressed(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEY_H */
