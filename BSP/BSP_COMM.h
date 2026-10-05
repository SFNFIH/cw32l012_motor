/**
 * @file    BSP_COMM.h
 * @brief   调试串口 UART1: PB12=TX, PB11=RX, 115200-8-N-1 (原理图 P3)
 *
 * 发送为阻塞轮询; 接收在 ReadLine 里轮询组包, 返回完整一行 ('\n')。
 */
#ifndef BSP_COMM_H
#define BSP_COMM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_UART_Init(void);
void BSP_UART_Write(const char *s);
void BSP_UART_WriteBytes(const uint8_t *data, uint32_t len);

/** 非阻塞取一行命令. 返回长度, 0 = 暂无完整行 */
uint32_t BSP_UART_ReadLine(char *buf, uint32_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMM_H */
