/**
 * @file    BSP_COMM.c
 * @brief   UART1 轮询收发 (命令/遥测), 带超时/溢出保护
 */
#include "BSP_COMM.h"
#include "cw32l012.h"
#include "cw32l012_gpio.h"
#include "cw32l012_sysctrl.h"
#include "cw32l012_uart.h"

#define BSP_UART_BAUD        115200U
#define BSP_UART_PCLK_HZ     96000000U
#define BSP_UART_TX_GUARD    200000U   /* TX 等待上限, 防止硬件异常时死循环 */
#define BSP_UART_RX_GUARD    64U       /* 单次最多处理 64 字节 */

static char    s_rx_buf[64];
static uint32_t s_rx_len = 0U;

void BSP_UART_Init(void)
{
    UART_InitTypeDef uart = {0};
    GPIO_InitTypeDef gpio = {0};

    __SYSCTRL_GPIOB_CLK_ENABLE();
    __SYSCTRL_UART1_CLK_ENABLE();

    gpio.IT   = GPIO_IT_NONE;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;        /* TX */
    gpio.Pins = GPIO_PIN_12;
    GPIO_Init(CW_GPIOB, &gpio);
    gpio.Mode = GPIO_MODE_INPUT_PULLUP;     /* RX */
    gpio.Pins = GPIO_PIN_11;
    GPIO_Init(CW_GPIOB, &gpio);
    PB12_AFx_UART1TXD();
    PB11_AFx_UART1RXD();

    uart.UART_BaudRate  = BSP_UART_BAUD;
    uart.UART_Over      = UART_Over_16;
    uart.UART_Source    = UART_Source_PCLK;
    uart.UART_UclkFreq  = BSP_UART_PCLK_HZ;
    uart.UART_StartBit  = UART_StartBit_FE;
    uart.UART_StopBits  = UART_StopBits_1;
    uart.UART_Parity    = UART_Parity_No;
    uart.UART_Mode      = UART_Mode_Rx | UART_Mode_Tx;
    uart.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    UART_Init(CW_UART1, &uart);

    s_rx_len = 0U;
}

static void uart_tx_byte(uint8_t b)
{
    uint32_t guard = BSP_UART_TX_GUARD;

    while ((UART_GetFlagStatus(CW_UART1, UART_FLAG_TXE) == RESET) && (guard > 0U))
    {
        guard--;
    }
    if (guard == 0U)
    {
        return;                     /* TX 异常: 丢弃, 绝不阻塞主循环 */
    }
    UART_SendData(CW_UART1, b);
}

void BSP_UART_WriteBytes(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        uart_tx_byte(data[i]);
    }
}

void BSP_UART_Write(const char *s)
{
    const char *p = s;

    while (*p != '\0')
    {
        uart_tx_byte((uint8_t)(*p));
        p++;
    }
}

uint32_t BSP_UART_ReadLine(char *buf, uint32_t max_len)
{
    uint32_t guard = BSP_UART_RX_GUARD;

    /* 溢出/噪声错误: 清标志, 防止 RC 卡死导致主循环挂死 */
    if (UART_GetFlagStatus(CW_UART1, UART_FLAG_ORE) != RESET)
    {
        (void)UART_ReceiveData(CW_UART1);
        UART_ClearFlag(CW_UART1, UART_FLAG_ORE);
        UART_ClearFlag(CW_UART1, UART_FLAG_RC);
        s_rx_len = 0U;
    }
    if (UART_GetFlagStatus(CW_UART1, UART_FLAG_NE) != RESET)
    {
        UART_ClearFlag(CW_UART1, UART_FLAG_NE);
    }

    while ((UART_GetFlagStatus(CW_UART1, UART_FLAG_RC) != RESET) && (guard > 0U))
    {
        char c = (char)UART_ReceiveData(CW_UART1);

        UART_ClearFlag(CW_UART1, UART_FLAG_RC);
        guard--;

        if ((c == '\n') || (c == '\r'))
        {
            if (s_rx_len > 0U)
            {
                uint32_t n = s_rx_len;

                if (n > (max_len - 1U))
                {
                    n = max_len - 1U;
                }
                for (uint32_t i = 0U; i < n; i++)
                {
                    buf[i] = s_rx_buf[i];
                }
                buf[n] = '\0';
                s_rx_len = 0U;
                return n;
            }
        }
        else if (s_rx_len < (sizeof(s_rx_buf) - 1U))
        {
            s_rx_buf[s_rx_len] = c;
            s_rx_len++;
        }
    }

    return 0U;
}
