/**
 * @file    BSP_KEY.c
 * @brief   PB10 按键: 20 ms 消抖, 单击/双击识别
 */
#include "BSP_KEY.h"
#include "cw32l012.h"
#include "cw32l012_gpio.h"
#include "cw32l012_sysctrl.h"

#define KEY_DEBOUNCE_MS     20U     /* 消抖时间 */
#define KEY_DOUBLE_MS       300U    /* 双击窗口 */
#define KEY_LONG_MS         1000U   /* 超过此时间不算单击 */

static uint8_t  s_last_raw = 1U;    /* 1=松开, 0=按下 */
static uint8_t  s_stable = 1U;
static uint16_t s_deb_cnt = 0U;
static uint16_t s_press_ms = 0U;
static uint16_t s_click_wait_ms = 0U;
static uint8_t  s_click_cnt = 0U;
static volatile uint8_t s_event = BSP_KEY_EVENT_NONE;

static uint8_t key_raw(void)
{
    return (PB10_GETVALUE() != 0U) ? 1U : 0U;
}

void BSP_KEY_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __SYSCTRL_GPIOB_CLK_ENABLE();

    g.IT   = GPIO_IT_NONE;
    g.Mode = GPIO_MODE_INPUT_PULLUP;
    g.Pins = GPIO_PIN_10;
    GPIO_Init(CW_GPIOB, &g);

    s_last_raw = key_raw();
    s_stable = s_last_raw;
    s_deb_cnt = 0U;
    s_press_ms = 0U;
    s_click_wait_ms = 0U;
    s_click_cnt = 0U;
    s_event = BSP_KEY_EVENT_NONE;
}

void BSP_KEY_Tick1ms(void)
{
    uint8_t raw = key_raw();

    if (raw != s_last_raw)
    {
        s_last_raw = raw;
        s_deb_cnt = 0U;
    }
    else if (s_deb_cnt < KEY_DEBOUNCE_MS)
    {
        s_deb_cnt++;
        if ((s_deb_cnt >= KEY_DEBOUNCE_MS) && (s_stable != raw))
        {
            s_stable = raw;

            if (s_stable == 0U)             /* 按下 */
            {
                s_press_ms = 0U;
            }
            else if (s_press_ms < KEY_LONG_MS)  /* 松开: 有效单击 */
            {
                if (s_click_cnt == 0U)
                {
                    s_click_cnt = 1U;
                    s_click_wait_ms = KEY_DOUBLE_MS;
                }
                else
                {
                    s_click_cnt = 0U;
                    s_click_wait_ms = 0U;
                    s_event = BSP_KEY_EVENT_DOUBLE;
                }
            }
        }
    }

    if (s_stable == 0U)
    {
        if (s_press_ms < 0xFFFFU) { s_press_ms++; }
    }
    else if (s_click_wait_ms > 0U)
    {
        s_click_wait_ms--;
        if (s_click_wait_ms == 0U)
        {
            if (s_click_cnt == 1U)
            {
                s_click_cnt = 0U;
                s_event = BSP_KEY_EVENT_CLICK;
            }
        }
    }
}

uint8_t BSP_KEY_GetEvent(void)
{
    uint8_t e = s_event;

    if (e != BSP_KEY_EVENT_NONE)
    {
        s_event = BSP_KEY_EVENT_NONE;
    }
    return e;
}

uint8_t BSP_KEY_IsPressed(void)
{
    return (s_stable == 0U) ? 1U : 0U;
}
