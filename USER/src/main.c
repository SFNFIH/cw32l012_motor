/**
 * @file    main.c
 * @brief   无感 BLDC 六步驱动
 *
 *   调速: 电位器 PA09 (上电默认), 也可 UART 手动给定
 *   启停/换向/遥测: UART1 (PB12/PB11, 115200)
 *     start | stop | dir | spd=<rpm> | auto | ?
 *   状态灯: PC13
 */
#include "main.h"
#include "BSP_LED.h"
#include "BSP_COMM.h"
#include "BSP_ADC.h"
#include "BSP_PWM.h"
#include "BSP_BEMF_ADC.h"
#include "BSP_SIXSTEP.h"
#include "BSP_KEY.h"
#include "cw32l012_systick.h"
#include "cw32l012_atim.h"
#include "cw32l012_iwdt.h"
#include <stdio.h>
#include <string.h>

#define APP_HCLK_HZ         96000000U

static uint8_t s_vofa_mode = 1U;    /* 1=JustFloat 波形, 0=文本遥测 */

static void clock_init(void);
static void handle_command(const char *line);
static void handle_key(uint8_t event);
static void led_task(uint32_t now);
static void telemetry_task(uint32_t now);

/** 无符号整数 -> IEEE754 单精度位模式 (避免软浮点开销) */
static uint32_t f32_bits(uint16_t v)
{
    int e = 15;
    uint32_t m;

    if (v == 0U)
    {
        return 0U;
    }
    while ((v & 0x8000U) == 0U)
    {
        v = (uint16_t)(v << 1);
        e--;
    }
    m = (uint32_t)(v & 0x7FFFU);
    return ((uint32_t)(e + 127) << 23) | (m << 8);
}

/** VOFA+ JustFloat: 2 通道 float + 帧尾 {00 00 80 7F} */
static void vofa_send(void)
{
    uint8_t  frame[12];
    uint32_t f[2];
    uint8_t  i;

    f[0] = f32_bits(BSP_SIXSTEP_GetDuty());       /* ch0: 占空比 (电位器直接控制) */
    f[1] = f32_bits(BSP_SIXSTEP_GetSpeedRpm());   /* ch1: 转速 */

    for (i = 0U; i < 2U; i++)
    {
        frame[i * 4U + 0U] = (uint8_t)(f[i] & 0xFFU);
        frame[i * 4U + 1U] = (uint8_t)((f[i] >> 8) & 0xFFU);
        frame[i * 4U + 2U] = (uint8_t)((f[i] >> 16) & 0xFFU);
        frame[i * 4U + 3U] = (uint8_t)((f[i] >> 24) & 0xFFU);
    }
    frame[8]  = 0x00U;
    frame[9]  = 0x00U;
    frame[10] = 0x80U;
    frame[11] = 0x7FU;

    BSP_UART_WriteBytes(frame, sizeof(frame));
}

static const char *state_name(SixStepState_t st)
{
    switch (st)
    {
    case SIXSTEP_ALIGN: return "ALIGN";
    case SIXSTEP_RAMP:  return "RAMP";
    case SIXSTEP_RUN:   return "RUN";
    case SIXSTEP_FAULT: return "FAULT";
    case SIXSTEP_BENCH: return "BENCH";
    case SIXSTEP_OPEN:  return "OPEN";
    case SIXSTEP_VF:    return "VF";
    default:            return "IDLE";
    }
}

int main(void)
{
    clock_init();
    InitTick(APP_HCLK_HZ);

    /* 独立看门狗: 2s, 主循环喂狗; 任何软件挂死都会自动复位 */
    __SYSCTRL_IWDT_CLK_ENABLE();
    IWDT_SetPeriod(IWDT_2_SECS);

    BSP_LED_Init();
    BSP_UART_Init();
    BSP_PWM_Init();
    BSP_ADC_Init();
    BSP_BEMF_Init(BSP_SIXSTEP_OnBemfSample);
    BSP_SIXSTEP_Init();
    BSP_KEY_Init();

    BSP_UART_Write("\r\nCW32L012 sensorless BLDC (six-step, V/f open loop)\r\n");
    BSP_UART_Write("key: click = start/stop, double click = dir\r\n");
    BSP_UART_Write("vofa: JustFloat 2ch (duty, rpm) @50Hz; vofa=0 -> text\r\n");
    BSP_UART_Write("cmd: start|stop|dir|spd=<rpm>|auto|motor=<2208|4006>|vofa=<0|1>|dbg|?\r\n");

    {
        uint32_t rf = SYSCTRL_GetAllRstFlag();
        char out[96];
        int k;

        SYSCTRL_ClearRstFlag(SYSCTRL_RESETFLAG_ALL);
        k = snprintf(out, sizeof(out),
                     "boot: reset flags=0x%02lX (b0=POR b3=LVD b4=IWDT b6=PIN b8=LOCKUP)\r\n",
                     (unsigned long)rf);
        if (k > 0) { BSP_UART_Write(out); }
    }

    while (1)
    {
        uint32_t now = GetTick();
        char line[48];
        uint8_t key = BSP_KEY_GetEvent();

        IWDT_Refresh();         /* 喂狗: 主循环挂死时自动复位 */

        if (key != BSP_KEY_EVENT_NONE)
        {
            handle_key(key);
        }

        led_task(now);
        telemetry_task(now);

        if (BSP_UART_ReadLine(line, sizeof(line)) > 0U)
        {
            handle_command(line);
        }
    }
}

static void clock_init(void)
{
    SYSCTRL_HSI_Enable(SYSCTRL_HSIOSC_DIV1);
    SYSCTRL_SysClk_Switch(SYSCTRL_SYSCLKSRC_HSI);
    SYSCTRL_HCLKPRS_Config(SYSCTRL_HCLK_DIV1);
    SYSCTRL_PCLKPRS_Config(SYSCTRL_PCLK_DIV1);
    SystemCoreClock = APP_HCLK_HZ;
}

/** 按键: 单击启停, 双击换向 */
static void handle_key(uint8_t event)
{
    if (event == BSP_KEY_EVENT_CLICK)
    {
        SixStepState_t st = BSP_SIXSTEP_GetState();

        if ((st == SIXSTEP_ALIGN) || (st == SIXSTEP_RAMP) ||
            (st == SIXSTEP_RUN)   || (st == SIXSTEP_BENCH))
        {
            BSP_SIXSTEP_Stop();
            BSP_UART_Write("key: stop\r\n");
        }
        else
        {
            /* 按键启动默认用电位器调速, 清掉 UART 手动给定 */
            BSP_SIXSTEP_SetManualTarget(0xFFFFU);
            BSP_SIXSTEP_Start();
            BSP_UART_Write("key: start (pot)\r\n");
        }
    }
    else if (event == BSP_KEY_EVENT_DOUBLE)
    {
        BSP_SIXSTEP_ToggleDirection();
        BSP_UART_Write("key: dir toggled\r\n");
    }
}

static void handle_command(const char *line)
{
    if ((strcmp(line, "start") == 0) || (strcmp(line, "s") == 0))
    {
        BSP_SIXSTEP_Start();
        BSP_UART_Write("ok: start\r\n");
    }
    else if ((strcmp(line, "stop") == 0) || (strcmp(line, "x") == 0))
    {
        BSP_SIXSTEP_Stop();
        BSP_UART_Write("ok: stop\r\n");
    }
    else if ((strcmp(line, "dir") == 0) || (strcmp(line, "d") == 0))
    {
        BSP_SIXSTEP_ToggleDirection();
        BSP_UART_Write("ok: dir toggled\r\n");
    }
    else if (strcmp(line, "auto") == 0)
    {
        BSP_SIXSTEP_SetManualTarget(0xFFFFU);
        BSP_UART_Write("ok: speed from pot\r\n");
    }
    else if (strncmp(line, "spd=", 4U) == 0)
    {
        uint32_t v = 0U;
        const char *p = line + 4U;

        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10U + (uint32_t)(*p - '0');
            p++;
        }
        BSP_SIXSTEP_SetManualTarget((uint16_t)v);   /* 上限由电机档案决定 */
        BSP_UART_Write("ok: manual speed\r\n");
    }
    else if (strncmp(line, "hold=", 5U) == 0)
    {
        uint32_t v = 0U;
        const char *p = line + 5U;

        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10U + (uint32_t)(*p - '0');
            p++;
        }
        BSP_SIXSTEP_BenchHold((uint16_t)v);
        BSP_UART_Write("ok: bench hold (stop to exit)\r\n");
    }
    else if (strncmp(line, "step=", 5U) == 0)
    {
        uint32_t v = 0U;
        const char *p = line + 5U;

        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10U + (uint32_t)(*p - '0');
            p++;
        }
        BSP_SIXSTEP_BenchStep((uint8_t)v);
        BSP_UART_Write("ok: bench step\r\n");
    }
    else if (strncmp(line, "ol=", 3U) == 0)
    {
        uint32_t period = 0U;
        uint32_t duty = 800U;
        const char *p = line + 3U;

        while ((*p >= '0') && (*p <= '9'))
        {
            period = period * 10U + (uint32_t)(*p - '0');
            p++;
        }
        if (*p == ',')
        {
            duty = 0U;
            p++;
            while ((*p >= '0') && (*p <= '9'))
            {
                duty = duty * 10U + (uint32_t)(*p - '0');
                p++;
            }
        }
        BSP_SIXSTEP_OpenLoop((uint16_t)period, (uint16_t)duty);
        BSP_UART_Write("ok: open loop (stop to exit)\r\n");
    }
    else if (strncmp(line, "motor=", 6U) == 0)
    {
        if (strcmp(line + 6U, "2208") == 0)
        {
            BSP_SIXSTEP_SetMotorProfile(BSP_SIXSTEP_PROFILE_2208);
            BSP_UART_Write("ok: profile 2208 (250rpm)\r\n");
        }
        else if (strcmp(line + 6U, "4006") == 0)
        {
            BSP_SIXSTEP_SetMotorProfile(BSP_SIXSTEP_PROFILE_4006);
            BSP_UART_Write("ok: profile 4006 (3000rpm)\r\n");
        }
        else
        {
            BSP_UART_Write("err: motor=2208|4006\r\n");
        }
    }
    else if (strncmp(line, "trig=", 5U) == 0)
    {
        uint32_t v = 0U;
        const char *p = line + 5U;

        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10U + (uint32_t)(*p - '0');
            p++;
        }
        if (v > BSP_PWM_ARR) { v = BSP_PWM_ARR; }
        ATIM_SetCompare4((uint16_t)v);
        {
            char out[48];
            int k = snprintf(out, sizeof(out), "ok: trig=%u\r\n", (unsigned)v);
            if (k > 0) { BSP_UART_Write(out); }
        }
    }
    else if (strncmp(line, "trace=", 6U) == 0)
    {
        uint32_t v = 0U;
        const char *p = line + 6U;

        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10U + (uint32_t)(*p - '0');
            p++;
        }
        BSP_SIXSTEP_TraceStart((uint16_t)v);
        BSP_UART_Write("ok: trace armed\r\n");
    }
    else if (strcmp(line, "dump") == 0)
    {
        char out[48];
        uint16_t n = BSP_SIXSTEP_TraceCount();
        uint16_t i;
        int k = snprintf(out, sizeof(out), "trace %u\r\n", (unsigned)n);

        if (k > 0) { BSP_UART_Write(out); }
        for (i = 0U; i < n; i++)
        {
            uint16_t v = 0U;
            uint8_t st = BSP_SIXSTEP_TraceGet(i, &v);

            k = snprintf(out, sizeof(out), "t,%u,%u,%u\r\n",
                         (unsigned)i, (unsigned)st, (unsigned)v);
            if (k > 0) { BSP_UART_Write(out); }
        }
        BSP_UART_Write("trace end\r\n");
    }
    else if (strncmp(line, "vofa=", 5U) == 0)
    {
        s_vofa_mode = (line[5] == '0') ? 0U : 1U;
        BSP_UART_Write((s_vofa_mode != 0U) ? "ok: vofa justfloat on\r\n"
                                           : "ok: text telemetry\r\n");
    }
    else if (strcmp(line, "hang") == 0)
    {
        BSP_UART_Write("ok: hang (watchdog test, will reset in ~2s)\r\n");
        while (1)
        {
        }
    }
    else if (strcmp(line, "dbg") == 0)
    {
        char out[208];
        int n = snprintf(out, sizeof(out),
                         "dbg st=%s step=%u key=%u prof=%u pot=%u vbus=%u isout=%u zero=%u bA=%u bB=%u bC=%u duty=%u pll=%lu zcf=%lu zcr=%lu\r\n",
                         state_name(BSP_SIXSTEP_GetState()),
                         (unsigned)BSP_SIXSTEP_GetStep(),
                         (unsigned)BSP_KEY_IsPressed(),
                         (unsigned)BSP_SIXSTEP_GetMotorProfile(),
                         (unsigned)BSP_ADC_GetPotRaw(),
                         (unsigned)BSP_ADC_GetVbusRaw(),
                         (unsigned)BSP_ADC_GetIsoutRaw(),
                         (unsigned)BSP_SIXSTEP_GetIsoutZero(),
                         (unsigned)g_bemf_raw[0],
                         (unsigned)g_bemf_raw[1],
                         (unsigned)g_bemf_raw[2],
                         (unsigned)BSP_SIXSTEP_GetDuty(),
                         (unsigned long)BSP_SIXSTEP_GetPllCount(),
                         (unsigned long)BSP_SIXSTEP_GetZcFallCount(),
                         (unsigned long)BSP_SIXSTEP_GetZcRiseCount());
        if (n > 0) { BSP_UART_Write(out); }
    }
    else
    {
        char out[80];
        int n = snprintf(out, sizeof(out), "err:%s\r\n", line);
        if (n > 0) { BSP_UART_Write(out); }
        BSP_UART_Write("cmd: start|stop|dir|spd=<rpm>|auto|motor=<2208|4006>|dbg|?\r\n");
    }
}

static void led_task(uint32_t now)
{
    switch (BSP_SIXSTEP_GetState())
    {
    case SIXSTEP_RUN:
        BSP_LED_On();
        break;

    case SIXSTEP_ALIGN:
    case SIXSTEP_RAMP:
        if (((now / 100U) & 1U) != 0U) { BSP_LED_On(); }
        else                           { BSP_LED_Off(); }
        break;

    case SIXSTEP_FAULT:
        if ((now % 600U) < 300U) { BSP_LED_On(); }
        else                     { BSP_LED_Off(); }
        break;

    default:
        BSP_LED_Off();
        break;
    }
}

static void telemetry_task(uint32_t now)
{
    static uint32_t last_tx = 0U;
    char line[128];
    int n;
    uint32_t period = (s_vofa_mode != 0U) ? 20U : 100U;

    if ((now - last_tx) < period)
    {
        return;
    }
    last_tx = now;

    if (s_vofa_mode != 0U)
    {
        vofa_send();            /* 50 Hz JustFloat 波形 */
        return;
    }

    n = snprintf(line, sizeof(line),
                 "st=%s step=%u dir=%d src=%s rpm=%u tgt=%u duty=%u vbus=%umV ibus=%umA zc=%lu miss=%u flt=%u\r\n",
                 state_name(BSP_SIXSTEP_GetState()),
                 (unsigned)BSP_SIXSTEP_GetStep(),
                 (int)BSP_SIXSTEP_GetDirection(),
                 (BSP_SIXSTEP_GetManualMode() != 0U) ? "MAN" : "POT",
                 (unsigned)BSP_SIXSTEP_GetSpeedRpm(),
                 (unsigned)BSP_SIXSTEP_GetTargetRpm(),
                 (unsigned)BSP_SIXSTEP_GetDuty(),
                 (unsigned)BSP_SIXSTEP_GetVbusMv(),
                 (unsigned)BSP_SIXSTEP_GetIsoutMa(),
                 (unsigned long)BSP_SIXSTEP_GetZcCount(),
                 (unsigned)BSP_SIXSTEP_GetMissCount(),
                 (unsigned)BSP_SIXSTEP_GetFault());

    if (n > 0)
    {
        BSP_UART_Write(line);
    }
}

/* 芯片库 base_types.h 固定开启 USE_FULL_ASSERT, 需要应用提供该桩函数 */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    CW_ATIM->BDTR_f.MOE = 0U;
    while (1)
    {
    }
}
