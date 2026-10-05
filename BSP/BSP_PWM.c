/**
 * @file    BSP_PWM.c
 * @brief   三相六步互补 PWM (ATIM, 15 kHz, 死区 0.5us)
 *
 * PWM 相的桥臂工作在互补 PWM; 下桥常通的相把比较模式设为 FORCED_INACTIVE,
 * 互补输出即为高 (下桥开); 浮空相关闭 CCxE/CCxNE, OSSR=1 使两个门极输入
 * 保持低电平, MOSFET 全关, 电机相线真正浮空。
 *
 * ATIM_CH4 只用于产生 ADC 触发 (OC4REFC), 不引到引脚。
 * 触发点设在计数峰值: 上桥全关 (OFF 窗), 浮空相端电压直接等于反电势。
 */
#include "BSP_PWM.h"
#include "cw32l012.h"
#include "cw32l012_atim.h"
#include "cw32l012_gpio.h"
#include "cw32l012_sysctrl.h"

/* step -> {上桥 PWM 相, 下桥常通相, 浮空相} */
static const uint8_t s_hi_phase[6]  = {0U, 0U, 1U, 1U, 2U, 2U};
static const uint8_t s_lo_phase[6]  = {1U, 2U, 2U, 0U, 0U, 1U};
static const uint8_t s_flt_phase[6] = {2U, 1U, 0U, 2U, 1U, 0U};

static uint16_t s_duty = BSP_PWM_DUTY_MIN;

static void pwm_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();

    g.IT   = GPIO_IT_NONE;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pins = GPIO_PIN_15;
    GPIO_Init(CW_GPIOA, &g);
    g.Pins = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_Init(CW_GPIOB, &g);

    PB05_AFx_ATIMCH1();
    PB06_AFx_ATIMCH2();
    PB07_AFx_ATIMCH3();
    PA15_AFx_ATIMCH1N();
    PB03_AFx_ATIMCH2N();
    PB04_AFx_ATIMCH3N();
}

/** 只改比较模式位, 不改 CCER, 便于一次写 CCER 完成换相 */
static void set_oc_mode(uint8_t phase, uint32_t mode)
{
    uint32_t m  = mode & 0x07U;
    uint32_t mh = (mode >> 3) & 0x01U;

    if (phase == 0U)
    {
        CW_ATIM->CCMR1CMP_f.OC1M  = m;
        CW_ATIM->CCMR1CMP_f.OC1MH = mh;
    }
    else if (phase == 1U)
    {
        CW_ATIM->CCMR1CMP_f.OC2M  = m;
        CW_ATIM->CCMR1CMP_f.OC2MH = mh;
    }
    else
    {
        CW_ATIM->CCMR2CMP_f.OC3M  = m;
        CW_ATIM->CCMR2CMP_f.OC3MH = mh;
    }
}

void BSP_PWM_Init(void)
{
    ATIM_InitTypeDef    tim = {0};
    ATIM_OCInitTypeDef  oc  = {0};

    pwm_gpio_init();
    __SYSCTRL_ATIM_CLK_ENABLE();

    tim.BufferState        = ENABLE;
    tim.CounterAlignedMode = ATIM_COUNT_ALIGN_MODE_CENTER_BOTH;
    tim.CounterDirection   = ATIM_COUNTING_UP;
    tim.CounterOPMode      = ATIM_OP_MODE_REPETITIVE;
    tim.Prescaler          = 0U;
    tim.ReloadValue        = BSP_PWM_ARR;
    tim.RepetitionCounter  = 0U;
    ATIM_Init(&tim);

    oc.OCPolarity       = ATIM_OCPOLARITY_NONINVERT;
    oc.OCMode           = ATIM_OCMODE_PWM1;
    oc.OCFastMode       = ATIM_OC_FAST_MODE_DISABLE;
    oc.OCInterruptState = DISABLE;
    oc.BufferState      = ENABLE;   /* CCR 预装载, 占空比更新无毛刺 */
    oc.OCComplement     = DISABLE;  /* 互补输出由 ApplyStep 统一打开 */
    ATIM_OC1Init(&oc);
    ATIM_OC2Init(&oc);
    ATIM_OC3Init(&oc);
    ATIM_OC4Init(&oc);              /* 仅做 ADC 触发 */

    ATIM_CH1Config(DISABLE);
    ATIM_CH2Config(DISABLE);
    ATIM_CH3Config(DISABLE);
    ATIM_CH4Config(ENABLE);

    ATIM_SetCompare1(s_duty);
    ATIM_SetCompare2(s_duty);
    ATIM_SetCompare3(s_duty);
    ATIM_SetCompare4(BSP_PWM_TRIG_LEVEL);

    ATIM_SetPWMDeadtime((int16_t)BSP_PWM_DEADTIME,
                        (int16_t)BSP_PWM_DEADTIME,
                        ENABLE);

    /* 关断/禁用通道时输出低电平: 上桥 HIN=0, 下桥 LIN=0, 避免门极悬空 */
    CW_ATIM->BDTR_f.OSSR = 1U;
    CW_ATIM->BDTR_f.OSSI = 1U;
    CW_ATIM->BDTR_f.BKE  = 0U;

    ATIM_CtrlPWMOutputs(DISABLE);
    ATIM_Cmd(ENABLE);               /* 计数器常开, ADC 触发不受 MOE 影响 */
}

void BSP_PWM_EnableOutputs(void)
{
    ATIM_CtrlPWMOutputs(ENABLE);
}

void BSP_PWM_DisableOutputs(void)
{
    ATIM_CtrlPWMOutputs(DISABLE);
}

void BSP_PWM_FloatAll(void)
{
    CW_ATIM->CCER &= ~0x777UL;      /* CC1E/1NE/2E/2NE/3E/3NE 全清 */
}

void BSP_PWM_ApplyStep(uint8_t step)
{
    uint8_t  i;
    uint8_t  hi;
    uint8_t  lo;
    uint8_t  fl;
    uint32_t ccer = 0U;

    step = (uint8_t)(step % 6U);
    hi   = s_hi_phase[step];
    lo   = s_lo_phase[step];
    fl   = s_flt_phase[step];

    for (i = 0U; i < 3U; i++)
    {
        if (i == hi)
        {
            set_oc_mode(i, ATIM_OCMODE_PWM1);
        }
        else if (i == lo)
        {
            set_oc_mode(i, ATIM_OCMODE_FORCED_INACTIVE);
        }
    }

    /* CCxE = bit 4*i, CCxNE = bit 4*i+2; 浮空相两位都不置, 整体一次写入 */
    for (i = 0U; i < 3U; i++)
    {
        if (i != fl)
        {
            ccer |= (1UL << (4U * i)) | (1UL << (4U * i + 2U));
        }
    }

    CW_ATIM->CCER = (CW_ATIM->CCER & ~0x777UL) | ccer;
}

void BSP_PWM_SetDuty(uint16_t duty)
{
    if (duty < 1U) { duty = 1U; }
    if (duty > BSP_PWM_ARR) { duty = BSP_PWM_ARR; }

    s_duty = duty;
    ATIM_SetCompare1(duty);
    ATIM_SetCompare2(duty);
    ATIM_SetCompare3(duty);
}

uint16_t BSP_PWM_GetDuty(void)
{
    return s_duty;
}
