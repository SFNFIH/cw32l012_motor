/**
 * @file    BSP_SIXSTEP.c
 * @brief   无感 BLDC 六步换相控制: 对齐 -> 开环强拖 -> 反电势过零闭环
 *
 * 反电势采样 (ATIM 计数峰值, 上桥全关的 OFF 窗):
 *   PWM 关断期间上桥全关, 两导通相下桥打开, 中性点 = 0,
 *   浮空相端电压直接等于该相反电势 eX, 过零参考为 0 (逐相标定零点)。
 *   与占空比、母线电压、分压比误差无关。
 *   过零确认后再延时 30° (半拍) 换相, 即通常的 "过零 + 30°" 换相法。
 *
 * 定时:
 *   BTIM2  1 MHz 自由计数 -> 测步距 (60° 电角度)
 *   BTIM3  1 MHz 单次 -> 30° 延时换相 / 开环步进
 *
 * 保护: 母线过压/欠压, 母线过流 (ISOUT), 启动超时, 堵转 (连续丢失过零)
 */
#include "BSP_SIXSTEP.h"
#include "BSP_PWM.h"
#include "BSP_BEMF_ADC.h"
#include "BSP_ADC.h"
#include "cw32l012_btim.h"
#include "cw32l012_sysctrl.h"

/* ================= 可调参数 ================= */
#define MOTOR_POLE_PAIRS        7U      /* 4006 KV380: 14 极 = 7 对极 */

#define ALIGN_STEP_INDEX        0U      /* 对齐向量: A+ B- */
#define ALIGN_DUTY              250U    /* ~7.8% (限制起步电流) */
#define ALIGN_TIME_MS           300U

#define RAMP_PERIOD_START_US    60000U  /* 开环起始步距 ~2.8Hz 电频率 */
#define RAMP_PERIOD_END_US      12000U  /* 开环最快步距 (进入 V/f 的交接点) */
#define RAMP_PERIOD_DECAY_DIV   64U     /* 每步衰减 1/64 */
#define RAMP_DUTY_START         400U
#define RAMP_TIMEOUT_MS         12000U
#define LOCK_ZC_STREAK          4U      /* 连续过零次数达标 -> 闭环 */

#define ZC_ARM_CNT              6       /* 过零确认前 "看到另一侧" 的最小幅度 */
#define ZC_CONF_CNT             2       /* 过零确认回差 (OFF 窗低阈值) */
#define ZC_PEAK_THR             12U     /* OFF 窗阈值: ADC 削底(0-8)之上一点 */
#define ZC_CONFIRM_CNT          2U      /* 连续采样确认次数 */

/* BEMF 相位锁定 (PLL) 试验: 本板模拟前端采样窗内信息不足, 暂关闭 */
#define CFG_PLL                 0U
#define PLL_EV_FRAC_NUM         42      /* 期望事件位置 = period * 42/64 (~66%) */
#define PLL_EV_FRAC_DEN         64
#define PLL_TRIM_DIV            32      /* 每次事件修正 period 误差的 1/32 */
#define PLL_TRIM_NDIV           200     /* 单次修正上限 0.5% */
#define ZC_BLANK_DIV            8U      /* 换相后盲区 = 步距/8 */
#define ZC_BLANK_MIN_US         150U

#define RUN_TIMEOUT_NUM         5U      /* 无过零超时 = 步距 * 5/4 */
#define RUN_TIMEOUT_DEN         4U
#define RUN_MISS_LIMIT          6U      /* 连续强换相次数上限 -> 退回开环 */
#define RELOCK_LIMIT            4U      /* 连续重锁失败次数 -> 故障 */

/* 失速检测 (V/f 模式): 指令转速较高但反电势幅值消失 -> 停机 */
#define STALL_RPM_MIN           300U    /* 高于此转速才检测 */
#define STALL_AMP_CNT           25U     /* 反电势幅值低于此值视为停转 */
#define STALL_MS                300U    /* 持续时间 */

#define ZC_DELAY_MIN_US         120U
#define ZC_DELAY_MAX_US         30000U

#define TARGET_RPM_MIN          50U     /* 低于此值视为油门归零 -> 停机 */
#define POT_START_RAW           200U    /* 电位器阈值: >200 启动, <200 停机 */

#define VBUS_MIN_MV             7000U
#define VBUS_MAX_MV             30000U
#define VBUS_MV_PER_COUNT_NUM   1051U   /* mV = raw*5000/4095 * 1051/51 */
#define VBUS_MV_PER_COUNT_DEN   51U

#define OC_RAW_THRESHOLD        1638U   /* 20A * (10mΩ*10) 对应 ADC 计数 */
#define OC_CONFIRM_CNT          3U
#define OC_BLANK_MS             300U    /* 启动后过流保护屏蔽时间 */

#define SPEED_LOOP_ENABLE       1U
#define SPEED_LOOP_PERIOD_MS    10U
#define RPM_FF_MAX              340U    /* 前馈: 12V 满占空比估计机械转速 (2208) */
#define INTEG_LIMIT             100000
#define DUTY_SLEW_PER_CYCLE     16      /* 每 10ms 占空比最大变化量 */

/* 闭环过零: 1=启用 (需 BEMF 分压加 1.2V 偏置的硬件改动), 0=纯开环 V/f */
#define CFG_CLOSED_LOOP         0U

/* 电机配置档案: 2208 / 4006 的开环 V/f 参数 */
typedef struct
{
    uint16_t rpm_max;           /* 电位器满量程机械转速 */
    uint16_t duty_base;         /* duty = base + rpm * per10 / 10 */
    uint16_t duty_per10;
    uint16_t duty_min;
    uint16_t period_min_us;     /* 最高电频率对应步距 */
    uint16_t ramp_end_us;       /* 开环斜坡结束 (进入闭环/交接) 步距 */
    uint16_t ramp_duty_step;
    uint16_t rpm_ff;            /* 速度环前馈: 满占空比估计转速 */
    uint16_t lock_period_us;    /* 允许闭环锁定的最低速步距 */
} SixStepProfile_t;

static const SixStepProfile_t s_profiles[2] =
{
    /* 2208: 8Ω / KV~35, 50..250 rpm, duty = 240 + 8.4*rpm */
    { 250U,  240U, 84U, 400U, 5714U, 12000U, 5U, 340U,  12000U },
    /* 4006: 0.17Ω / KV360, 占空比直接映射, duty = 185 + 0.7*rpm */
    { 4000U, 185U, 7U,  250U, 300U,  1500U,  4U, 4300U, 3000U  },
};

static uint8_t s_profile = BSP_SIXSTEP_PROFILE_4006;   /* 默认 4006 */

/* V/f 开环运行参数 (各档案共用上限) */
#define VF_PERIOD_MAX_US        60000U
#define VF_DUTY_MAX             3100U   /* 96.9% (占空比上限, 解除转速限制) */
#define VF_DUTY_SLEW            8U      /* 每 10ms 占空比最大变化 (约 3V/s) */
#define SOFT_CUR_LIMIT_CNT      280U    /* 软限流: 约 3.4A, 超过则暂停抬压 */

/* ================= 六步表 ================= */
/* step: 0:A+B-  1:A+C-  2:B+C-  3:B+A-  4:C+A-  5:C+B- */
static const uint8_t s_float_phase[6]  = {2U, 1U, 0U, 2U, 1U, 0U};
/* 浮空相过零方向 (实测标定): 1=下降沿, 0=上升沿 */
static const uint8_t s_zc_fall_fwd[6]  = {0U, 1U, 0U, 1U, 0U, 1U};

/* ================= 状态 ================= */
static volatile SixStepState_t s_state = SIXSTEP_IDLE;
static volatile uint8_t  s_step = 0U;
static volatile int8_t   s_dir = 1;
static volatile uint8_t  s_fault = FAULT_NONE;
static volatile uint16_t s_duty = BSP_PWM_DUTY_MIN;

static volatile uint16_t s_period_us = 0U;       /* 60° 步距 */
static volatile uint16_t s_ramp_period_us = RAMP_PERIOD_START_US;
static volatile uint16_t s_open_period_us = 20000U;
static volatile uint16_t s_vf_period_us = RAMP_PERIOD_END_US;
static volatile uint16_t s_target_rpm = 0U;
static volatile uint16_t s_speed_rpm = 0U;
static volatile uint16_t s_threshold = 0U;       /* 过零参考 = Vbus/2 */

static volatile uint16_t s_vbus_raw_f = 0U;
static volatile uint16_t s_isout_raw = 0U;
static volatile uint16_t s_isout_zero = 2048U;   /* 电流零偏 (上电/停机时校准) */
static volatile uint16_t s_pot_raw = 0U;

static volatile uint8_t  s_manual = 0U;
static volatile uint16_t s_manual_rpm = 0U;
static volatile uint8_t  s_pot_arm = 1U;      /* 电位器启动允许 (停机后需回低位重新解除) */

static volatile uint8_t  s_zc_armed = 0U;
static volatile uint8_t  s_zc_cnt = 0U;
static volatile uint8_t  s_zc_found = 0U;
static volatile uint16_t s_zc_streak = 0U;
static volatile uint32_t s_zc_total = 0U;
static volatile uint32_t s_zc_fall = 0U;
static volatile uint32_t s_zc_rise = 0U;
static volatile uint16_t s_miss = 0U;             /* 连续强换相次数 */
static volatile uint8_t  s_relock = 0U;           /* 闭环失步后重锁次数 */
static volatile uint16_t s_zc_zero[3] = {4U, 4U, 4U};  /* 各相削底电平+4 (对齐时标定) */
static uint8_t  s_cal_idx = 0U;
static uint16_t s_cal_ms = 0U;

/* 调试采样跟踪: 记录浮空相采样 (step, 采样值) */
#define TRACE_MAX   256U
static uint8_t  s_trace_buf[TRACE_MAX * 3U];   /* [0]=sample_lo [1]=sample_hi [2]=step */
static uint16_t s_trace_total = 0U;
static uint16_t s_trace_count = 0U;
static uint8_t  s_trace_skip = 0U;

/* BEMF 相位锁定 (PLL) 状态 */
static volatile uint16_t s_pospeak = 0U;      /* 虚拟中性点差值正峰值跟踪 */
static volatile uint16_t s_bemf_amp = 0U;     /* 反电势幅值 (失速检测用) */
static volatile uint16_t s_stall_ms = 0U;
static volatile uint8_t  s_ev_seen = 0U;      /* 本步已产生相位基准事件 */
static volatile uint32_t s_pll_cnt = 0U;
static int16_t  s_prev_diff = 0;
static volatile uint16_t s_above_cnt = 0U;    /* 连续处于半峰值之上的采样数 */

static uint16_t s_align_ms = 0U;
static uint16_t s_ramp_ms = 0U;
static uint16_t s_oc_cnt = 0U;
static uint16_t s_oc_blank_ms = 0U;
static uint16_t s_speed_div = 0U;
static int32_t  s_integ = 0;

/* ================= 内部函数 ================= */
static uint16_t now_us(void)
{
    return BTIM_GetCounter(CW_BTIM2);
}

static void pwm_all_off(void)
{
    BTIM_Cmd(CW_BTIM3, DISABLE);
    BSP_PWM_FloatAll();
    BSP_PWM_DisableOutputs();
}

static void enter_fault(uint8_t code)
{
    s_fault = code;
    s_state = SIXSTEP_FAULT;
    s_pot_arm = 0U;                 /* 故障后需电位器回低位再启动 */
    pwm_all_off();
}

/** 软限流: 母线电流超过阈值返回 1 (加速时暂停抬压用) */
static uint8_t cur_high(void)
{
    int32_t d = (int32_t)s_isout_raw - (int32_t)s_isout_zero;

    if (d < 0) { d = -d; }
    return (d > (int32_t)SOFT_CUR_LIMIT_CNT) ? 1U : 0U;
}

/** BEMF 相位锁定: 事件晚到 -> 周期加长 (让转子追上); 早到 -> 周期缩短 */
static void pll_trim(uint16_t t_us)
{
    int32_t expect = ((int32_t)s_period_us * PLL_EV_FRAC_NUM) / PLL_EV_FRAC_DEN;
    int32_t err = (int32_t)t_us - expect;
    int32_t trim = err / PLL_TRIM_DIV;
    int32_t max_trim = (int32_t)s_vf_period_us / PLL_TRIM_NDIV;
    int32_t np;

    if (max_trim < 1) { max_trim = 1; }
    if (trim > max_trim) { trim = max_trim; }
    if (trim < -max_trim) { trim = -max_trim; }

    np = (int32_t)s_vf_period_us + trim;
    if (np < (int32_t)s_profiles[s_profile].period_min_us)
    {
        np = (int32_t)s_profiles[s_profile].period_min_us;
    }
    if (np > (int32_t)VF_PERIOD_MAX_US) { np = (int32_t)VF_PERIOD_MAX_US; }
    s_vf_period_us = (uint16_t)np;
}

/** 预约 us 微秒后换相 (BTIM3 单次定时) */
static void schedule_commute(uint16_t us){
    if (us < ZC_DELAY_MIN_US) { us = ZC_DELAY_MIN_US; }
    if (us > ZC_DELAY_MAX_US) { us = ZC_DELAY_MAX_US; }

    BTIM_Cmd(CW_BTIM3, DISABLE);
    BTIM_ClearITPendingBit(CW_BTIM3, BTIM_IT_UPDATE);
    BTIM_SetAutoreload(CW_BTIM3, (uint16_t)(us - 1U));
    BTIM_SetCounter(CW_BTIM3, 0U);
    BTIM_Cmd(CW_BTIM3, ENABLE);
}

/** 换到下一个向量, 同时测量/滤波步距 */
static void commute_next(void)
{
    if ((s_state == SIXSTEP_RUN) && (s_period_us != 0U))
    {
        uint16_t t = now_us();

        if ((t > 100U) && (t < 30000U))
        {
            s_period_us = (uint16_t)(((uint32_t)s_period_us * 3U + t) / 4U);
        }
    }

    if (s_dir > 0)
    {
        s_step = (uint8_t)((s_step + 1U) % 6U);
    }
    else
    {
        s_step = (uint8_t)((s_step + 5U) % 6U);
    }

    BSP_PWM_ApplyStep(s_step);

    BTIM_SetCounter(CW_BTIM2, 0U);
    s_zc_found = 0U;
    s_zc_armed = 0U;
    s_zc_cnt = 0U;
    s_ev_seen = 0U;
    s_above_cnt = 0U;
}

/** 对齐结束: 进入开环强拖 */
static void start_ramp(void)
{
    s_state = SIXSTEP_RAMP;
    s_ramp_period_us = RAMP_PERIOD_START_US;
    s_period_us = RAMP_PERIOD_START_US;
    s_duty = RAMP_DUTY_START;
    BSP_PWM_SetDuty(s_duty);
    s_zc_streak = 0U;
    s_ramp_ms = 0U;

    commute_next();
    schedule_commute(s_ramp_period_us);
}

/** 斜坡结束: 进入 V/f 开环运行 (电位器控压控频) */
static void enter_vf(void)
{
    s_state = SIXSTEP_VF;
    s_vf_period_us = s_ramp_period_us;
    s_period_us = s_vf_period_us;
    commute_next();
    schedule_commute(s_vf_period_us);
}

/** V/f: 由 (命令) 步距求占空比 */
static uint16_t vf_duty_for_period(uint16_t period_us)
{
    const SixStepProfile_t *pf = &s_profiles[s_profile];
    uint32_t rpm = 1428571UL / (uint32_t)period_us;
    uint32_t duty;

    if (rpm < TARGET_RPM_MIN) { rpm = TARGET_RPM_MIN; }
    duty = (uint32_t)pf->duty_base + (rpm * pf->duty_per10) / 10UL;
    if (duty < pf->duty_min) { duty = pf->duty_min; }
    if (duty > VF_DUTY_MAX) { duty = VF_DUTY_MAX; }
    return (uint16_t)duty;
}

/** V/f 反算: 占空比 -> 步距 (保持压频一致) */
static uint16_t vf_period_for_duty(uint16_t duty)
{
    const SixStepProfile_t *pf = &s_profiles[s_profile];
    uint32_t rpm;
    uint32_t period;

    if (duty <= pf->duty_base) { return VF_PERIOD_MAX_US; }

    rpm = ((uint32_t)(duty - pf->duty_base) * 10U) / pf->duty_per10;
    if (rpm < TARGET_RPM_MIN) { rpm = TARGET_RPM_MIN; }

    period = 1428571UL / rpm;
    if (period < pf->period_min_us) { period = pf->period_min_us; }
    if (period > VF_PERIOD_MAX_US) { period = VF_PERIOD_MAX_US; }
    return (uint16_t)period;
}

/** V/f: 目标转速 -> 步距/占空比 (按电机档案, 限斜率) */
/** V/f 运行更新: 电位器直接给占空比 (频率按 V/f 反算跟随); 手动模式按转速 */
static void vf_update(void)
{
    const SixStepProfile_t *pf = &s_profiles[s_profile];
    uint32_t want_duty;
    uint32_t want_period;

    if (s_manual != 0U)
    {
        uint32_t rpm = (s_manual_rpm < TARGET_RPM_MIN) ? TARGET_RPM_MIN : s_manual_rpm;
        uint32_t p = 1428571UL / rpm;

        if (p < pf->period_min_us) { p = pf->period_min_us; }
        if (p > VF_PERIOD_MAX_US) { p = VF_PERIOD_MAX_US; }
        want_duty = vf_duty_for_period((uint16_t)p);
    }
    else
    {
        /* 电位器直接映射占空比 (200..4095 -> duty_min..96.9%) */
        if (s_pot_raw <= POT_START_RAW)
        {
            want_duty = pf->duty_min;
        }
        else
        {
            want_duty = (uint32_t)pf->duty_min
                        + ((uint32_t)(s_pot_raw - POT_START_RAW)
                           * (VF_DUTY_MAX - pf->duty_min))
                          / (4095U - POT_START_RAW);
        }
        if (want_duty > VF_DUTY_MAX) { want_duty = VF_DUTY_MAX; }
    }

    /* 占空比限斜率 (加速受软限流约束) */
    if (s_duty > (uint16_t)(want_duty + VF_DUTY_SLEW))
    {
        s_duty = (uint16_t)(s_duty - VF_DUTY_SLEW);
    }
    else if ((uint32_t)s_duty + VF_DUTY_SLEW < want_duty)
    {
        if (cur_high() == 0U)
        {
            s_duty = (uint16_t)(s_duty + VF_DUTY_SLEW);
        }
    }
    else
    {
        s_duty = (uint16_t)want_duty;
    }

    /* 周期始终由实际占空比反算: 限流冻结占空比时频率一起停, 保证压频一致 */
    want_period = vf_period_for_duty(s_duty);

    /* 步距限速: 加速 1/48, 减速 1/160 + 绝对限幅 */
    if (s_vf_period_us > want_period)          /* 加速: 步距减小 */
    {
        uint32_t d = ((uint32_t)s_vf_period_us - want_period + 47U) / 48U;

        if (d < 1U) { d = 1U; }
        s_vf_period_us = (uint16_t)(s_vf_period_us - d);
    }
    else if (s_vf_period_us < want_period)     /* 减速: 步距增大 */
    {
        uint32_t d = (want_period - (uint32_t)s_vf_period_us + 159U) / 160U;

        if (d > 20U) { d = 20U; }
        if (d < 1U) { d = 1U; }
        s_vf_period_us = (uint16_t)((uint32_t)s_vf_period_us + d);
    }

    s_period_us = s_vf_period_us;
    s_target_rpm = (uint16_t)(1428571UL / (uint32_t)s_vf_period_us);
    BSP_PWM_SetDuty(s_duty);
}

/** 闭环失步: 退回开环, 从当前步距×3 重新强拖 */
static void enter_ramp_recover(void)
{
    uint32_t p = (uint32_t)s_period_us * 3U;

    if (p < s_profiles[s_profile].ramp_end_us) { p = s_profiles[s_profile].ramp_end_us; }
    if (p > RAMP_PERIOD_START_US) { p = RAMP_PERIOD_START_US; }

    s_state = SIXSTEP_RAMP;
    s_ramp_period_us = (uint16_t)p;
    s_period_us = (uint16_t)p;
    s_ramp_ms = 0U;
    s_miss = 0U;
    s_zc_streak = 0U;
    s_integ = 0;
    s_zc_found = 0U;
    s_zc_armed = 0U;
    s_zc_cnt = 0U;

    commute_next();
    schedule_commute(s_ramp_period_us);
}

/** 速度环: 目标转速 -> 占空比 (前馈 + PI + 限斜率) */
static void speed_loop(void)
{
#if (SPEED_LOOP_ENABLE != 0U)
    int32_t err = (int32_t)s_target_rpm - (int32_t)s_speed_rpm;
    int32_t duty;

    s_integ += err;
    if (s_integ > INTEG_LIMIT)  { s_integ = INTEG_LIMIT; }
    if (s_integ < -INTEG_LIMIT) { s_integ = -INTEG_LIMIT; }

    duty  = ((int32_t)s_target_rpm * (int32_t)BSP_PWM_DUTY_MAX)
            / (int32_t)s_profiles[s_profile].rpm_ff;
    duty += err / 2;
    duty += s_integ / 64;

    if (duty < (int32_t)BSP_PWM_DUTY_MIN) { duty = (int32_t)BSP_PWM_DUTY_MIN; }
    if (duty > (int32_t)BSP_PWM_DUTY_MAX) { duty = (int32_t)BSP_PWM_DUTY_MAX; }

    if (duty > ((int32_t)s_duty + DUTY_SLEW_PER_CYCLE))
    {
        duty = (int32_t)s_duty + DUTY_SLEW_PER_CYCLE;
    }
    if (duty < ((int32_t)s_duty - DUTY_SLEW_PER_CYCLE))
    {
        duty = (int32_t)s_duty - DUTY_SLEW_PER_CYCLE;
    }

    s_duty = (uint16_t)duty;
    BSP_PWM_SetDuty(s_duty);
#else
    uint32_t duty = ((uint32_t)s_target_rpm * BSP_PWM_DUTY_MAX)
                    / (uint32_t)s_profiles[s_profile].rpm_max;

    if (duty < BSP_PWM_DUTY_MIN) { duty = BSP_PWM_DUTY_MIN; }
    if (duty > BSP_PWM_DUTY_MAX) { duty = BSP_PWM_DUTY_MAX; }
    s_duty = (uint16_t)duty;
    BSP_PWM_SetDuty(s_duty);
#endif
}

/* ================= 对外接口 ================= */
void BSP_SIXSTEP_Init(void)
{
    BTIM_TimeBaseInitTypeDef tb = {0};

    __SYSCTRL_BTIM123_CLK_ENABLE();

    /* BTIM1: 1 ms 任务节拍 (96MHz/96 = 1MHz, 1000 计数) */
    tb.BTIM_Mode      = BTIM_MODE_TIMER;
    tb.BTIM_CountMode = BTIM_COUNT_MODE_REPETITIVE;
    tb.BTIM_Prescaler = 95U;
    tb.BTIM_Period    = 999U;
    BTIM_TimeBaseInit(CW_BTIM1, &tb);
    BTIM_ITConfig(CW_BTIM1, BTIM_IT_UPDATE, ENABLE);
    BTIM_Cmd(CW_BTIM1, ENABLE);
    NVIC_SetPriority(BTIM1_IRQn, 2U);
    NVIC_EnableIRQ(BTIM1_IRQn);

    /* BTIM2: 1 MHz 自由计数 (96MHz/96), 用于量步距 */
    tb.BTIM_Period    = 0xFFFFU;
    BTIM_TimeBaseInit(CW_BTIM2, &tb);
    BTIM_AutoReloadPreloadConfig(CW_BTIM2, DISABLE);
    BTIM_Cmd(CW_BTIM2, ENABLE);

    /* BTIM3: 1 MHz 单次延时 */
    BTIM_TimeBaseInit(CW_BTIM3, &tb);
    BTIM_AutoReloadPreloadConfig(CW_BTIM3, DISABLE);
    BTIM_ITConfig(CW_BTIM3, BTIM_IT_UPDATE, ENABLE);
    BTIM_Cmd(CW_BTIM3, DISABLE);
    NVIC_SetPriority(BTIM3_HALLTIM_IRQn, 0U);
    NVIC_EnableIRQ(BTIM3_HALLTIM_IRQn);

    s_state = SIXSTEP_IDLE;
    s_fault = FAULT_NONE;
    s_step = 0U;
    s_dir = 1;
    s_duty = BSP_PWM_DUTY_MIN;
    BSP_PWM_SetDuty(s_duty);
}

void BSP_SIXSTEP_Start(void)
{
    if ((s_state != SIXSTEP_IDLE) && (s_state != SIXSTEP_FAULT) &&
        (s_state != SIXSTEP_BENCH) && (s_state != SIXSTEP_OPEN))
    {
        return;
    }

    s_fault = FAULT_NONE;
    s_miss = 0U;
    s_relock = 0U;
    s_zc_total = 0U;
    s_zc_streak = 0U;
    s_zc_found = 0U;
    s_zc_armed = 0U;
    s_zc_cnt = 0U;
    s_integ = 0;
    s_period_us = 0U;
    s_speed_rpm = 0U;
    s_speed_div = 0U;
    s_oc_cnt = 0U;
    s_oc_blank_ms = OC_BLANK_MS;
    s_step = ALIGN_STEP_INDEX;
    s_duty = ALIGN_DUTY;
    s_cal_idx = 0U;
    s_cal_ms = 0U;
    s_pot_arm = 1U;
    s_pospeak = 0U;
    s_bemf_amp = 0U;
    s_stall_ms = 0U;
    s_ev_seen = 0U;
    s_pll_cnt = 0U;
    s_prev_diff = 0;

    BSP_PWM_SetDuty(s_duty);
    BSP_PWM_FloatAll();
    BSP_PWM_ApplyStep(s_step);
    BSP_PWM_EnableOutputs();

    BTIM_SetCounter(CW_BTIM2, 0U);
    s_align_ms = 0U;
    s_state = SIXSTEP_ALIGN;
}

void BSP_SIXSTEP_Stop(void)
{
    pwm_all_off();
    s_state = SIXSTEP_IDLE;
    s_fault = FAULT_NONE;
    s_pot_arm = 0U;                 /* 手动停机后, 需电位器回低位再自动启动 */
    s_speed_rpm = 0U;
    s_period_us = 0U;
    s_zc_streak = 0U;
    s_miss = 0U;
    s_duty = BSP_PWM_DUTY_MIN;
}

/** 调试: 固定 step0 (A+ B-) 持续输出, 用于验证 PWM/驱动/采样链路 */
void BSP_SIXSTEP_BenchHold(uint16_t duty)
{
    if (duty < BSP_PWM_DUTY_MIN) { duty = BSP_PWM_DUTY_MIN; }
    if (duty > BSP_PWM_DUTY_MAX) { duty = BSP_PWM_DUTY_MAX; }

    pwm_all_off();
    s_fault = FAULT_NONE;
    s_step = ALIGN_STEP_INDEX;
    s_duty = duty;
    BSP_PWM_SetDuty(s_duty);
    BSP_PWM_ApplyStep(s_step);
    BSP_PWM_EnableOutputs();
    s_oc_blank_ms = 100U;
    s_oc_cnt = 0U;
    s_state = SIXSTEP_BENCH;
}

void BSP_SIXSTEP_BenchStep(uint8_t step)
{
    if (s_state != SIXSTEP_BENCH)
    {
        return;
    }
    s_step = (uint8_t)(step % 6U);
    BSP_PWM_ApplyStep(s_step);
}

/** 调试: 固定周期开环旋转, 用于观察电机跟随与过零检测 */
void BSP_SIXSTEP_OpenLoop(uint16_t period_us, uint16_t duty)
{
    if (period_us < 1000U) { period_us = 1000U; }
    if (duty < BSP_PWM_DUTY_MIN) { duty = BSP_PWM_DUTY_MIN; }
    if (duty > BSP_PWM_DUTY_MAX) { duty = BSP_PWM_DUTY_MAX; }

    pwm_all_off();
    s_fault = FAULT_NONE;
    s_open_period_us = period_us;
    s_period_us = period_us;
    s_duty = duty;
    BSP_PWM_SetDuty(s_duty);
    s_zc_found = 0U;
    s_zc_armed = 0U;
    s_zc_cnt = 0U;
    s_zc_total = 0U;
    s_zc_fall = 0U;
    s_zc_rise = 0U;
    s_miss = 0U;
    s_oc_blank_ms = 100U;
    s_oc_cnt = 0U;
    s_step = ALIGN_STEP_INDEX;
    BSP_PWM_ApplyStep(s_step);
    BTIM_SetCounter(CW_BTIM2, 0U);
    BSP_PWM_EnableOutputs();
    s_state = SIXSTEP_OPEN;
    schedule_commute(s_open_period_us);
}

void BSP_SIXSTEP_SetDirection(int8_t dir)
{
    if ((s_state == SIXSTEP_ALIGN) || (s_state == SIXSTEP_RAMP) || (s_state == SIXSTEP_RUN))
    {
        BSP_SIXSTEP_Stop();
    }
    s_dir = (dir >= 0) ? 1 : -1;
}

int8_t BSP_SIXSTEP_GetDirection(void)
{
    return s_dir;
}

void BSP_SIXSTEP_ToggleDirection(void)
{
    BSP_SIXSTEP_SetDirection((s_dir > 0) ? -1 : 1);
}

void BSP_SIXSTEP_SetManualTarget(uint16_t rpm)
{
    if (rpm == 0xFFFFU)
    {
        s_manual = 0U;
    }
    else
    {
        s_manual = 1U;
        s_manual_rpm = (rpm > s_profiles[s_profile].rpm_max)
                       ? s_profiles[s_profile].rpm_max : rpm;
    }
}

void BSP_SIXSTEP_SetMotorProfile(uint8_t profile)
{
    if (profile > BSP_SIXSTEP_PROFILE_4006)
    {
        profile = BSP_SIXSTEP_PROFILE_4006;
    }

    BSP_SIXSTEP_Stop();
    s_profile = profile;

    if ((s_manual != 0U) && (s_manual_rpm > s_profiles[s_profile].rpm_max))
    {
        s_manual_rpm = s_profiles[s_profile].rpm_max;
    }
}

uint8_t BSP_SIXSTEP_GetMotorProfile(void)
{
    return s_profile;
}

uint8_t BSP_SIXSTEP_GetManualMode(void)
{
    return s_manual;
}

void BSP_SIXSTEP_OnBemfSample(void)
{
    uint8_t  step;
    uint8_t  falling;
    uint16_t blank;

    /* 调试跟踪: 每 2 个采样记一点 (记录虚拟中性点差值, 偏置 2048 便于观察正负) */
    if (s_trace_count < s_trace_total)
    {
        if (s_trace_skip == 0U)
        {
            uint8_t  fl = s_float_phase[s_step];
            int16_t  vx = (int16_t)g_bemf_raw[(uint8_t)((fl + 1U) % 3U)];
            int16_t  vy = (int16_t)g_bemf_raw[(uint8_t)((fl + 2U) % 3U)];
            int16_t  d  = (int16_t)g_bemf_raw[fl] - (int16_t)((vx + vy) / 2);
            uint16_t v  = (uint16_t)(d + 2048);
            uint16_t i  = (uint16_t)(s_trace_count * 3U);

            s_trace_buf[i]      = (uint8_t)(v & 0xFFU);
            s_trace_buf[i + 1U] = (uint8_t)(v >> 8);
            s_trace_buf[i + 2U] = s_step;
            s_trace_count++;
            s_trace_skip = 1U;
        }
        else
        {
            s_trace_skip = 0U;
        }
    }

    if ((s_state != SIXSTEP_RAMP) && (s_state != SIXSTEP_RUN) &&
        (s_state != SIXSTEP_OPEN) && (s_state != SIXSTEP_VF))
    {
        return;
    }
    if (s_zc_found != 0U)
    {
        return;
    }

    step = s_step;

    /* 堵转保护: 超过 1.25 个步距仍无过零 -> 强换相, 连续多次则报故障 */
    if (s_state == SIXSTEP_RUN)
    {
        uint32_t limit = ((uint32_t)s_period_us * RUN_TIMEOUT_NUM) / RUN_TIMEOUT_DEN;

        if (limit < 1000U) { limit = 1000U; }
        if ((uint32_t)now_us() > limit)
        {
            s_miss++;
            if (s_miss >= RUN_MISS_LIMIT)
            {
                if (s_relock >= RELOCK_LIMIT)
                {
                    enter_fault(FAULT_STALL);
                }
                else
                {
                    s_relock++;
                    enter_ramp_recover();
                }
                return;
            }
            commute_next();
            return;
        }
    }

    /* 换相后盲区, 躲开续流/振铃 */
    if ((s_state == SIXSTEP_RUN) || (s_state == SIXSTEP_OPEN))
    {
        blank = (uint16_t)(s_period_us / ZC_BLANK_DIV);
    }
    else
    {
        blank = (uint16_t)(s_ramp_period_us / ZC_BLANK_DIV);
    }
    if (blank < ZC_BLANK_MIN_US) { blank = ZC_BLANK_MIN_US; }
    if (now_us() < blank)
    {
        return;
    }

    /* --- BEMF 相位锁定事件: 浮空相正半波下降到半峰值 (每转 3 次) --- */
#if (CFG_PLL != 0U)
    {
        uint8_t fl = s_float_phase[step];
        int16_t vf = (int16_t)g_bemf_raw[fl];
        int16_t vx = (int16_t)g_bemf_raw[(uint8_t)((fl + 1U) % 3U)];
        int16_t vy = (int16_t)g_bemf_raw[(uint8_t)((fl + 2U) % 3U)];
        int16_t d16 = (int16_t)(vf - ((vx + vy) / 2));
        int16_t half;

        if (d16 > (int16_t)s_pospeak)
        {
            s_pospeak = (uint16_t)d16;
        }
        else
        {
            s_pospeak = (uint16_t)(s_pospeak - (s_pospeak >> 10));
        }

        half = (int16_t)(s_pospeak / 2U);

        if (d16 > half)
        {
            if (s_above_cnt < 0xFFFFU) { s_above_cnt++; }
        }
        else
        {
            /* 事件前需在半峰值之上持续 >=4 个采样, 滤掉换相尖峰 */
            if ((s_ev_seen == 0U) && ((int16_t)s_pospeak > 60) && (s_above_cnt >= 4U))
            {
                s_ev_seen = 1U;
                s_pll_cnt++;
                pll_trim(now_us());
            }
            s_above_cnt = 0U;
        }
        s_prev_diff = d16;
    }
#endif

    falling = (s_dir > 0) ? s_zc_fall_fwd[step] : (uint8_t)(s_zc_fall_fwd[step] ^ 1U);

    /* 虚拟中性点法: diff = vFloat - (vDrv1 + vDrv2)/2, 过零即 diff 过 0.
       同一时刻三相采样, 与占空比/分压误差/单端削波无关. */
    {
        uint8_t fl = s_float_phase[step];
        int16_t vf = (int16_t)g_bemf_raw[fl];
        int16_t vx = (int16_t)g_bemf_raw[(uint8_t)((fl + 1U) % 3U)];
        int16_t vy = (int16_t)g_bemf_raw[(uint8_t)((fl + 2U) % 3U)];
        int16_t d16 = (int16_t)(vf - ((vx + vy) / 2));
        int16_t m16 = (int16_t)ZC_ARM_CNT;
        int16_t c16 = (int16_t)ZC_CONF_CNT;
        int16_t a16 = (int16_t)((d16 < 0) ? -d16 : d16);

        /* 反电势幅值跟踪 (快速跟随, 用于失速检测) */
        if (a16 > (int16_t)s_bemf_amp)
        {
            s_bemf_amp = (uint16_t)a16;
        }
        else
        {
            s_bemf_amp = (uint16_t)(s_bemf_amp - (s_bemf_amp >> 6));
        }

        if (s_zc_armed == 0U)
        {
            /* 先看到过零前另一侧 (带余量) 才允许判过零 */
            if (falling != 0U)
            {
                if (d16 > m16)
                {
                    s_zc_armed = 1U;
                }
            }
            else
            {
                if (d16 < (int16_t)(-m16))
                {
                    s_zc_armed = 1U;
                }
            }
            return;
        }

        if (falling != 0U)
        {
            if (d16 < (int16_t)(-c16))
            {
                s_zc_cnt++;
            }
            else
            {
                s_zc_cnt = 0U;
            }
        }
        else
        {
            if (d16 > c16)
            {
                s_zc_cnt++;
            }
            else
            {
                s_zc_cnt = 0U;
            }
        }
    }

    if (s_zc_cnt < ZC_CONFIRM_CNT)
    {
        return;
    }
    s_zc_cnt = 0U;

    /* ADC 中断期间可能已被换相, 步号不符则丢弃 */
    if (step != s_step)
    {
        return;
    }

    s_zc_found = 1U;
    s_miss = 0U;
    s_zc_total++;
    if (falling != 0U) { s_zc_fall++; } else { s_zc_rise++; }

    if (s_state == SIXSTEP_RAMP)
    {
#if (CFG_CLOSED_LOOP != 0U)
        if (s_zc_streak < 0xFFFFU) { s_zc_streak++; }

        /* 开环阶段就用过零+30°对齐换相, 让转子尽早跟上 */
        schedule_commute((uint16_t)(s_ramp_period_us / 2U));

        if (s_zc_streak >= LOCK_ZC_STREAK)
        {
            /* 达到最低锁定转速才允许闭环, 防止低速噪声误锁 */
            if (s_ramp_period_us <= s_profiles[s_profile].lock_period_us)
            {
                s_state = SIXSTEP_RUN;
                s_integ = 0;
                s_speed_div = 0U;
                s_relock = 0U;
            }
            else
            {
                s_zc_streak = 0U;
            }
        }
#endif
    }
    else if (s_state == SIXSTEP_RUN)
    {
        schedule_commute((uint16_t)(s_period_us / 2U));
    }
    /* SIXSTEP_OPEN / SIXSTEP_VF: 只统计, 不闭环 */
}

void BSP_SIXSTEP_OnDelayTimerIRQ(void)
{
    if (BTIM_GetITStatus(CW_BTIM3, BTIM_IT_UPDATE) == RESET)
    {
        return;
    }
    BTIM_ClearITPendingBit(CW_BTIM3, BTIM_IT_UPDATE);
    BTIM_Cmd(CW_BTIM3, DISABLE);

    if (s_state == SIXSTEP_RAMP)
    {
        uint32_t p = s_ramp_period_us;

        if (s_zc_found == 0U)
        {
            s_zc_streak = 0U;   /* 本步没抓到过零, 连续计数清零 */
        }

        p -= p / RAMP_PERIOD_DECAY_DIV;
        if (p < s_profiles[s_profile].ramp_end_us) { p = s_profiles[s_profile].ramp_end_us; }
        s_ramp_period_us = (uint16_t)p;
        s_period_us = (uint16_t)p;

        /* 斜坡按 V/f 关系随频率抬压 (受软限流约束) */
        {
            uint16_t want = vf_duty_for_period(s_ramp_period_us);

            if (s_duty < want)
            {
                if (cur_high() == 0U)
                {
                    uint32_t d = (uint32_t)s_duty + 8U;

                    if (d > want) { d = want; }
                    s_duty = (uint16_t)d;
                    BSP_PWM_SetDuty(s_duty);
                }
            }
            else if (s_duty > want)
            {
                uint32_t d = (uint32_t)s_duty - 16U;

                if (d < want) { d = want; }
                s_duty = (uint16_t)d;
                BSP_PWM_SetDuty(s_duty);
            }
        }

        if (s_ramp_period_us <= s_profiles[s_profile].ramp_end_us)
        {
            enter_vf();         /* 斜坡到底: 交给 V/f 开环运行 */
        }
        else
        {
            commute_next();
            schedule_commute(s_ramp_period_us);
        }
    }
    else if (s_state == SIXSTEP_RUN)
    {
        commute_next();
    }
    else if (s_state == SIXSTEP_OPEN)
    {
        commute_next();
        schedule_commute(s_open_period_us);
    }
    else if (s_state == SIXSTEP_VF)
    {
        commute_next();
        schedule_commute(s_vf_period_us);
    }
}

void BSP_SIXSTEP_Tick1ms(void)
{
    uint32_t vbus_mv;
    int32_t  isout_delta;

    BSP_ADC_Update();

    /* --- 采样与滤波 --- */
    {
        uint16_t raw = BSP_ADC_GetVbusRaw();

        if (s_vbus_raw_f == 0U)
        {
            s_vbus_raw_f = raw;
        }
        else
        {
            s_vbus_raw_f = (uint16_t)(((uint32_t)s_vbus_raw_f * 3U + raw) / 4U);
        }
    }
    s_isout_raw = BSP_ADC_GetIsoutRaw();
    s_pot_raw   = BSP_ADC_GetPotRaw();
    s_threshold = (uint16_t)(s_vbus_raw_f / 2U);

    /* 停机时校准电流零偏 (OPA 偏置 2.5V) */
    if ((s_state == SIXSTEP_IDLE) || (s_state == SIXSTEP_FAULT))
    {
        s_isout_zero = (uint16_t)(((uint32_t)s_isout_zero * 15U + s_isout_raw) / 16U);
    }

    /* --- 目标转速 --- */
    if (s_manual != 0U)
    {
        s_target_rpm = s_manual_rpm;
    }
    else if (s_pot_raw <= POT_START_RAW)
    {
        s_target_rpm = 0U;
    }
    else
    {
        s_target_rpm = (uint16_t)(50U
                       + ((uint32_t)(s_pot_raw - POT_START_RAW)
                          * (s_profiles[s_profile].rpm_max - 50U))
                         / (4095U - POT_START_RAW));
    }

    /* --- 电位器阈值启停 (仅电位器模式): >200 启动, <200 停机 --- */
    if (s_manual == 0U)
    {
        if (s_pot_raw < POT_START_RAW)
        {
            s_pot_arm = 1U;         /* 回低位, 重新允许启动 */
            if ((s_state == SIXSTEP_ALIGN) || (s_state == SIXSTEP_RAMP) ||
                (s_state == SIXSTEP_RUN)   || (s_state == SIXSTEP_VF))
            {
                BSP_SIXSTEP_Stop();
                s_pot_arm = 1U;
            }
        }
        else if ((s_pot_arm != 0U) &&
                 ((s_state == SIXSTEP_IDLE) || (s_state == SIXSTEP_FAULT)))
        {
            BSP_SIXSTEP_Start();
        }
    }

    /* --- 保护 --- */
    if (s_oc_blank_ms > 0U) { s_oc_blank_ms--; }

    vbus_mv = ((uint32_t)s_vbus_raw_f * 5000U / 4095U)
              * VBUS_MV_PER_COUNT_NUM / VBUS_MV_PER_COUNT_DEN;

    if (vbus_mv > VBUS_MAX_MV)
    {
        enter_fault(FAULT_VBUS_OV);
        return;
    }
    if ((vbus_mv < VBUS_MIN_MV) &&
        (s_state != SIXSTEP_IDLE) && (s_state != SIXSTEP_FAULT))
    {
        enter_fault(FAULT_VBUS_UV);
        return;
    }

    if ((s_state == SIXSTEP_ALIGN) || (s_state == SIXSTEP_RAMP) ||
        (s_state == SIXSTEP_RUN)   || (s_state == SIXSTEP_BENCH) ||
        (s_state == SIXSTEP_OPEN)  || (s_state == SIXSTEP_VF))
    {
        isout_delta = (int32_t)s_isout_raw - (int32_t)s_isout_zero;
        if (isout_delta < 0) { isout_delta = -isout_delta; }

        if ((s_oc_blank_ms == 0U) && (isout_delta > (int32_t)OC_RAW_THRESHOLD))
        {
            s_oc_cnt++;
            if (s_oc_cnt >= OC_CONFIRM_CNT)
            {
                enter_fault(FAULT_OC);
                return;
            }
        }
        else
        {
            s_oc_cnt = 0U;
        }
    }

    /* --- 状态机 --- */
    switch (s_state)
    {
    case SIXSTEP_ALIGN:
        if (s_target_rpm < TARGET_RPM_MIN)      /* 油门为零立即停 */
        {
            BSP_SIXSTEP_Stop();
            break;
        }

        /* 先标定三相削底电平 (每相 20ms), 得到低阈值过零参考 */
        if (s_cal_idx < 3U)
        {
            if (s_cal_ms == 0U)
            {
                BSP_PWM_ApplyStep(s_cal_idx);
            }
            if (s_cal_ms < 20U)
            {
                s_cal_ms++;
            }
            if (s_cal_ms >= 20U)
            {
                uint8_t ph = s_float_phase[s_cal_idx];

                s_zc_zero[ph] = (uint16_t)(g_bemf_raw[ph] + 4U);
                s_cal_idx++;
                s_cal_ms = 0U;
                if (s_cal_idx >= 3U)
                {
                    BSP_PWM_ApplyStep(ALIGN_STEP_INDEX);
                    s_align_ms = 0U;
                }
            }
            break;
        }

        s_align_ms++;
        if (s_align_ms >= ALIGN_TIME_MS)
        {
            start_ramp();
        }
        break;

    case SIXSTEP_RAMP:
        if (s_target_rpm < TARGET_RPM_MIN)
        {
            BSP_SIXSTEP_Stop();
            break;
        }
        s_ramp_ms++;
        if (s_ramp_ms >= RAMP_TIMEOUT_MS)
        {
            enter_fault(FAULT_START_TIMEOUT);
        }
        break;

    case SIXSTEP_RUN:
        if (s_period_us > 0U)
        {
            s_speed_rpm = (uint16_t)(10000000UL /
                          ((uint32_t)s_period_us * MOTOR_POLE_PAIRS));
        }
        if (s_target_rpm < TARGET_RPM_MIN)
        {
            BSP_SIXSTEP_Stop();     /* 油门归零 */
            break;
        }
        s_speed_div++;
        if (s_speed_div >= SPEED_LOOP_PERIOD_MS)
        {
            s_speed_div = 0U;
            speed_loop();
        }
        break;

    case SIXSTEP_VF:
        if (s_vf_period_us > 0U)
        {
            s_speed_rpm = (uint16_t)(10000000UL /
                          ((uint32_t)s_vf_period_us * MOTOR_POLE_PAIRS));
        }
        if (s_target_rpm < TARGET_RPM_MIN)
        {
            BSP_SIXSTEP_Stop();     /* 油门归零 */
            break;
        }

        /* 失速检测: 指令转速较高但反电势幅值消失 -> 停机 */
        if ((s_speed_rpm > STALL_RPM_MIN) && (s_bemf_amp < STALL_AMP_CNT))
        {
            if (s_stall_ms < 2000U) { s_stall_ms++; }
            if (s_stall_ms >= STALL_MS)
            {
                enter_fault(FAULT_STALL);
                break;
            }
        }
        else
        {
            s_stall_ms = 0U;
        }

        s_speed_div++;
        if (s_speed_div >= SPEED_LOOP_PERIOD_MS)
        {
            s_speed_div = 0U;
            vf_update();
        }
        break;

    default:
        break;
    }
}

/* ================= 查询接口 ================= */
SixStepState_t BSP_SIXSTEP_GetState(void)
{
    return s_state;
}

uint8_t BSP_SIXSTEP_GetFault(void)
{
    return s_fault;
}

uint8_t BSP_SIXSTEP_GetStep(void)
{
    return s_step;
}

uint16_t BSP_SIXSTEP_GetDuty(void)
{
    return s_duty;
}

uint16_t BSP_SIXSTEP_GetPeriodUs(void)
{
    return s_period_us;
}

uint16_t BSP_SIXSTEP_GetSpeedRpm(void)
{
    return s_speed_rpm;
}

uint16_t BSP_SIXSTEP_GetTargetRpm(void)
{
    return s_target_rpm;
}

uint16_t BSP_SIXSTEP_GetVbusMv(void)
{
    return (uint16_t)(((uint32_t)s_vbus_raw_f * 5000U / 4095U)
                      * VBUS_MV_PER_COUNT_NUM / VBUS_MV_PER_COUNT_DEN);
}

uint16_t BSP_SIXSTEP_GetIsoutZero(void)
{
    return s_isout_zero;
}

uint16_t BSP_SIXSTEP_GetIsoutMa(void)
{
    int32_t delta = (int32_t)s_isout_raw - (int32_t)s_isout_zero;

    /* 1 LSB = 1.221 mV; 10mΩ * 10 倍 = 0.1 V/A -> 12.21 mA/LSB */
    if (delta < 0) { delta = -delta; }
    delta = (delta * 1221) / 100;
    if (delta > 65535) { delta = 65535; }
    return (uint16_t)delta;
}

uint32_t BSP_SIXSTEP_GetZcCount(void)
{
    return s_zc_total;
}

uint32_t BSP_SIXSTEP_GetZcFallCount(void)
{
    return s_zc_fall;
}

uint32_t BSP_SIXSTEP_GetZcRiseCount(void)
{
    return s_zc_rise;
}

uint16_t BSP_SIXSTEP_GetMissCount(void)
{
    return s_miss;
}

uint32_t BSP_SIXSTEP_GetPllCount(void)
{
    return s_pll_cnt;
}

void BSP_SIXSTEP_TraceStart(uint16_t n)
{
    if (n > TRACE_MAX) { n = TRACE_MAX; }
    s_trace_total = n;
    s_trace_count = 0U;
    s_trace_skip = 0U;
}

uint16_t BSP_SIXSTEP_TraceCount(void)
{
    return s_trace_count;
}

uint8_t BSP_SIXSTEP_TraceGet(uint16_t i, uint16_t *sample)
{
    if ((sample == 0) || (i >= s_trace_count))
    {
        return 0xFFU;
    }
    *sample = (uint16_t)s_trace_buf[i * 3U]
              | ((uint16_t)s_trace_buf[i * 3U + 1U] << 8);
    return s_trace_buf[i * 3U + 2U];
}
