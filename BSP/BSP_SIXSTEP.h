/**
 * @file    BSP_SIXSTEP.h
 * @brief   无感 BLDC 六步换相控制 (反电势过零)
 *
 *   IDLE --start--> ALIGN (对齐) --300ms--> RAMP (开环强拖, 同时找过零)
 *        --连续 N 次过零--> RUN (过零闭环, 30° 延时换相 + 速度环)
 *   VBUS 过压/欠压, 母线过流, 启动超时, 堵转 -> FAULT
 */
#ifndef BSP_SIXSTEP_H
#define BSP_SIXSTEP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SIXSTEP_IDLE = 0,
    SIXSTEP_ALIGN,
    SIXSTEP_RAMP,
    SIXSTEP_RUN,           /* 过零闭环 (CFG_CLOSED_LOOP=1 时使用) */
    SIXSTEP_FAULT,
    SIXSTEP_BENCH,         /* 调试: 固定向量保持, 用于验证功率级 */
    SIXSTEP_OPEN,          /* 调试: 固定周期开环旋转 */
    SIXSTEP_VF             /* 电位器调速: 开环 V/f 运行 */
} SixStepState_t;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_VBUS_OV,
    FAULT_VBUS_UV,
    FAULT_OC,
    FAULT_START_TIMEOUT,
    FAULT_STALL
} SixStepFault_t;

/** 电机配置档案: 0=2208 (8Ω 云台), 1=4006 (0.17Ω 航模) */
#define BSP_SIXSTEP_PROFILE_2208    0U
#define BSP_SIXSTEP_PROFILE_4006    1U

void BSP_SIXSTEP_SetMotorProfile(uint8_t profile);
uint8_t BSP_SIXSTEP_GetMotorProfile(void);

void BSP_SIXSTEP_Init(void);
void BSP_SIXSTEP_Start(void);
void BSP_SIXSTEP_Stop(void);

/** 调试: 固定 step0 (A+ B-) 给定占空比持续输出, stop 退出 */
void BSP_SIXSTEP_BenchHold(uint16_t duty);
/** 调试: 在 BENCH 状态下切换固定向量 (0..5) */
void BSP_SIXSTEP_BenchStep(uint8_t step);
/** 调试: 固定周期开环旋转 (period_us 每步, duty 占空比), stop 退出 */
void BSP_SIXSTEP_OpenLoop(uint16_t period_us, uint16_t duty);
/** 调试: 抓取最近 N 个浮空相采样 (每 2 次取 1 点) */
void BSP_SIXSTEP_TraceStart(uint16_t n);
uint16_t BSP_SIXSTEP_TraceCount(void);
uint8_t BSP_SIXSTEP_TraceGet(uint16_t i, uint16_t *sample);

void BSP_SIXSTEP_SetDirection(int8_t dir);
int8_t BSP_SIXSTEP_GetDirection(void);
void BSP_SIXSTEP_ToggleDirection(void);

/** 手动目标转速 (rpm); 传 0xFFFF 恢复用电位器 */
void BSP_SIXSTEP_SetManualTarget(uint16_t rpm);
uint8_t BSP_SIXSTEP_GetManualMode(void);

/** 1 ms 周期任务, 由 BTIM1 中断调用 */
void BSP_SIXSTEP_Tick1ms(void);
/** ADC1 EOS (BEMF 采样完成) 中断内调用 */
void BSP_SIXSTEP_OnBemfSample(void);
/** BTIM3 更新中断内调用 */
void BSP_SIXSTEP_OnDelayTimerIRQ(void);

SixStepState_t BSP_SIXSTEP_GetState(void);
uint8_t  BSP_SIXSTEP_GetFault(void);
uint8_t  BSP_SIXSTEP_GetStep(void);
uint16_t BSP_SIXSTEP_GetDuty(void);
uint16_t BSP_SIXSTEP_GetPeriodUs(void);
uint16_t BSP_SIXSTEP_GetSpeedRpm(void);
uint16_t BSP_SIXSTEP_GetTargetRpm(void);
uint16_t BSP_SIXSTEP_GetVbusMv(void);
uint16_t BSP_SIXSTEP_GetIsoutZero(void);
uint16_t BSP_SIXSTEP_GetIsoutMa(void);
uint32_t BSP_SIXSTEP_GetZcCount(void);
uint32_t BSP_SIXSTEP_GetPllCount(void);
uint16_t BSP_SIXSTEP_GetMissCount(void);
uint32_t BSP_SIXSTEP_GetZcFallCount(void);
uint32_t BSP_SIXSTEP_GetZcRiseCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SIXSTEP_H */
