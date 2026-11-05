/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pump.h
  * @brief          : 水泵控制模块（TIM4 CH1 运行 PWM / CH2 停止脉冲）
  ******************************************************************************
  * @attention
  *
  * 硬件约定：
  * - TIM4_CH1(PD12)：开泵时输出运行 PWM，关泵时为 0
  * - TIM4_CH2(PD13)：关泵时输出一个短脉冲，随后回 0；开泵时为 0
  *
  * 全部自定义逻辑均放置于 USER CODE 区域，符合项目约束。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PUMP_H__
#define __PUMP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported macro ------------------------------------------------------------*/
#define PUMP_RUN_CHANNEL    TIM_CHANNEL_1   /* PD12：运行 PWM */
#define PUMP_STOP_CHANNEL   TIM_CHANNEL_2   /* PD13：停止脉冲 */

#define PUMP_PWM_PSC        0U              /* TIM4 时钟 60MHz，PSC=0 */
#define PUMP_PWM_ARR        11999U          /* 60MHz/(11999+1)=5kHz */

#define PUMP_RUN_DUTY_DEFAULT   50U         /* ON 默认运行占空比（%） */
#define PUMP_MAX_DUTY           95U         /* 最大占空比，避免 100% 恒定高电平 */
#define PUMP_STOP_DUTY          20U         /* 停止脉冲占空比（%） */
#define PUMP_STOP_PULSE_MS      100U        /* 停止脉冲持续时间（ms） */

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 初始化水泵控制：启动 TIM4 CH1/CH2 PWM，默认关闭
 *
 * @details
 * - 设计意图：把 TIM4 的两路 PWM 固定为“运行 + 停止脉冲”的硬件动作
 * - 实现要点：设置 PWM 周期为 20kHz，两路 CCR 先清 0，再启动 CH1/CH2
 * - 注意事项：调用前必须已完成 MX_TIM4_Init()；本函数不使用 osDelay，
 *   可在 osKernelStart() 之前调用
 *
 * @param 无
 * @retval 无
 */
void Pump_Init(void);

/**
 * @brief 开泵：CH2 置 0，CH1 按当前设定占空比输出
 *
 * @details
 * - 设计意图：与“设置占空比”解耦，开关由独立命令控制
 * - 实现要点：先确保停止通道 CH2 为 0，再写 CH1 的 CCR
 * - 注意事项：占空比由 Pump_SetDuty() 预先保存，Pump_Init() 默认 50%
 *
 * @param 无
 * @retval 无
 */
void Pump_On(void);

/**
 * @brief 设置运行占空比（不改变开关状态）
 *
 * @details
 * - 实现要点：运行中立即更新 CH1 的 CCR；关闭时只保存，等下次开泵生效
 *
 * @param duty_percent 目标占空比 0~100（%）
 * @retval 无
 */
void Pump_SetDuty(uint8_t duty_percent);

/**
 * @brief 关泵：CH1 置 0，CH2 输出短脉冲后回 0
 *
 * @details
 * - 设计意图：按硬件要求用 CH2 的短脉冲完成停止动作
 * - 实现要点：CH1 CCR=0 -> CH2 CCR=停止占空比 -> osDelay -> CH2 CCR=0
 * - 注意事项：内部使用 osDelay，只能在 FreeRTOS 任务中调用，不可在 ISR 中调用
 *
 * @param 无
 * @retval 无
 */
void Pump_Off(void);

/**
 * @brief 查询水泵是否处于开启状态
 *
 * @param 无
 * @retval 1 开启，0 关闭
 */
uint8_t Pump_IsOn(void);

/**
 * @brief 查询最近一次设置的运行占空比
 *
 * @param 无
 * @retval 占空比 0~100（%）
 */
uint8_t Pump_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* __PUMP_H__ */
