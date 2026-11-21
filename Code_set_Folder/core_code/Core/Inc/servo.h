#ifndef __SERVO_H
#define __SERVO_H
#include "main.h"
#include "corexy.h"
#include "ws2812_spi.h"
#include "cmsis_os2.h"
#include "uart1_proto.h"
#include "turntable.h"
#include "Servo.h"
#include "WaterPump.h"
#include "Grab.h"
extern volatile uint8_t Servo1_FallFlag;
extern volatile uint8_t Servo1_RiseFlag;
extern uint8_t Servo2_OpenFlag;
extern uint8_t Servo2_CloseFlag;
void Servo_Init(void);
void Servo_SetAngle(float Angle, uint32_t CHANNEL);

/**
 * @brief 按脉宽（微秒）直接驱动指定 TIM4 通道
 *
 * @details 用于抓取流程精确定位舵机上升/下降位置；TIM4 为 1us 计数，CCR 即脉宽。
 *
 * @param pulse_us 高电平脉宽（微秒），内部钳位到 500~2500us
 * @param channel  TIM4 通道（TIM_CHANNEL_3 / TIM_CHANNEL_4）
 * @retval 无
 */
void Servo_SetPulseUs(uint32_t pulse_us, uint32_t channel);

/**
 * @brief 舵机2 执行「打开」动作：反转 10 个齿距（16 齿齿轮，约 225°）后停止
 *
 * @details 连续旋转舵机无位置反馈，按整圈时间 × 10/16 估算运行时长；
 *          内部使用 osDelay，只能在任务上下文调用，需与 Servo2_Return() 配对。
 *
 * @param 无
 * @retval 无
 */
void Servo2_Open(void);

/**
 * @brief 舵机2 执行「回位」动作：正转 10 个齿距，回到打开之前的位置
 *
 * @details 使用与 Servo2_Open() 相同的运行时长，转回后输出停止脉宽；
 *          内部使用 osDelay，只能在任务上下文调用。
 *
 * @param 无
 * @retval 无
 */
void Servo2_Return(void);

/**
 * @brief 舵机1 按齿数转动：正数增大脉宽，负数减小脉宽
 *
 * @details 舵机1 为 180° 位置舵机，1 齿 ≈ 167us 脉宽；以内部记录的当前脉宽为基准
 *          做增量，输出后钳位到 500~2500us。内部使用 osDelay，只能在任务上下文调用。
 *
 * @param teeth 要转动的齿数，正数增大脉宽（实机为上升），负数下降；范围约 -5~+5
 * @retval 无
 */
void Servo1_MoveTeeth(int8_t teeth);

void ServoTask(void *argument);
#endif
