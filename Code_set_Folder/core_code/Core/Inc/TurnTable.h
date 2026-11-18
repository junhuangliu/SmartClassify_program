#ifndef TURN_TABLE_H
#define TURN_TABLE_H

#include "stepper_motor.h"
#include "main.h"

/**
 * @brief 初始化转台步进电机通信。
 * @details 将全局转台电机对象 stepper_motor_b2 绑定到 UART5，并启动电机应答
 *          的空闲中断 + DMA 接收。调用前需完成 UART5 及其 RX DMA 初始化。
 * @retval HAL 状态码；未配置 UART5 RX DMA 时返回 HAL_ERROR。
 */
HAL_StatusTypeDef TurnTable_Init(void);
void TurnTable_Set(uint8_t Table);

/**
 * @brief 转台任务入口，当前为占位实现，循环 osDelay 让出 CPU。
 * @details 与 ServoTask 同为 osPriorityBelowNormal，按时间片轮转共享 CPU；
 *          后续转台命令接入后在此任务上下文执行阻塞动作。
 * @param argument 未使用。
 * @retval 无（任务不返回）。
 */
void TurnTableTask(void *argument);
extern uint8_t Turn_Flag;
#endif
