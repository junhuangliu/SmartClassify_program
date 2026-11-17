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
#endif
