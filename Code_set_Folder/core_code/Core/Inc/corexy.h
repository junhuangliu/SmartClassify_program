#ifndef COREXY_H
#define COREXY_H

#include "stepper_motor.h"

#define COREXY_MOTOR_A1_UART_HANDLE (&huart2)
#define COREXY_MOTOR_A2_UART_HANDLE (&huart3)
#define COREXY_MOTOR_B1_UART_HANDLE (&huart5)
#define COREXY_MOTOR_B2_UART_HANDLE (&huart6)

#define COREXY_FULL_STEPS_PER_REV 200.0f
#define COREXY_MICROSTEPS 16.0f
#define COREXY_BELT_TOOTH_PITCH_MM 2.0f
#define COREXY_PULLEY_TEETH 20.0f
#define COREXY_PULSES_PER_MM \
  ((COREXY_FULL_STEPS_PER_REV * COREXY_MICROSTEPS) / \
   (COREXY_BELT_TOOTH_PITCH_MM * COREXY_PULLEY_TEETH))

#define COREXY_MOTOR_A1_INVERT_DIRECTION 0U
#define COREXY_MOTOR_A2_INVERT_DIRECTION 0U
#define COREXY_MOTOR_B1_INVERT_DIRECTION 0U
#define COREXY_MOTOR_B2_INVERT_DIRECTION 0U

/* Move/read APIs wait for DMA completion; call them from a FreeRTOS task. */

/**
 * @brief 初始化四个 CoreXY 电机。
 * @details 按 A1=USART2、A2=USART3、B1=UART5、B2=USART6 绑定 UART，为每路打开空闲中断 + DMA 应答接收，
 *          并将当前规划坐标设为原点。
 * @retval HAL 状态码；任一路 UART 未配置 RX DMA 时返回 HAL_ERROR。
 */
HAL_StatusTypeDef CoreXY_Init(void);

/**
 * @brief 移动到绝对坐标。
 * @param x 目标 X 坐标，单位 mm。
 * @param y 目标 Y 坐标，单位 mm。
 * @param speed 电机最高速度，单位 RPM。
 * @param acceleration 加速度，范围 0 至 255。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_MoveTo(float x, float y, uint16_t speed,
                                uint8_t acceleration);

/**
 * @brief 从当前位置相对移动。
 * @param dx X 方向位移，单位 mm。
 * @param dy Y 方向位移，单位 mm。
 * @param speed 电机最高速度，单位 RPM。
 * @param acceleration 加速度，范围 0 至 255。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_MoveRelative(float dx, float dy, uint16_t speed,
                                      uint8_t acceleration);

/**
 * @brief 返回软件坐标原点。
 * @param speed 电机最高速度，单位 RPM。
 * @param acceleration 加速度，范围 0 至 255。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_MoveOrigin(uint16_t speed, uint8_t acceleration);

#endif
