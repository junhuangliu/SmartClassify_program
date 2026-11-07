#ifndef COREXY_H
#define COREXY_H

#include "stepper_motor.h"

#define COREXY_MOTOR_A_UART_HANDLE (&huart3)
#define COREXY_MOTOR_B_UART_HANDLE (&huart2)

#define COREXY_FULL_STEPS_PER_REV 200.0f
#define COREXY_MICROSTEPS 16.0f
#define COREXY_BELT_TOOTH_PITCH_MM 2.0f
#define COREXY_PULLEY_TEETH 20.0f
#define COREXY_PULSES_PER_MM \
  ((COREXY_FULL_STEPS_PER_REV * COREXY_MICROSTEPS) / \
   (COREXY_BELT_TOOTH_PITCH_MM * COREXY_PULLEY_TEETH))

#define COREXY_MOTOR_A_INVERT_DIRECTION 0U
#define COREXY_MOTOR_B_INVERT_DIRECTION 0U

typedef enum
{
  COREXY_STATE_IDLE = 0,
  COREXY_STATE_INIT_FAIL,
} CoreXY_State_t;

/**
 * @brief 初始化两电机 CoreXY 通信。
 * @details 为两路打开空闲中断 + DMA 应答接收，并清空规划坐标。
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
 * @brief 碰撞回零：X、Y 两轴依次压到机械硬限位，把该位置设为坐标原点 (0,0)。
 *
 * @details
 * - 机构无限位开关，用驱动器 0x3A 的堵转标志（Cgi_TF/Cgp_TF）判定撞到限位。
 * - 速度模式持续朝负方向运动 → 撞停即急停 → 解除堵转保护 → 驱动器位置清零，
 *   同时把 corexy_target_x/y 清 0。
 * - 阻塞执行：单轴最长 20s，整机约 30s（20mm/s、300mm 行程）。
 *
 * @param 无
 * @retval HAL_OK      两轴均已回零，规划坐标与驱动器位置均已清零
 * @retval HAL_TIMEOUT 某一轴等待堵转超时（已停机；坐标不回滚，由调用方决定重试或报警）
 * @retval HAL_ERROR   速度命令下发失败，或电机未动作（多半是驱动器回 E2 拒绝执行）
 */
HAL_StatusTypeDef CoreXY_MoveOrigin(void);

/**
 * @brief CoreXY 电机任务入口。
 * @details 在任务上下文执行用户的 CoreXY 电机控制逻辑。
 * @param argument 传给任务的参数。
 */
void CoreXYTask(void *argument);

/**
 * @brief 获取当前 CoreXY 状态。
 * @retval 状态枚举。
 */
CoreXY_State_t CoreXY_GetState(void);

/**
 * @brief 获取当前 CoreXY 状态的文本表示。
 * @retval 静态字符串，用于 UART1 STATUS 回复。
 */
const char *CoreXY_GetStateString(void);

#endif
