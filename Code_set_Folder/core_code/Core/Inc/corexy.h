#ifndef COREXY_H
#define COREXY_H

#include "stepper_motor.h"

#define COREXY_MOTOR_A_UART_HANDLE (&huart2)
#define COREXY_MOTOR_B_UART_HANDLE (&huart3)

#define COREXY_FULL_STEPS_PER_REV 200.0f
#define COREXY_MICROSTEPS 16.0f
#define COREXY_BELT_TOOTH_PITCH_MM 2.0f
#define COREXY_PULLEY_TEETH 20.0f
#define COREXY_PULSES_PER_MM \
  ((COREXY_FULL_STEPS_PER_REV * COREXY_MICROSTEPS) / \
   (COREXY_BELT_TOOTH_PITCH_MM * COREXY_PULLEY_TEETH))

#define COREXY_MOTOR_A_INVERT_DIRECTION 0U
#define COREXY_MOTOR_B_INVERT_DIRECTION 0U

#define COREXY_HOME_MODE 2U

/*
 * X 轴回零时 A/B 同向；Y 轴回零时 A/B 反向。
 * 首次上机请低速试探，按实际机械方向修改以下四个宏。
 */
#define COREXY_HOME_X_DIR_A 0U
#define COREXY_HOME_X_DIR_B 0U
#define COREXY_HOME_Y_DIR_A 0U
#define COREXY_HOME_Y_DIR_B 1U

#define COREXY_HOME_O_VEL 30U
#define COREXY_HOME_O_TM 10000U
#define COREXY_HOME_SL_VEL 300U
#define COREXY_HOME_SL_MA 800U
#define COREXY_HOME_SL_MS 60U
#define COREXY_HOME_TIMEOUT_MS 15000U
#define COREXY_HOME_START_DELAY_MS 50U
#define COREXY_HOME_POLL_MS 20U
#define COREXY_HOME_START_TIMEOUT_MS 1500U

typedef enum
{
  COREXY_CMD_NONE = 0,
  COREXY_CMD_HOME,
} CoreXY_Cmd_t;

typedef enum
{
  COREXY_STATE_IDLE = 0,
  COREXY_STATE_HOMING,
  COREXY_STATE_READY,
  COREXY_STATE_ERROR,
  COREXY_STATE_INIT_FAIL,
} CoreXY_State_t;

/**
 * @brief 初始化两电机 CoreXY 并创建回零命令队列。
 * @details A=USART2、B=USART3；为两路打开空闲中断 + DMA 应答接收，清空规划坐标，
 *          并创建 CoreXYTask 使用的命令队列。
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

/**
 * @brief 请求执行碰撞回零。
 * @details 只把回零命令放入队列并立即返回，实际阻塞回零由 CoreXYTask 执行，
 *          避免阻塞 UART1 命令接收任务。
 * @retval HAL_OK 已入队；HAL_BUSY 正在回零或队列满；HAL_ERROR 未初始化。
 */
HAL_StatusTypeDef CoreXY_RequestHome(void);

/**
 * @brief 请求中止回零或停止电机。
 * @details 回零进行中只置中止标志，由回零任务在轮询点安全撤销；
 *          未回零时直接停止 A/B 两电机。
 * @retval HAL_OK。
 */
HAL_StatusTypeDef CoreXY_RequestAbort(void);

/**
 * @brief 阻塞等待一条 CoreXY 命令。
 * @details 由 CoreXYTask 调用，内部使用 FreeRTOS 消息队列永久等待。
 * @retval 收到的命令；队列不可用时返回 COREXY_CMD_NONE。
 */
CoreXY_Cmd_t CoreXY_WaitCommand(void);

/**
 * @brief 执行两段式碰撞回零（先 X 后 Y）。
 * @details 使用 EMM_V5 模式 2，通过 0x3B 查询回零状态；成功后软件坐标置原点。
 *          本函数阻塞数秒，只能在 CoreXYTask 中调用。
 * @retval HAL_OK 成功；HAL_TIMEOUT 超时；HAL_ERROR 失败或中止。
 */
HAL_StatusTypeDef CoreXY_HomeTorque(void);

/**
 * @brief CoreXY 回零任务入口。
 * @details 阻塞等待命令队列，收到回零命令后执行 CoreXY_HomeTorque，
 *          并通过 USART1 回复完成或失败结果。
 * @param argument 未使用。
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
