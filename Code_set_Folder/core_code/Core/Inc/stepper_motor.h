#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Emm_V5.h"

#define STEPPER_MOTOR_ADDRESS 0x01U
#define STEPPER_MOTOR_RX_FRAME_SIZE 8U
#define STEPPER_MOTOR_HOMING_FRAME_SIZE 4U
#define STEPPER_MOTOR_RX_DMA_BUF_SIZE 32U
#define STEPPER_MOTOR_SPEED_VALID (1U << 0)
#define STEPPER_MOTOR_POSITION_VALID (1U << 1)
#define STEPPER_MOTOR_HOMING_VALID (1U << 2)
#define STEPPER_MOTOR_OFLAG_RUNNING (1U << 2)
#define STEPPER_MOTOR_OFLAG_FAILED (1U << 3)

typedef struct
{
  UART_HandleTypeDef *huart;
  volatile int16_t speed_rpm;
  volatile int32_t position_pulses;
  volatile uint8_t homing_status;
  volatile uint8_t valid_data;
  uint8_t rx_dma_buf[STEPPER_MOTOR_RX_DMA_BUF_SIZE];
  uint8_t rx_frame[STEPPER_MOTOR_RX_FRAME_SIZE];
  volatile uint8_t rx_length;
  volatile uint16_t rx_dma_size;
} StepperMotor_t;

extern StepperMotor_t stepper_motor_a1;
extern StepperMotor_t stepper_motor_a2;
extern StepperMotor_t stepper_motor_b1;
extern StepperMotor_t stepper_motor_b2;

/**
 * @brief 初始化电机对象并开启应答接收。
 * @details 关联 CubeMX 初始化的 UART，清空速度、位置及接收解析状态，然后启动
 *          空闲中断 + DMA 接收；电机应答由 StepperMotor_RxEventCallback 自动解析。
 * @param motor 电机对象。
 * @param huart 此电机专用的 UART 句柄。
 * @retval HAL 状态码；该 UART 未配置 RX DMA 时返回 HAL_ERROR。
 */
HAL_StatusTypeDef StepperMotor_Init(StepperMotor_t *motor, UART_HandleTypeDef *huart);

/**
 * @brief 请求读取电机速度。
 * @details 使用 EMM_V5 系统参数命令；收到的响应由 StepperMotor_ParseDmaData 更新到 speed_rpm。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_ReadSpeed(StepperMotor_t *motor);

/**
 * @brief 请求读取电机位置。
 * @details 使用 EMM_V5 系统参数命令；收到的响应由 StepperMotor_ParseDmaData 更新到 position_pulses。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_ReadPosition(StepperMotor_t *motor);

/**
 * @brief 立即停止电机。
 * @details 通过 EMM_V5 停止命令停机，不等待多机同步触发。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_Stop(StepperMotor_t *motor);

/**
 * @brief 设置电机速度。
 * @details 使用 EMM_V5 速度命令；sync 为 true 时等待后续同步触发。
 * @param motor 电机对象。
 * @param direction 方向，0 为正向，1 为反向。
 * @param speed_rpm 速度，范围 0 至 5000 RPM。
 * @param acceleration 加速度，范围 0 至 255。
 * @param sync 是否等待 EMM_V5 同步触发。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_SetSpeed(StepperMotor_t *motor, uint8_t direction,
                                        uint16_t speed_rpm, uint8_t acceleration,
                                        bool sync);

/**
 * @brief 设置电机位置模式运动。
 * @details 使用 EMM_V5 位置命令；relative 选择相对/绝对位置模式，sync 为 true 时等待同步触发。
 * @param motor 电机对象。
 * @param direction 方向，0 为正向，1 为反向。
 * @param speed_rpm 速度，范围 0 至 5000 RPM。
 * @param acceleration 加速度，范围 0 至 255。
 * @param pulses 运动脉冲数。
 * @param relative 是否相对运动，true 表示相对运动。
 * @param sync 是否等待 EMM_V5 同步触发。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_SetPosition(StepperMotor_t *motor, uint8_t direction,
                                           uint16_t speed_rpm, uint8_t acceleration,
                                           uint32_t pulses, bool relative, bool sync);

/**
 * @brief 清除本电机缓存的回零状态。
 * @details 在发起 0x3B 查询前调用，避免把上一次查询结果误当成新结果。
 * @param motor 电机对象。
 */
void StepperMotor_ClearHomingStatus(StepperMotor_t *motor);

/**
 * @brief 请求读取本电机回零状态。
 * @details 使用 EMM_V5 的 S_OFLAG 查询；应答由 StepperMotor_ParseDmaData 解析到 homing_status，
 *          并置位 STEPPER_MOTOR_HOMING_VALID。调用方需等待应答到达后再读取。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_ReadHomingStatus(StepperMotor_t *motor);

/**
 * @brief 配置 EMM_V5 回零参数。
 * @details 设置回零模式（2 为无限位碰撞回零）、方向、速度、超时及碰撞检测参数；
 *          共享命令缓冲区发完后才返回，调用方可在 A/B 两路间安全复用。
 * @param motor 电机对象。
 * @param mode 回零模式，2 为无限位碰撞回零。
 * @param direction 回零方向，0 为正向，1 为反向。
 * @param vel 回零速度，单位 RPM。
 * @param timeout_ms 回零超时，单位 ms。
 * @param sl_vel 碰撞检测速度阈值，单位 RPM。
 * @param sl_ma 碰撞检测电流阈值，单位 mA。
 * @param sl_ms 碰撞检测持续时间，单位 ms。
 * @param store 是否将参数保存到电机内部。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_HomeSetParams(StepperMotor_t *motor, uint8_t mode,
                                             uint8_t direction, uint16_t vel,
                                             uint32_t timeout_ms, uint16_t sl_vel,
                                             uint16_t sl_ma, uint16_t sl_ms,
                                             bool store);

/**
 * @brief 触发本电机回零。
 * @details 使用 EMM_V5 回零触发命令；sync 为 true 时等待同步触发。
 * @param motor 电机对象。
 * @param mode 回零模式，2 为无限位碰撞回零。
 * @param sync 是否等待 EMM_V5 同步触发。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_HomeTrigger(StepperMotor_t *motor, uint8_t mode,
                                           bool sync);

/**
 * @brief 中止本电机回零并立即停机。
 * @details 先发送 EMM_V5 回零中断命令，再发送立即停止命令，确保碰撞回零不会继续顶住限位。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_HomeAbort(StepperMotor_t *motor);

/**
 * @brief 解析本电机 UART 收到的 DMA 数据。
 * @details 解析 EMM_V5 速度、实时位置和回零状态响应，支持帧分段及连续接收。
 * @param motor 接收数据的电机对象。
 * @param data DMA 接收数据。
 * @param length 本次新收到的字节数。
 * @retval true 表示至少解析到一帧有效响应。
 */
bool StepperMotor_ParseDmaData(StepperMotor_t *motor, const uint8_t *data,
                               uint16_t length);

/**
 * @brief 电机串口接收事件回调（由 HAL_UARTEx_RxEventCallback 分发进来）。
 * @details 空闲中断触发后取本次新收到的字节解析应答，并重新启动接收。
 * @param huart 触发事件的 UART 句柄。
 * @param size 本次空闲前累计接收到的字节数。
 */
void StepperMotor_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief 电机串口错误回调（由 HAL_UART_ErrorCallback 分发进来）。
 * @details 清除溢出标志并重启接收，避免一次 ORE 让闭环反馈永久失效。
 * @param huart 出错的 UART 句柄。
 */
void StepperMotor_ErrorCallback(UART_HandleTypeDef *huart);

#endif
