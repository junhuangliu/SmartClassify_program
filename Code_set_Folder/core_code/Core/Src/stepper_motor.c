#include "stepper_motor.h"
#include "cmsis_os2.h"
#include <limits.h>

#define STEPPER_MOTOR_FRAME_END 0x6BU
#define STEPPER_MOTOR_CMD_READ_SPEED 0x35U
#define STEPPER_MOTOR_CMD_READ_POSITION 0x36U
#define STEPPER_MOTOR_CMD_READ_HOMING 0x3BU
#define STEPPER_MOTOR_TX_TIMEOUT_MS 100U

StepperMotor_t stepper_motor_a1;
StepperMotor_t stepper_motor_a2;
StepperMotor_t stepper_motor_b1;
StepperMotor_t stepper_motor_b2;

/**
 * @brief 等待本电机 DMA 发送完成。
 * @details EMM_V5 命令使用共享静态发送缓冲区；等待当前 UART 发完后再发下一条，防止缓冲区被覆盖。
 * @param motor 电机对象。
 * @retval HAL_TIMEOUT 表示等待超时，否则返回 HAL_OK。
 */
static HAL_StatusTypeDef StepperMotor_WaitTx(StepperMotor_t *motor)
{
  uint32_t start = HAL_GetTick();

  while (motor->huart->gState == HAL_UART_STATE_BUSY_TX)
  {
    if ((HAL_GetTick() - start) >= STEPPER_MOTOR_TX_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
    osDelay(1U);
  }

  return HAL_OK;
}

/**
 * @brief 启动（或重启）本电机的空闲中断 + DMA 接收。
 * @details
 * - 设计意图：EMM_V5 应答帧长度不固定（普通应答 4 字节、转速 6 字节、位置 8 字节），
 *   固定长度 DMA 会把不同长度的帧拼错位，因此用 ReceiveToIdle，靠总线空闲切分报文。
 * - 实现要点：空闲事件后 HAL 在 Normal 模式已停止接收，必须重新调用本函数才能继续收；
 *   关掉 DMA 半传输中断，避免半帧被当成一次事件重复解析。
 * - 注意事项：只能在中断/任务上下文调用；循环模式重复调用会返回 HAL_BUSY（无害）。
 * @param motor 电机对象。
 * @retval HAL_OK 已启动接收；HAL_BUSY 表示循环模式接收仍在进行；HAL_ERROR 表示该 UART 未配置 RX DMA。
 */
static HAL_StatusTypeDef StepperMotor_StartRx(StepperMotor_t *motor)
{
  HAL_StatusTypeDef status;

  if (motor->huart->hdmarx == NULL)
  {
    return HAL_ERROR;
  }

  status = HAL_UARTEx_ReceiveToIdle_DMA(motor->huart, motor->rx_dma_buf,
                                       STEPPER_MOTOR_RX_DMA_BUF_SIZE);
  if (status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(motor->huart->hdmarx, DMA_IT_HT);
  }
  return status;
}

/**
 * @brief 初始化电机对象并开启应答接收。
 * @details 关联 UART、清空速度/位置/解析状态后，立即打开空闲中断 + DMA 接收，
 *          这样后续 ReadSpeed/ReadPosition 发出的查询应答会由中断自动解析进结构体。
 * @param motor 电机对象。
 * @param huart 此电机专用的 UART 句柄。
 * @retval HAL 状态码；该 UART 未配置 RX DMA 时返回 HAL_ERROR。
 */
HAL_StatusTypeDef StepperMotor_Init(StepperMotor_t *motor, UART_HandleTypeDef *huart)
{
  motor->huart = huart;
  motor->speed_rpm = 0;
  motor->position_pulses = 0;
  motor->homing_status = 0U;
  motor->valid_data = 0U;
  motor->rx_length = 0U;
  motor->rx_dma_size = 0U;
  return StepperMotor_StartRx(motor);
}

/**
 * @brief 电机串口接收事件回调（空闲中断）。
 * @details
 * - 设计意图：在中断里直接按 huart 分辨是哪台电机，把本次收到的应答解析进对应结构体，
 *   并立刻重新开启接收，保证闭环反馈连续。
 * - 实现要点：Normal 模式下 Size 就是本次突发长度，从偏移 0 解析；
 *   循环模式下 Size 是自启动以来的累计写入量，只解析新增部分，避免重复解析旧数据。
 * - 注意事项：运行在 UART 中断上下文，只做解析和重启接收；未初始化的电机 huart 为 NULL，
 *   比较自然不成立，不会误入分支。
 * @param huart 触发事件的 UART 句柄。
 * @param size 本次空闲前累计接收到的字节数。
 */
void StepperMotor_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart == stepper_motor_a1.huart)
  {
    if (size > stepper_motor_a1.rx_dma_size)
    {
      (void)StepperMotor_ParseDmaData(&stepper_motor_a1,
                                      &stepper_motor_a1.rx_dma_buf[stepper_motor_a1.rx_dma_size],
                                      (uint16_t)(size - stepper_motor_a1.rx_dma_size));
    }
    stepper_motor_a1.rx_dma_size = (huart->RxState == HAL_UART_STATE_READY) ? 0U : size;
    (void)StepperMotor_StartRx(&stepper_motor_a1);
  }
  else if (huart == stepper_motor_a2.huart)
  {
    if (size > stepper_motor_a2.rx_dma_size)
    {
      (void)StepperMotor_ParseDmaData(&stepper_motor_a2,
                                      &stepper_motor_a2.rx_dma_buf[stepper_motor_a2.rx_dma_size],
                                      (uint16_t)(size - stepper_motor_a2.rx_dma_size));
    }
    stepper_motor_a2.rx_dma_size = (huart->RxState == HAL_UART_STATE_READY) ? 0U : size;
    (void)StepperMotor_StartRx(&stepper_motor_a2);
  }
  else if (huart == stepper_motor_b1.huart)
  {
    if (size > stepper_motor_b1.rx_dma_size)
    {
      (void)StepperMotor_ParseDmaData(&stepper_motor_b1,
                                      &stepper_motor_b1.rx_dma_buf[stepper_motor_b1.rx_dma_size],
                                      (uint16_t)(size - stepper_motor_b1.rx_dma_size));
    }
    stepper_motor_b1.rx_dma_size = (huart->RxState == HAL_UART_STATE_READY) ? 0U : size;
    (void)StepperMotor_StartRx(&stepper_motor_b1);
  }
  else if (huart == stepper_motor_b2.huart)
  {
    if (size > stepper_motor_b2.rx_dma_size)
    {
      (void)StepperMotor_ParseDmaData(&stepper_motor_b2,
                                      &stepper_motor_b2.rx_dma_buf[stepper_motor_b2.rx_dma_size],
                                      (uint16_t)(size - stepper_motor_b2.rx_dma_size));
    }
    stepper_motor_b2.rx_dma_size = (huart->RxState == HAL_UART_STATE_READY) ? 0U : size;
    (void)StepperMotor_StartRx(&stepper_motor_b2);
  }
}

/**
 * @brief 电机串口错误回调。
 * @details 溢出/帧错误会让 HAL 结束接收，若不重开会永久丢失该电机的反馈数据；
 *          这里按 huart 分辨电机，清标志、丢弃半帧后重启接收。
 * @param huart 出错的 UART 句柄。
 */
void StepperMotor_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == stepper_motor_a1.huart)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    stepper_motor_a1.rx_length = 0U;
    stepper_motor_a1.rx_dma_size = 0U;
    (void)StepperMotor_StartRx(&stepper_motor_a1);
  }
  else if (huart == stepper_motor_a2.huart)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    stepper_motor_a2.rx_length = 0U;
    stepper_motor_a2.rx_dma_size = 0U;
    (void)StepperMotor_StartRx(&stepper_motor_a2);
  }
  else if (huart == stepper_motor_b1.huart)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    stepper_motor_b1.rx_length = 0U;
    stepper_motor_b1.rx_dma_size = 0U;
    (void)StepperMotor_StartRx(&stepper_motor_b1);
  }
  else if (huart == stepper_motor_b2.huart)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    stepper_motor_b2.rx_length = 0U;
    stepper_motor_b2.rx_dma_size = 0U;
    (void)StepperMotor_StartRx(&stepper_motor_b2);
  }
}

/**
 * @brief 请求读取电机速度。
 * @details 使用 EMM_V5 系统参数命令；收到的响应由 StepperMotor_ParseDmaData 更新到 speed_rpm。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_ReadSpeed(StepperMotor_t *motor)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  status = Emm_V5_Read_Sys_Params(motor->huart, S_VEL);
  return (status == HAL_OK) ? StepperMotor_WaitTx(motor) : status;
}

/**
 * @brief 请求读取电机位置。
 * @details 使用 EMM_V5 系统参数命令；收到的响应由 StepperMotor_ParseDmaData 更新到 position_pulses。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_ReadPosition(StepperMotor_t *motor)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  status = Emm_V5_Read_Sys_Params(motor->huart, S_CPOS);
  return (status == HAL_OK) ? StepperMotor_WaitTx(motor) : status;
}

/**
 * @brief 立即停止电机。
 * @details 等待该 UART 的 DMA 命令发送完成后，再发送 EMM_V5 停止命令。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_Stop(StepperMotor_t *motor)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  status = Emm_V5_Stop_Now(motor->huart, false);
  return (status == HAL_OK) ? StepperMotor_WaitTx(motor) : status;
}

/**
 * @brief 清除本电机缓存的回零状态。
 * @details
 * - 设计意图：0x3B 查询是异步应答，清除有效标志后才能判断本次查询是否真的收到新数据。
 * - 实现要点：同时清零状态字节和 valid_data 中的回零有效位，不影响速度/位置的有效位。
 * - 注意事项：可在任务或中断上下文调用；字段为 volatile，避免编译器合并读写。
 * @param motor 电机对象。
 */
void StepperMotor_ClearHomingStatus(StepperMotor_t *motor)
{
  motor->homing_status = 0U;
  motor->valid_data &= ~STEPPER_MOTOR_HOMING_VALID;
}

/**
 * @brief 请求读取本电机回零状态。
 * @details
 * - 设计意图：碰撞回零期间需要反复查询 0x3B，确认电机是否仍在回零、是否失败。
 * - 实现要点：先等待共享发送缓冲区空闲，再发 S_OFLAG 查询，最后等待 DMA 发完；
 *   应答由 StepperMotor_ParseDmaData 在 UART 空闲中断里写入 homing_status。
 * - 注意事项：本函数只保证命令发完，不保证应答已到；调用方应留出总线空闲时间再检查有效位。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_ReadHomingStatus(StepperMotor_t *motor)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  status = Emm_V5_Read_Sys_Params(motor->huart, S_OFLAG);
  return (status == HAL_OK) ? StepperMotor_WaitTx(motor) : status;
}

/**
 * @brief 配置 EMM_V5 回零参数。
 * @details
 * - 设计意图：把模式、方向、速度、超时和碰撞检测阈值一次性写入驱动器，供后续回零触发使用。
 * - 实现要点：EMM_V5 命令缓冲区是静态共享的，因此发送前后都等待本 UART 的 DMA 发送完成；
 *   A/B 两路必须串行调用，不能同时进入本函数。
 * - 数据流：参数经 Emm_V5_Origin_Modify_Params 打包，通过本电机 UART 的 DMA 发出。
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
                                             bool store)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  Emm_V5_Origin_Modify_Params(motor->huart, store, mode, direction, vel,
                              timeout_ms, sl_vel, sl_ma, sl_ms, false);
  return StepperMotor_WaitTx(motor);
}

/**
 * @brief 触发本电机回零。
 * @details
 * - 设计意图：在参数配置完成后启动回零；sync=true 时先缓存命令，等同步触发后两轴一起动。
 * - 实现要点：等待 DMA 空闲后发送回零触发命令，再等待发送完成，防止共享缓冲区被下一路覆盖。
 * - 注意事项：sync=true 时还需由调用方发送 Emm_V5_Synchronous_motion 才会真正开始运动。
 * @param motor 电机对象。
 * @param mode 回零模式，2 为无限位碰撞回零。
 * @param sync 是否等待 EMM_V5 同步触发。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_HomeTrigger(StepperMotor_t *motor, uint8_t mode,
                                           bool sync)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  Emm_V5_Origin_Trigger_Return(motor->huart, mode, sync);
  return StepperMotor_WaitTx(motor);
}

/**
 * @brief 中止本电机回零并立即停机。
 * @details
 * - 设计意图：回零失败、超时或用户发 init:0 时，必须尽快撤销回零并停止电机，避免持续顶住硬限位。
 * - 实现要点：先发回零中断命令，再发立即停止命令，两条命令之间都等待 DMA 发送完成。
 * - 注意事项：本函数会占用该 UART 的发送通道，只应由正在执行回零的任务调用，避免与状态轮询并发。
 * @param motor 电机对象。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_HomeAbort(StepperMotor_t *motor)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  Emm_V5_Origin_Interrupt(motor->huart);
  status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  return StepperMotor_Stop(motor);
}

/**
 * @brief 设置电机速度。
 * @details 等待共享 EMM_V5 命令缓冲区可用后发送速度命令，并等待 DMA 发送完成。
 * @param motor 电机对象。
 * @param direction 方向，0 为正向，1 为反向。
 * @param speed_rpm 速度，范围 0 至 5000 RPM。
 * @param acceleration 加速度，范围 0 至 255。
 * @param sync 是否等待 EMM_V5 同步触发。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef StepperMotor_SetSpeed(StepperMotor_t *motor, uint8_t direction,
                                        uint16_t speed_rpm, uint8_t acceleration,
                                        bool sync)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  status = Emm_V5_Vel_Control(motor->huart, direction, speed_rpm,
                              acceleration, sync);
  return (status == HAL_OK) ? StepperMotor_WaitTx(motor) : status;
}

/**
 * @brief 设置电机位置模式运动。
 * @details 等待共享 EMM_V5 命令缓冲区可用后发送位置命令，并等待 DMA 发送完成。
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
                                           uint32_t pulses, bool relative, bool sync)
{
  HAL_StatusTypeDef status = StepperMotor_WaitTx(motor);
  if (status != HAL_OK)
  {
    return status;
  }
  status = Emm_V5_Pos_Control(motor->huart, direction, speed_rpm, acceleration,
                              pulses, relative ? 0U : 1U, sync);
  return (status == HAL_OK) ? StepperMotor_WaitTx(motor) : status;
}


/////****************************************///////////////////
/**
 * @brief 解析本电机 UART 收到的 DMA 数据。
 * @details 按地址、命令码和帧结束字节解析速度或位置响应；数据位标记用于调用方判断测量值是否更新。
 * @param motor 接收数据的电机对象。
 * @param data DMA 接收数据。
 * @param length 本次新收到的字节数。
 * @retval true 表示至少解析到一帧有效响应。
 */
bool StepperMotor_ParseDmaData(StepperMotor_t *motor, const uint8_t *data,
                               uint16_t length)
{
  bool parsed = false;
  uint16_t i;

  for (i = 0U; i < length; ++i)
  {
    uint8_t byte = data[i];

    if (motor->rx_length == 0U)
    {
      if (byte == STEPPER_MOTOR_ADDRESS)
      {
        motor->rx_frame[0] = byte;
        motor->rx_length = 1U;
      }
      continue;
    }

    if ((motor->rx_length == 1U) &&
        (byte != STEPPER_MOTOR_CMD_READ_SPEED) &&
        (byte != STEPPER_MOTOR_CMD_READ_POSITION) &&
        (byte != STEPPER_MOTOR_CMD_READ_HOMING))
    {
      motor->rx_length = (byte == STEPPER_MOTOR_ADDRESS) ? 1U : 0U;
      if (motor->rx_length != 0U)
      {
        motor->rx_frame[0] = byte;
      }
      continue;
    }

    motor->rx_frame[motor->rx_length++] = byte;
    if (motor->rx_length == 2U)
    {
      continue;
    }

    uint8_t frame_length;

    switch (motor->rx_frame[1])
    {
      case STEPPER_MOTOR_CMD_READ_SPEED:
        frame_length = 6U;
        break;
      case STEPPER_MOTOR_CMD_READ_POSITION:
        frame_length = STEPPER_MOTOR_RX_FRAME_SIZE;
        break;
      case STEPPER_MOTOR_CMD_READ_HOMING:
        frame_length = STEPPER_MOTOR_HOMING_FRAME_SIZE;
        break;
      default:
        frame_length = 0U;
        break;
    }

    if (frame_length == 0U)
    {
      motor->rx_length = 0U;
      continue;
    }

    if (motor->rx_length == frame_length)
    {
      if (motor->rx_frame[frame_length - 1U] == STEPPER_MOTOR_FRAME_END)
      {
        switch (motor->rx_frame[1])
        {
          case STEPPER_MOTOR_CMD_READ_SPEED:
          {
            uint16_t speed = ((uint16_t)motor->rx_frame[3] << 8) | motor->rx_frame[4];
            if (speed <= INT16_MAX)
            {
              motor->speed_rpm = (motor->rx_frame[2] == 0U) ?
                                 (int16_t)speed : -(int16_t)speed;
              motor->valid_data |= STEPPER_MOTOR_SPEED_VALID;
              parsed = true;
            }
            break;
          }
          case STEPPER_MOTOR_CMD_READ_POSITION:
          {
            uint32_t position = ((uint32_t)motor->rx_frame[3] << 24) |
                                ((uint32_t)motor->rx_frame[4] << 16) |
                                ((uint32_t)motor->rx_frame[5] << 8) |
                                motor->rx_frame[6];
            int64_t signed_position = (motor->rx_frame[2] == 0U) ?
                                      (int64_t)position : -(int64_t)position;
            if ((signed_position >= INT32_MIN) && (signed_position <= INT32_MAX))
            {
              motor->position_pulses = (int32_t)signed_position;
              motor->valid_data |= STEPPER_MOTOR_POSITION_VALID;
              parsed = true;
            }
            break;
          }
          case STEPPER_MOTOR_CMD_READ_HOMING:
            motor->homing_status = motor->rx_frame[2];
            motor->valid_data |= STEPPER_MOTOR_HOMING_VALID;
            parsed = true;
            break;
          default:
            break;
        }
      }
      motor->rx_length = 0U;
    }
  }

  return parsed;
}
