#include "corexy.h"
#include "ws2812_spi.h"
#include "cmsis_os2.h"
#include "uart1_proto.h"
#include "turntable.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "Servo.h"
#include "WaterPump.h"
#include "Grab.h"
#define COREXY_TX_TIMEOUT_US 100000U


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static float corexy_target_x;
static float corexy_target_y;
static volatile CoreXY_State_t corexy_state = COREXY_STATE_IDLE;
static const char *corexy_origin_error_message;

/**
 * @brief 配置并启动 CoreXY 独占的 TIM5 微秒计时器。
 * @details
 * - 设计意图：为运动超时提供独立于 HAL/FreeRTOS SysTick 的硬件计时基准。
 * - 实现要点：读取 APB1 定时器时钟，配置 TIM5 每 1 us 计数一次并以 32 位自由运行。
 * - 注意事项：TIM5 专供 CoreXY 计时，不使用中断；当前 APB1 定时器时钟需为 1 MHz 的整数倍。
 * @retval HAL_OK 配置成功；HAL_ERROR 时钟频率不支持整数微秒分频。
 */
static HAL_StatusTypeDef CoreXY_TimebaseInit(void)
{
  RCC_ClkInitTypeDef clock_config = {0};
  uint32_t flash_latency = 0U;
  uint32_t timer_clock_hz;

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  timer_clock_hz = HAL_RCC_GetPCLK1Freq();
  if (clock_config.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    timer_clock_hz *= 2U;
  }

  if ((timer_clock_hz < 1000000U) || ((timer_clock_hz % 1000000U) != 0U))
  {
    return HAL_ERROR;
  }

  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->CR1 = 0U;
  TIM5->DIER = 0U;
  TIM5->PSC = (timer_clock_hz / 1000000U) - 1U;
  TIM5->ARR = UINT32_MAX;
  TIM5->EGR = TIM_EGR_UG;
  TIM5->CNT = 0U;
  TIM5->SR = 0U;
  TIM5->CR1 = TIM_CR1_CEN;

  return HAL_OK;
}

/**
 * @brief 读取 TIM5 当前的微秒计数。
 * @details TIM5 以 1 MHz 自由运行；对计数值做无符号差可安全处理单次溢出。
 * @retval 当前计数值，单位 us。
 */
static uint32_t CoreXY_TimeNowUs(void)
{
  return TIM5->CNT;
}

/**
 * @brief 等待指定电机的 DMA 发送完成。
 * @details
 * - 设计意图：EMM_V5 库使用静态共享发送缓冲区，上一条命令未发完就发下一条会被覆盖。
 * - 实现要点：轮询 UART 的 gState，使用 TIM5 计量发送超时；等待期间让出 CPU。
 * - 注意事项：本函数可在任务上下文调用，不能在中断中调用。
 * @param motor 电机对象。
 * @retval HAL_OK 发送完成；HAL_TIMEOUT 等待超时。
 */
static HAL_StatusTypeDef CoreXY_WaitTx(StepperMotor_t *motor)
{
  uint32_t start_us = CoreXY_TimeNowUs();

  while (motor->huart->gState == HAL_UART_STATE_BUSY_TX)
  {
    if ((CoreXY_TimeNowUs() - start_us) >= COREXY_TX_TIMEOUT_US)
    {
      return HAL_TIMEOUT;
    }
    osDelay(1U);
  }

  return HAL_OK;
}

/**
 * @brief 立即停止 A/B 两电机。
 * @details
 * - 设计意图：运动命令失败时停止两台电机，避免只动一侧导致机构歪斜。
 * - 实现要点：逐路发送 EMM_V5 立即停止命令；StepperMotor_Stop 内部已等待 DMA 发送完成。
 * - 注意事项：会占用两路 UART 发送通道，只应由单一控制任务调用。
 */
static void CoreXY_StopAll(void)
{
  (void)StepperMotor_Stop(&stepper_motor_a1);
  (void)StepperMotor_Stop(&stepper_motor_b1);
}

/**
 * @brief 对两电机下发一组同步位置命令。
 * @details
 * - 设计意图：CoreXY 逆解为 A=dx+dy、B=dx-dy；两路必须先缓存位置命令，再一起发同步触发，
 *   否则电机会先后启动，导致轨迹歪斜。
 * - 实现要点：按两路中较大的位移分配速度，保证两路同时到达；脉冲数四舍五入到整数。
 *   任一路发送失败立即停止两电机，避免只动一侧。
 * - 数据流：dx/dy（mm）-> 皮带位移 -> 脉冲数/方向/速度 -> EMM_V5 位置命令 -> 同步触发。
 * - 注意事项：函数会等待 DMA 发送完成，必须在 FreeRTOS 任务中调用。
 * @param dx X 方向位移，单位 mm。
 * @param dy Y 方向位移，单位 mm。
 * @param speed 电机最高速度，单位 RPM。
 * @param acceleration 加速度参数。
 * @retval HAL 状态码。
 */
static HAL_StatusTypeDef CoreXY_Move(float dx, float dy, uint16_t speed,
                                     uint8_t acceleration)
{
  StepperMotor_t *motors[2] = {&stepper_motor_a1, &stepper_motor_b1};
  const uint8_t invert[2] =
  {
    COREXY_MOTOR_A_INVERT_DIRECTION,
    COREXY_MOTOR_B_INVERT_DIRECTION
  };
  float belt_move[2] = {dx + dy, dx - dy};
  uint32_t pulses[2];
  uint8_t direction[2];
  uint16_t motor_speed[2];
  float max_move;
  HAL_StatusTypeDef status;
  uint8_t i;

  if ((speed > 5000U) || ((speed == 0U) && ((dx != 0.0f) || (dy != 0.0f))) ||
      !isfinite(dx) || !isfinite(dy))
  {
    return HAL_ERROR;
  }

  if (corexy_state == COREXY_STATE_INIT_FAIL)
  {
    return HAL_ERROR;
  }

  max_move = fmaxf(fabsf(belt_move[0]), fabsf(belt_move[1]));
  if (max_move == 0.0f)
  {
    return HAL_OK;
  }

  for (i = 0U; i < 2U; ++i)
  {
    double scaled_pulses = fabs((double)belt_move[i]) *
                           (double)COREXY_PULSES_PER_MM;
    if (scaled_pulses > ((double)UINT32_MAX - 0.5))
    {
      return HAL_ERROR;
    }
    pulses[i] = (uint32_t)(scaled_pulses + 0.5);
    direction[i] = (uint8_t)((belt_move[i] < 0.0f) ^ (invert[i] != 0U));
    motor_speed[i] = (pulses[i] == 0U) ? 0U :
                     (uint16_t)(speed * fabsf(belt_move[i]) / max_move + 0.5f);
    if ((pulses[i] != 0U) && (motor_speed[i] == 0U))
    {
      motor_speed[i] = 1U;
    }
  }

  for (i = 0U; i < 2U; ++i)
  {
    if (pulses[i] != 0U)
    {
      status = StepperMotor_SetPosition(motors[i], direction[i], motor_speed[i],
                                        acceleration, pulses[i], true, true);
      if (status != HAL_OK)
      {
        /* 位置命令发送失败：停掉可能已启动的另一台电机。 */
        CoreXY_StopAll();
        return status;
      }
    }
  }

  for (i = 0U; i < 2U; ++i)
  {
    if (pulses[i] != 0U)
    {
      status = Emm_V5_Synchronous_motion(motors[i]->huart);
      if (status != HAL_OK)
      {
        /* 同步触发命令失败：防止两台电机只启动一台。 */
        CoreXY_StopAll();
        return status;
      }
    }
  }

  for (i = 0U; i < 2U; ++i)
  {
    if (pulses[i] != 0U)
    {
      status = CoreXY_WaitTx(motors[i]);
      if (status != HAL_OK)
      {
        /* 等待发送完成失败：停止两台电机，避免运动状态不确定。 */
        CoreXY_StopAll();
        return status;
      }
    }
  }

  corexy_target_x += dx;
  corexy_target_y += dy;
  return HAL_OK;
}

/**
 * @brief 初始化两电机 CoreXY 通信。
 * @details
 * - 设计意图：把 A/B 两路绑定到对应 UART，供后续普通位置/速度控制使用。
 * - 实现要点：两路都调用 StepperMotor_Init 打开空闲中断 + DMA 接收；任一路失败都记录为初始化失败。
 * - 注意事项：在 osKernelStart 之前调用；调用前应完成 UART 和 DMA 外设初始化。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_Init(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (CoreXY_TimebaseInit() != HAL_OK)
  {
    status = HAL_ERROR;
  }
  if (StepperMotor_Init(&stepper_motor_a1, COREXY_MOTOR_A_UART_HANDLE) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  if (StepperMotor_Init(&stepper_motor_b1, COREXY_MOTOR_B_UART_HANDLE) != HAL_OK)
  {
    status = HAL_ERROR;
  }

  corexy_target_x = 0.0f;
  corexy_target_y = 0.0f;
  corexy_state = (status == HAL_OK) ? COREXY_STATE_IDLE : COREXY_STATE_INIT_FAIL;
  return status;
}

/**
 * @brief 移动到绝对坐标。
 * @details 根据当前规划坐标计算相对位移，再由 CoreXY_Move 进行逆解。
 *          机械 Y 轴方向与坐标定义相反，因此 dy 取反后下发，成功后回写真实目标坐标。
 * @param x 目标 X 坐标，单位 mm。
 * @param y 目标 Y 坐标，单位 mm。
 * @param speed 电机最高速度，单位 RPM。
 * @param acceleration 加速度参数。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_MoveTo(float x, float y, uint16_t speed,
                                uint8_t acceleration)
{
  float dx = x - corexy_target_x;
  /* 机械 Y 轴转动方向与坐标定义相反：dy 取反后再做 CoreXY 逆解 */
  float dy = corexy_target_y - y;
  HAL_StatusTypeDef status = CoreXY_Move(dx, dy, speed, acceleration);

  if (status == HAL_OK)
  {
    /* CoreXY_Move 累加的是传入的 dy，这里回写真实目标，保持 Y 坐标语义不变 */
    corexy_target_y = y;
  }
  return status;
}

/**
 * @brief 从当前位置相对移动。
 * @details CoreXY_Move 完成运动学逆解及电机同步触发。
 * @param dx X 方向位移，单位 mm。
 * @param dy Y 方向位移，单位 mm。
 * @param speed 电机最高速度，单位 RPM。
 * @param acceleration 加速度参数。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_MoveRelative(float dx, float dy, uint16_t speed,
                                      uint8_t acceleration)
{
  return CoreXY_Move(dx, dy, speed, acceleration);
}

/**
 * @brief 读取 CoreXY 当前规划坐标 X。
 * @details 直接返回 CoreXY_MoveTo 内部跟踪的坐标，模块外部不需要再单独保存一份，
 *          读取本身无副作用，可在任务上下文或运动过程中调用。
 * @retval 当前 X 坐标，单位 mm。
 */
float CoreXY_GetPositionX(void)
{
  return corexy_target_x;
}

/**
 * @brief 读取 CoreXY 当前规划坐标 Y。
 * @details 与 CoreXY_GetPositionX 配套；注意内部 Y 坐标已按机械方向做过取反处理，
 *          返回的是对外的物理坐标，可直接与 CoreXY_MoveTo 的入参比较。
 * @retval 当前 Y 坐标，单位 mm。
 */
float CoreXY_GetPositionY(void)
{
  return corexy_target_y;
}

/**
 * @brief 通过 USART1 上报指定电机最近一次 DMA 接收诊断信息。
 * @details 在任务上下文将中断中保存的原始字节、接收事件计数、HAL 错误码和接收重启状态
 *          格式化为一行文本，便于区分未收到数据、收到异常帧及 UART/DMA 错误。
 * @param motor 电机对象。
 * @param motor_name 上报使用的电机标识。
 * @param usart_name 电机连接的 USART 标识。
 */
static void CoreXY_ReportMotorRxDiagnostic(const StepperMotor_t *motor,
                                           const char *motor_name,
                                           const char *usart_name)
{
  char message[256];
  int offset;
  uint16_t i;
  uint16_t rx_length = motor->rx_last_length;

  offset = snprintf(message, sizeof(message),
                    "DIAG %s %s events=%lu len=%u errors=%lu err=0x%08lX "
                    "rxstart=%d samples=%u current=%u rx=",
                    motor_name, usart_name,
                    (unsigned long)motor->rx_event_count,
                    (unsigned int)rx_length,
                    (unsigned long)motor->rx_error_count,
                    (unsigned long)motor->rx_last_error_code,
                    (int)motor->rx_start_status,
                    (unsigned int)motor->current_sample_id,
                    (unsigned int)motor->current_ma);
  if ((offset < 0) || ((size_t)offset >= sizeof(message)))
  {
    return;
  }

  for (i = 0U; (i < rx_length) && (i < STEPPER_MOTOR_RX_DMA_BUF_SIZE); ++i)
  {
    int written = snprintf(&message[offset], sizeof(message) - (size_t)offset,
                           "%02X ", (unsigned int)motor->rx_last_data[i]);
    if ((written < 0) || ((size_t)written >= sizeof(message) - (size_t)offset))
    {
      return;
    }
    offset += written;
  }

  {
    int written = snprintf(&message[offset], sizeof(message) - (size_t)offset,
                           "\r\n");
    if ((written < 0) ||
        ((size_t)written >= sizeof(message) - (size_t)offset))
    {
      return;
    }
  }
  Uart1Proto_Send(message);
}

/**
 * @brief 以速度模式持续驱动两轴，并通过电流增量完成 CoreXY 碰撞回零。
 * @details 两轴依次沿负方向持续运动，不依赖位置查询或分段到位判断；任一电机电流相对
 *          运动基线连续超过阈值即停止两台电机。超时使用 TIM5 微秒计数，与 SysTick 分离。
 *          回零过程不操作 WS2812，灯带状态由其它模块负责。
 * @retval HAL_OK 两轴回零成功；其他状态表示初始化、通信或运动超时/失败。
 */
HAL_StatusTypeDef CoreXY_MoveOrigin(void)
{
  /************AICodeBegin************/
  const uint16_t Threshold = 1850U;
  const uint8_t Times = 3U;
  const uint16_t home_speed_rpm = 30U;
  const uint8_t home_acc = 20U;
  const uint32_t poll_ms = 20U;
  const uint32_t reply_timeout_ms = 100U;
  const uint32_t axis_timeout_ms = 20000U;
  const uint32_t command_gap_ms = 5U;
  StepperMotor_t *motors[2] = {&stepper_motor_a1, &stepper_motor_b1};
  const uint8_t invert[2] =
  {
    COREXY_MOTOR_A_INVERT_DIRECTION,
    COREXY_MOTOR_B_INVERT_DIRECTION
  };
  enum
  {
    EXIT_REASON_NORMAL = 0U,
    EXIT_REASON_INIT,
    EXIT_REASON_MOVE_COMMAND,
    EXIT_REASON_FEEDBACK_COMMAND,
    EXIT_REASON_FEEDBACK_TIMEOUT,
    EXIT_REASON_MOTION_TIMEOUT
  } exit_reason = EXIT_REASON_NORMAL;
  HAL_StatusTypeDef status = HAL_OK;
  bool motion_active = false;
  uint8_t axis;
  uint8_t i;

  corexy_origin_error_message = NULL;

  if ((corexy_state == COREXY_STATE_INIT_FAIL) || (Times == 0U))
  {
    status = HAL_ERROR;
    exit_reason = EXIT_REASON_INIT;
    goto cleanup;
  }

  for (axis = 0U; axis < 2U; ++axis)
  {
    uint16_t baseline_current[2] = {0U, 0U};
    uint8_t over_threshold[2] = {0U, 0U};
    bool baseline_ready = false;
    bool stalled;
    float dx = (axis == 0U) ? -1.0f : 0.0f;
    float dy = (axis == 1U) ? -1.0f : 0.0f;
    float belt_move[2] = {dx + dy, dx - dy};
    uint8_t direction[2];
    uint32_t axis_start_us;

    for (i = 0U; i < 2U; ++i)
    {
      Emm_V5_Reset_Clog_Pro(motors[i]->huart);
      osDelay(command_gap_ms);
    }

    for (i = 0U; i < 2U; ++i)
    {
      direction[i] = (uint8_t)((belt_move[i] < 0.0f) ^
                               (invert[i] != 0U));
    }

    motion_active = true;
    for (i = 0U; i < 2U; ++i)
    {
      status = StepperMotor_SetSpeed(motors[i], direction[i], home_speed_rpm,
                                     home_acc, true);
      if (status != HAL_OK)
      {
        exit_reason = EXIT_REASON_MOVE_COMMAND;
        goto cleanup;
      }
    }

    for (i = 0U; i < 2U; ++i)
    {
      status = Emm_V5_Synchronous_motion(motors[i]->huart);
      if (status != HAL_OK)
      {
        exit_reason = EXIT_REASON_MOVE_COMMAND;
        goto cleanup;
      }
    }

    for (i = 0U; i < 2U; ++i)
    {
      status = CoreXY_WaitTx(motors[i]);
      if (status != HAL_OK)
      {
        exit_reason = EXIT_REASON_MOVE_COMMAND;
        goto cleanup;
      }
    }

    axis_start_us = CoreXY_TimeNowUs();
    stalled = false;
    while (!stalled)
    {
      uint8_t sample_id[2];
      uint32_t reply_start_us = CoreXY_TimeNowUs();
      bool replies_ready = false;

      for (i = 0U; i < 2U; ++i)
      {
        sample_id[i] = motors[i]->current_sample_id;
        status = StepperMotor_ReadCurrent(motors[i]);
        if (status != HAL_OK)
        {
          exit_reason = EXIT_REASON_FEEDBACK_COMMAND;
          goto cleanup;
        }
      }

      while (!replies_ready)
      {
        replies_ready =
            (motors[0]->current_sample_id != sample_id[0]) &&
            (motors[1]->current_sample_id != sample_id[1]);
        if (replies_ready)
        {
          break;
        }
        if ((CoreXY_TimeNowUs() - reply_start_us) >=
            (reply_timeout_ms * 1000U))
        {
          bool motor_a_replied =
              (motors[0]->current_sample_id != sample_id[0]);
          bool motor_b_replied =
              (motors[1]->current_sample_id != sample_id[1]);

          status = HAL_TIMEOUT;
          exit_reason = EXIT_REASON_FEEDBACK_TIMEOUT;
          if (!motor_a_replied && !motor_b_replied)
          {
            corexy_origin_error_message =
                "ERR ORIGIN: no current reply from motor A/B\r\n";
          }
          else if (!motor_a_replied)
          {
            corexy_origin_error_message =
                "ERR ORIGIN: no current reply from motor A (USART3)\r\n";
          }
          else
          {
            corexy_origin_error_message =
                "ERR ORIGIN: no current reply from motor B (USART2)\r\n";
          }
          goto cleanup;
        }
        osDelay(1U);
      }

      if (!baseline_ready)
      {
        baseline_current[0] = motors[0]->current_ma;
        baseline_current[1] = motors[1]->current_ma;
        baseline_ready = true;
      }
      else
      {
        for (i = 0U; i < 2U; ++i)
        {
          int32_t current_delta = (int32_t)motors[i]->current_ma -
                                  (int32_t)baseline_current[i];
          if (current_delta >= (int32_t)Threshold)
          {
            if (over_threshold[i] < Times)
            {
              over_threshold[i]++;
            }
            if (over_threshold[i] >= Times)
            {
              stalled = true;
            }
          }
          else
          {
            over_threshold[i] = 0U;
          }
        }
      }

      if (!stalled &&
          ((CoreXY_TimeNowUs() - axis_start_us) >= (axis_timeout_ms * 1000U)))
      {
        status = HAL_TIMEOUT;
        exit_reason = EXIT_REASON_MOTION_TIMEOUT;
        goto cleanup;
      }
      if (!stalled)
      {
        osDelay(poll_ms);
      }
    }

    CoreXY_StopAll();
    motion_active = false;
    for (i = 0U; i < 2U; ++i)
    {
      Emm_V5_Reset_Clog_Pro(motors[i]->huart);
      osDelay(command_gap_ms);
      Emm_V5_Reset_CurPos_To_Zero(motors[i]->huart);
      osDelay(command_gap_ms);
      motors[i]->position_pulses = 0;
    }
    if (axis == 0U)
    {
      corexy_target_x = 0.0f;
    }
    else
    {
      corexy_target_y = 0.0f;
    }
  }

cleanup:
  if (motion_active)
  {
    CoreXY_StopAll();
  }
  if (exit_reason == EXIT_REASON_FEEDBACK_TIMEOUT)
  {
    CoreXY_ReportMotorRxDiagnostic(motors[0], "A", "USART3");
    CoreXY_ReportMotorRxDiagnostic(motors[1], "B", "USART2");
  }
  switch (exit_reason)
  {
    case EXIT_REASON_INIT:
      corexy_origin_error_message = "ERR ORIGIN: init failed\r\n";
      break;
    case EXIT_REASON_MOVE_COMMAND:
      corexy_origin_error_message = "ERR ORIGIN: motor command failed\r\n";
      break;
    case EXIT_REASON_FEEDBACK_COMMAND:
      corexy_origin_error_message = "ERR ORIGIN: current query failed\r\n";
      break;
    case EXIT_REASON_FEEDBACK_TIMEOUT:
      if (corexy_origin_error_message == NULL)
      {
        corexy_origin_error_message = "ERR ORIGIN: current feedback timeout\r\n";
      }
      break;
    case EXIT_REASON_MOTION_TIMEOUT:
      corexy_origin_error_message = "ERR ORIGIN: current threshold not reached\r\n";
      break;
    case EXIT_REASON_NORMAL:
    default:
      break;
  }
  return status;
  /************AICodeEnd************/
}
  /************AICodeBegin1************/
/**
 * @brief 仅用 USART2 反馈完成 CoreXY 两轴回零：沿 -Y 压到底（回退 15mm 已临时禁用）、再沿 -X 压到底。
 *
 * @details
 * - 设计意图：USART3 侧电流反馈不可用，只能用 USART2 对应电机 B 的相电流增量判断是否
 *   撞到硬限位；运动统一交给 CoreXY_MoveTo（位置模式），本函数只负责发目标点、轮询电流、
 *   判定后停两台电机并把该轴坐标归零。
 * - 实现要点：
 *   1) axis 0（Y 轴）：CoreXY_MoveTo(0, -10000) 走到底，电流增量连续 2 次超阈值即停两台
 *      电机并清零驱动器位置；原逻辑随后 CoreXY_MoveTo(0, 15) 回退 15mm 脱开限位、等 1.5s
 *      稳定，现已临时注释掉（取消注释即可恢复），Y 原点直接取撞停点。
 *   2) axis 1（X 轴）：CoreXY_MoveTo(-10000, 0) 走到底，判定逻辑与 Y 轴完全一致，仅阈值
 *      不同 —— Y 轴带动整个横梁负载更重（threshold_y_ma），X 轴较轻（threshold_x_ma）。
 * - 数据流：CoreXY_MoveTo 产生两路脉冲 -> StepperMotor_ReadCurrent 轮询电机 B 相电流 ->
 *   current_ma / current_sample_id -> 与运动基线比较 -> 判定撞停。
 * - 注意事项：阻塞执行，只能在 FreeRTOS 任务上下文调用（内部 osDelay）；首次采样只取基线
 *   不参与判定，避免启动瞬间误触发；单轴 20s 超时，任何失败都先停两台电机再返回。
 * @retval HAL_OK 两轴回零成功，corexy_target_x/y 均已归零。
 * @retval HAL_TIMEOUT 电流应答超时或未检测到撞停（已停机）。
 * @retval HAL_ERROR 初始化失败或命令下发失败（已停机）。
 */
HAL_StatusTypeDef CoreXY_MoveOriginUSART2(void)
{
  /* 电流阈值变量：现场调试时只改这两个值即可，单位 mA，指相对运动基线的增量 */
  uint16_t threshold_y_ma = 400U; /* Y 轴：带动整个横梁，负载重，阈值大 */
  uint16_t threshold_x_ma = 250U; /* X 轴：负载轻，阈值小 */
  const uint8_t required_samples = 2U;
  const uint32_t poll_ms = 20U;
  const uint32_t reply_timeout_ms = 100U;
  const uint32_t axis_timeout_ms = 20000U;
  const uint32_t command_gap_ms = 5U;
  StepperMotor_t *motors[2] = {&stepper_motor_a1, &stepper_motor_b1};
  StepperMotor_t *feedback_motor = &stepper_motor_b1;
  HAL_StatusTypeDef status = HAL_OK;
  bool motion_active = false;
  uint8_t axis;
  uint8_t i;

  corexy_origin_error_message = NULL;
  if ((corexy_state == COREXY_STATE_INIT_FAIL) || (required_samples == 0U))
  {
    corexy_origin_error_message = "ERR ORIGIN: init failed\r\n";
    goto cleanup;
  }

  for (axis = 0U; axis < 2U; ++axis)
  {
    /* 按当前轴取对应阈值变量：axis 0 为 Y 轴，axis 1 为 X 轴 */
    uint16_t threshold = (axis == 0U) ? threshold_y_ma : threshold_x_ma;
    uint16_t baseline_current = 0U;
    uint8_t over_threshold = 0U;
    bool baseline_ready = false;
    bool stalled = false;
    uint32_t axis_start_us;

    for (i = 0U; i < 2U; ++i)
    {
      Emm_V5_Reset_Clog_Pro(motors[i]->huart);
      osDelay(command_gap_ms);
    }

    /* 位置模式给足行程，撞限位由电流增量判定 */
    motion_active = true;
    status = (axis == 0U) ? CoreXY_MoveTo(0.0f, -10000.0f, 70U, 20U) :
                            CoreXY_MoveTo(-10000.0f, 0.0f, 70U, 20U);
    if (status != HAL_OK)
    {
      corexy_origin_error_message = "ERR ORIGIN: motor command failed\r\n";
      goto cleanup;
    }

    axis_start_us = CoreXY_TimeNowUs();
    while (!stalled)
    {
      uint8_t sample_id = feedback_motor->current_sample_id;
      uint32_t reply_start_us = CoreXY_TimeNowUs();

      status = StepperMotor_ReadCurrent(feedback_motor);
      if (status != HAL_OK)
      {
        corexy_origin_error_message = "ERR ORIGIN: current query failed\r\n";
        goto cleanup;
      }

      while (feedback_motor->current_sample_id == sample_id)
      {
        if ((CoreXY_TimeNowUs() - reply_start_us) >=
            (reply_timeout_ms * 1000U))
        {
          status = HAL_TIMEOUT;
          corexy_origin_error_message =
              "ERR ORIGIN: no current reply from motor B (USART2)\r\n";
          CoreXY_ReportMotorRxDiagnostic(feedback_motor, "B", "USART2");
          goto cleanup;
        }
        osDelay(1U);
      }

      if (!baseline_ready)
      {
        baseline_current = feedback_motor->current_ma;
        baseline_ready = true;
      }
      else if (((int32_t)feedback_motor->current_ma -
                (int32_t)baseline_current) >= (int32_t)threshold)
      {
        if (over_threshold < required_samples)
        {
          over_threshold++;
        }
        stalled = (over_threshold >= required_samples);
      }
      else
      {
        over_threshold = 0U;
      }

      if (!stalled &&
          ((CoreXY_TimeNowUs() - axis_start_us) >= (axis_timeout_ms * 1000U)))
      {
        status = HAL_TIMEOUT;
        corexy_origin_error_message =
            "ERR ORIGIN: current threshold not reached\r\n";
        goto cleanup;
      }
      if (!stalled)
      {
        osDelay(poll_ms);
      }
    }

    /* 撞停即停两台电机，再解除堵转保护并清零驱动器位置，绝对坐标才能继续使用 */
    CoreXY_StopAll();
    motion_active = false;
    for (i = 0U; i < 2U; ++i)
    {
      Emm_V5_Reset_Clog_Pro(motors[i]->huart);
      osDelay(command_gap_ms);
      Emm_V5_Reset_CurPos_To_Zero(motors[i]->huart);
      osDelay(command_gap_ms);
      motors[i]->position_pulses = 0;
    }

    if (axis == 0U)
    {
      /* 撞停点即 Y 原点；回退 15mm 脱开硬限位，避免后续 X 运动被限位卡住 */
      corexy_target_y = 0.0f;
      
      status = CoreXY_MoveTo(0.0f, 15.0f, 30U, 20U);
      if (status != HAL_OK)
      {
        corexy_origin_error_message = "ERR ORIGIN: motor command failed\r\n";
        goto cleanup;
      }
      osDelay(1500U);
      
      /* 回退后的位置重新记为 Y 原点，保证下一步是纯 X 运动 */
      corexy_target_y = 0.0f;
    }
    else
    {
      corexy_target_x = 0.0f;
    }
  }

cleanup:
  if (motion_active)
  {
    CoreXY_StopAll();
  }
  return status;
}
  /************AICodeEnd1************/
/**
 * @brief CoreXY 电机任务入口。
 * @details
 * - 设计意图：为 CoreXY 电机控制保留独立 FreeRTOS 任务入口。
 * - 实现要点：上电先做一次碰撞回零（先 Y 后 X）；失败时在 UART1 报告具体原因，
 *   不自动重试，避免反复顶限位；之后进入 1s 周期的空循环，等待后续命令接入。
 * - 注意事项：回零是阻塞调用（两轴各有 20s 安全超时），期间本任务不处理其它请求。
 * @param argument 未使用。
 */
void CoreXYTask(void *argument)
{
  

Grab_To_Zero();
osDelay(1000U);
  HAL_StatusTypeDef origin_status;

  (void)argument;


  /* 上电用 USART2 单侧电流反馈回零一次（先 Y 后 X）；
     失败只报一行，不自动重试，避免反复顶限位 */
  origin_status = CoreXY_MoveOriginUSART2();
  if (origin_status != HAL_OK)
  {
    Uart1Proto_Send((corexy_origin_error_message != NULL) ?
                    corexy_origin_error_message :
                    "ERR ORIGIN: unknown failure\r\n");
  }





//CoreXY_MoveTo(Destination_X, Destination_Y, 100U, 50U);//（95 255盘子坐标）  25    Y轴（0-260）  X轴（0-190 水泵不在摄像头一侧） 

//WaterPump_Stop();

//Grab_Begin();
Grab_XY(120,30 , (uint16_t)(Destination_X - 6U), (uint16_t)(Destination_Y + 6U));
///StepperMotor_SetSpeed(&stepper_motor_b2, 0, 100,10, false);
//osDelay(1000U);
//StepperMotor_Stop(&stepper_motor_b2);

  /* 上电自检：抓取电机以 200RPM 正转 5s 后停机并把当前位置记为 0；
     内部用 osDelay，只挂起本任务，不会像 HAL_Delay 那样卡住整个系统 */
  
//Grab_Begin();
//osDelay(1000U);
//Grab_End();
  for (;;)
  {
   
    osDelay(5000U);
   
  }
}

/**
 * @brief 获取当前 CoreXY 状态。
 * @details 直接返回 volatile 状态变量，供 UART1 STATUS 命令读取。
 * @retval 状态枚举。
 */
CoreXY_State_t CoreXY_GetState(void)
{
  return corexy_state;
}

/**
 * @brief 获取当前 CoreXY 状态的文本表示。
 * @details 返回静态常量字符串，调用方无需释放；用于 UART1 文本协议。
 * @retval 静态字符串。
 */
const char *CoreXY_GetStateString(void)
{
  switch (corexy_state)
  {
    case COREXY_STATE_IDLE:
      return "IDLE";
    case COREXY_STATE_INIT_FAIL:
      return "INIT_FAIL";
    default:
      return "UNKNOWN";
  }
}
