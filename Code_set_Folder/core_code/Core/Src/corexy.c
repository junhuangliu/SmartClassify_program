#include "corexy.h"
#include "cmsis_os2.h"
#include <limits.h>
#include <math.h>

#define COREXY_TX_TIMEOUT_MS 100U

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

static float corexy_target_x;
static float corexy_target_y;

/**
 * @brief 对四台电机下发一组同步位置命令。
 * @details 逆解为 A=dx+dy、B=dx-dy。先逐路缓存位置命令，再向所有参与电机发送 EMM_V5 同步触发，避免电机依次开始整段运动。
 * @param dx X 方向位移，单位 mm。
 * @param dy Y 方向位移，单位 mm。
 * @param speed 电机最高速度，单位 RPM。
 * @param acceleration 加速度参数。
 * @retval HAL 状态码。
 */
static HAL_StatusTypeDef CoreXY_Move(float dx, float dy, uint16_t speed,
                                     uint8_t acceleration)
{
  StepperMotor_t *motors[4] =
  {
    &stepper_motor_a1, &stepper_motor_a2,
    &stepper_motor_b1, &stepper_motor_b2
  };
  const uint8_t invert[4] =
  {
    COREXY_MOTOR_A1_INVERT_DIRECTION,
    COREXY_MOTOR_A2_INVERT_DIRECTION,
    COREXY_MOTOR_B1_INVERT_DIRECTION,
    COREXY_MOTOR_B2_INVERT_DIRECTION
  };
  float belt_move[4] = {dx + dy, dx + dy, dx - dy, dx - dy};
  float max_move = fmaxf(fabsf(belt_move[0]), fabsf(belt_move[2]));
  uint32_t pulses[4];
  uint8_t direction[4];
  uint16_t motor_speed[4];
  uint32_t start;
  HAL_StatusTypeDef status;
  uint8_t i;

  if ((speed > 5000U) || ((speed == 0U) && ((dx != 0.0f) || (dy != 0.0f))) ||
      !isfinite(dx) || !isfinite(dy))
  {
    return HAL_ERROR;
  }

  if (max_move == 0.0f)
  {
    return HAL_OK;
  }

  for (i = 0U; i < 4U; ++i)
  {
    double scaled_pulses = fabs((double)belt_move[i]) *
                           (double)COREXY_PULSES_PER_MM;
    if (scaled_pulses > ((double)UINT32_MAX - 0.5))
    {
      return HAL_ERROR;
    }
    pulses[i] = (uint32_t)(scaled_pulses + 0.5);
    direction[i] = (uint8_t)((belt_move[i] < 0.0f) ^
                             (invert[i] != 0U));
    motor_speed[i] = (pulses[i] == 0U) ? 0U :
                     (uint16_t)(speed * fabsf(belt_move[i]) / max_move + 0.5f);
    if ((pulses[i] != 0U) && (motor_speed[i] == 0U))
    {
      motor_speed[i] = 1U;
    }
  }

  for (i = 0U; i < 4U; ++i)
  {
    if (pulses[i] != 0U)
    {
      status = StepperMotor_SetPosition(motors[i], direction[i], motor_speed[i],
                                        acceleration, pulses[i], true, true);
      if (status != HAL_OK)
      {
        return status;
      }
    }
  }

  for (i = 0U; i < 4U; ++i)
  {
    if (pulses[i] != 0U)
    {
      status = Emm_V5_Synchronous_motion(motors[i]->huart);
      if (status != HAL_OK)
      {
        return status;
      }
    }
  }

  start = HAL_GetTick();
  for (i = 0U; i < 4U; ++i)
  {
    if (pulses[i] != 0U)
    {
      while (motors[i]->huart->gState == HAL_UART_STATE_BUSY_TX)
      {
        if ((HAL_GetTick() - start) >= COREXY_TX_TIMEOUT_MS)
        {
          return HAL_TIMEOUT;
        }
        osDelay(1U);
      }
    }
  }

  corexy_target_x += ((belt_move[0] < 0.0f) ? -1.0f : 1.0f) *
                     ((float)pulses[0] / COREXY_PULSES_PER_MM) * 0.5f +
                     ((belt_move[2] < 0.0f) ? -1.0f : 1.0f) *
                     ((float)pulses[2] / COREXY_PULSES_PER_MM) * 0.5f;
  corexy_target_y += ((belt_move[0] < 0.0f) ? -1.0f : 1.0f) *
                     ((float)pulses[0] / COREXY_PULSES_PER_MM) * 0.5f -
                     ((belt_move[2] < 0.0f) ? -1.0f : 1.0f) *
                     ((float)pulses[2] / COREXY_PULSES_PER_MM) * 0.5f;
  return HAL_OK;
}

/**
 * @brief 初始化四个 CoreXY 电机。
 * @details 将四个电机对象绑定到对应的 CubeMX UART 句柄，同时为每路打开空闲中断 + DMA 应答接收，
 *          最后把规划坐标设为原点。任一路 UART 缺少 RX DMA 时返回 HAL_ERROR。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_Init(void)
{
  HAL_StatusTypeDef status = StepperMotor_Init(&stepper_motor_a1, COREXY_MOTOR_A1_UART_HANDLE);

  if (StepperMotor_Init(&stepper_motor_a2, COREXY_MOTOR_A2_UART_HANDLE) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  if (StepperMotor_Init(&stepper_motor_b1, COREXY_MOTOR_B1_UART_HANDLE) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  if (StepperMotor_Init(&stepper_motor_b2, COREXY_MOTOR_B2_UART_HANDLE) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  corexy_target_x = 0.0f;
  corexy_target_y = 0.0f;
  return status;
}

/**
 * @brief 移动到绝对坐标。
 * @details 根据当前规划坐标计算相对位移，再由 CoreXY_Move 进行逆解。
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
  float dy = y - corexy_target_y;
  return CoreXY_Move(dx, dy, speed, acceleration);
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
 * @brief 返回软件坐标原点。
 * @details 使用当前规划坐标作为移动起点，逆解至 (0,0)。
 * @param speed 电机最高速度，单位 RPM。
 * @param acceleration 加速度参数。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_MoveOrigin(uint16_t speed, uint8_t acceleration)
{
  return CoreXY_MoveTo(0.0f, 0.0f, speed, acceleration);
}
