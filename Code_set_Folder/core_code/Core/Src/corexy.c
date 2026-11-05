#include "corexy.h"
#include "cmsis_os2.h"
#include "uart1_proto.h"
#include <limits.h>
#include <math.h>
#include <stdint.h>

#define COREXY_TX_TIMEOUT_MS 100U

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static float corexy_target_x;
static float corexy_target_y;
static volatile CoreXY_State_t corexy_state = COREXY_STATE_IDLE;
static volatile uint8_t corexy_abort_requested;
static osMessageQueueId_t corexy_cmd_queue;

/**
 * @brief 等待指定电机的 DMA 发送完成。
 * @details
 * - 设计意图：EMM_V5 库使用静态共享发送缓冲区，上一条命令未发完就发下一条会被覆盖。
 * - 实现要点：轮询 UART 的 gState，超时时间与原有 CoreXY 逻辑保持一致；等待期间让出 CPU。
 * - 注意事项：本函数可在任务上下文调用，不能在中断中调用。
 * @param motor 电机对象。
 * @retval HAL_OK 发送完成；HAL_TIMEOUT 等待超时。
 */
static HAL_StatusTypeDef CoreXY_WaitTx(StepperMotor_t *motor)
{
  uint32_t start = HAL_GetTick();

  while (motor->huart->gState == HAL_UART_STATE_BUSY_TX)
  {
    if ((HAL_GetTick() - start) >= COREXY_TX_TIMEOUT_MS)
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
 * - 设计意图：运动命令失败、回零失败或用户中止时，必须让两个电机同时停下来，避免机构继续挤压。
 * - 实现要点：逐路发送 EMM_V5 立即停止命令；StepperMotor_Stop 内部已等待 DMA 发送完成。
 * - 注意事项：会占用两路 UART 发送通道，只应由单一控制任务调用。
 */
static void CoreXY_StopAll(void)
{
  (void)StepperMotor_Stop(&stepper_motor_a1);
  (void)StepperMotor_Stop(&stepper_motor_b1);
}

/**
 * @brief 撤销 A/B 两路回零并立即停止。
 * @details
 * - 设计意图：碰撞回零失败或超时后，先中断回零过程，再发送停止命令，防止持续顶住硬限位。
 * - 实现要点：A/B 依次执行 HomeAbort；每路内部都会等待共享 DMA 缓冲区空闲。
 * - 注意事项：仅由正在执行回零的 CoreXYTask 调用，避免与状态轮询并发操作同一 UART。
 */
static void CoreXY_AbortHoming(void)
{
  (void)StepperMotor_HomeAbort(&stepper_motor_a1);
  (void)StepperMotor_HomeAbort(&stepper_motor_b1);
}

/**
 * @brief 对两电机下发一组同步位置命令。
 * @details
 * - 设计意图：CoreXY 逆解为 A=dx+dy、B=dx-dy；两路必须先缓存位置命令，再一起发同步触发，
 *   否则电机会先后启动，导致轨迹歪斜。
 * - 实现要点：按两路中较大的位移分配速度，保证两路同时到达；脉冲数四舍五入到整数。
 *   任一路发送失败立即停止两电机，避免只动一侧。
 * - 数据流：dx/dy（mm）-> 皮带位移 -> 脉冲数/方向/速度 -> EMM_V5 位置命令 -> 同步触发。
 * - 注意事项：函数会等待 DMA 发送完成，必须在 FreeRTOS 任务中调用；回零期间拒绝运动。
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
  uint32_t start;
  HAL_StatusTypeDef status;
  uint8_t i;

  if ((speed > 5000U) || ((speed == 0U) && ((dx != 0.0f) || (dy != 0.0f))) ||
      !isfinite(dx) || !isfinite(dy))
  {
    return HAL_ERROR;
  }

  if (corexy_state == COREXY_STATE_HOMING)
  {
    return HAL_BUSY;
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
        CoreXY_StopAll();
        return status;
      }
    }
  }

  start = HAL_GetTick();
  for (i = 0U; i < 2U; ++i)
  {
    if (pulses[i] != 0U)
    {
      while (motors[i]->huart->gState == HAL_UART_STATE_BUSY_TX)
      {
        if ((HAL_GetTick() - start) >= COREXY_TX_TIMEOUT_MS)
        {
          CoreXY_StopAll();
          return HAL_TIMEOUT;
        }
        osDelay(1U);
      }
    }
  }

  corexy_target_x += dx;
  corexy_target_y += dy;
  return HAL_OK;
}

/**
 * @brief 执行单段碰撞回零。
 * @details
 * - 设计意图：把一段回零（X 或 Y）的参数配置、同步触发和 0x3B 状态轮询封装在一起，
 *   两段共用同一套超时与失败处理。
 * - 实现要点：先配置 A/B 回零参数，再缓存触发命令并同步触发；随后周期查询 0x3B，
 *   必须看到“正在回零”后才接受“回零结束”，避免命令尚未生效被误判为完成。
 * - 注意事项：状态轮询要占用 UART，本函数只由 CoreXYTask 调用；中止标志在轮询点检查。
 * @param dir_a A 电机回零方向。
 * @param dir_b B 电机回零方向。
 * @retval HAL_OK 两电机均完成；HAL_TIMEOUT 超时；HAL_ERROR 失败或中止。
 */
static HAL_StatusTypeDef CoreXY_HomePhase(uint8_t dir_a, uint8_t dir_b)
{
  HAL_StatusTypeDef status;
  uint32_t start;
  uint8_t seen_a = 0U;
  uint8_t seen_b = 0U;
  uint8_t done_a = 0U;
  uint8_t done_b = 0U;

  status = StepperMotor_HomeSetParams(&stepper_motor_a1, COREXY_HOME_MODE, dir_a,
                                      COREXY_HOME_O_VEL, COREXY_HOME_O_TM,
                                      COREXY_HOME_SL_VEL, COREXY_HOME_SL_MA,
                                      COREXY_HOME_SL_MS, false);
  if (status != HAL_OK)
  {
    return status;
  }

  status = StepperMotor_HomeSetParams(&stepper_motor_b1, COREXY_HOME_MODE, dir_b,
                                      COREXY_HOME_O_VEL, COREXY_HOME_O_TM,
                                      COREXY_HOME_SL_VEL, COREXY_HOME_SL_MA,
                                      COREXY_HOME_SL_MS, false);
  if (status != HAL_OK)
  {
    return status;
  }

  StepperMotor_ClearHomingStatus(&stepper_motor_a1);
  StepperMotor_ClearHomingStatus(&stepper_motor_b1);

  status = StepperMotor_HomeTrigger(&stepper_motor_a1, COREXY_HOME_MODE, true);
  if (status != HAL_OK)
  {
    return status;
  }

  status = StepperMotor_HomeTrigger(&stepper_motor_b1, COREXY_HOME_MODE, true);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Emm_V5_Synchronous_motion(stepper_motor_a1.huart);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Emm_V5_Synchronous_motion(stepper_motor_b1.huart);
  if (status != HAL_OK)
  {
    return status;
  }

  status = CoreXY_WaitTx(&stepper_motor_a1);
  if (status != HAL_OK)
  {
    return status;
  }
  status = CoreXY_WaitTx(&stepper_motor_b1);
  if (status != HAL_OK)
  {
    return status;
  }

  osDelay(COREXY_HOME_START_DELAY_MS);
  start = HAL_GetTick();

  for (;;)
  {
    if (corexy_abort_requested)
    {
      return HAL_ERROR;
    }

    if ((HAL_GetTick() - start) >= COREXY_HOME_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }

    if (!done_a)
    {
      StepperMotor_ClearHomingStatus(&stepper_motor_a1);
      status = StepperMotor_ReadHomingStatus(&stepper_motor_a1);
      if (status != HAL_OK)
      {
        return status;
      }
    }

    if (!done_b)
    {
      StepperMotor_ClearHomingStatus(&stepper_motor_b1);
      status = StepperMotor_ReadHomingStatus(&stepper_motor_b1);
      if (status != HAL_OK)
      {
        return status;
      }
    }

    osDelay(COREXY_HOME_POLL_MS);

    if (!done_a && ((stepper_motor_a1.valid_data & STEPPER_MOTOR_HOMING_VALID) != 0U))
    {
      if ((stepper_motor_a1.homing_status & STEPPER_MOTOR_OFLAG_FAILED) != 0U)
      {
        return HAL_ERROR;
      }
      if ((stepper_motor_a1.homing_status & STEPPER_MOTOR_OFLAG_RUNNING) != 0U)
      {
        seen_a = 1U;
      }
      else if (seen_a != 0U)
      {
        done_a = 1U;
      }
    }

    if (!done_b && ((stepper_motor_b1.valid_data & STEPPER_MOTOR_HOMING_VALID) != 0U))
    {
      if ((stepper_motor_b1.homing_status & STEPPER_MOTOR_OFLAG_FAILED) != 0U)
      {
        return HAL_ERROR;
      }
      if ((stepper_motor_b1.homing_status & STEPPER_MOTOR_OFLAG_RUNNING) != 0U)
      {
        seen_b = 1U;
      }
      else if (seen_b != 0U)
      {
        done_b = 1U;
      }
    }

    if ((seen_a == 0U) && (seen_b == 0U) &&
        ((HAL_GetTick() - start) >= COREXY_HOME_START_TIMEOUT_MS))
    {
      return HAL_ERROR;
    }

    if ((done_a != 0U) && (done_b != 0U))
    {
      return HAL_OK;
    }
  }
}

/**
 * @brief 初始化两电机 CoreXY 并创建回零命令队列。
 * @details
 * - 设计意图：把 A/B 两路绑定到 USART2/USART3，并提前创建队列，供 UART1 命令任务投递回零请求。
 * - 实现要点：两路都调用 StepperMotor_Init 打开空闲中断 + DMA 接收；任一路失败都记录为初始化失败。
 * - 注意事项：在 osKernelStart 之前调用也能创建 FreeRTOS 队列；重复调用不会重复创建队列。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef CoreXY_Init(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (StepperMotor_Init(&stepper_motor_a1, COREXY_MOTOR_A_UART_HANDLE) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  if (StepperMotor_Init(&stepper_motor_b1, COREXY_MOTOR_B_UART_HANDLE) != HAL_OK)
  {
    status = HAL_ERROR;
  }

  if (corexy_cmd_queue == NULL)
  {
    corexy_cmd_queue = osMessageQueueNew(4U, sizeof(CoreXY_Cmd_t), NULL);
  }
  if (corexy_cmd_queue == NULL)
  {
    status = HAL_ERROR;
  }

  corexy_target_x = 0.0f;
  corexy_target_y = 0.0f;
  corexy_abort_requested = 0U;
  corexy_state = (status == HAL_OK) ? COREXY_STATE_IDLE : COREXY_STATE_INIT_FAIL;
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

/**
 * @brief 请求执行碰撞回零。
 * @details
 * - 设计意图：UART1 命令处理必须快速返回，不能阻塞数秒；这里只负责检查状态并把命令入队。
 * - 实现要点：先把状态置为 HOMING 预留执行权，再入队；入队失败则恢复原状态。
 * - 注意事项：重复的 init:1 会因状态为 HOMING 而返回 HAL_BUSY。
 * @retval HAL_OK 已入队；HAL_BUSY 正在回零或队列满；HAL_ERROR 未初始化。
 */
HAL_StatusTypeDef CoreXY_RequestHome(void)
{
  CoreXY_Cmd_t cmd = COREXY_CMD_HOME;
  CoreXY_State_t previous_state;

  if (corexy_state == COREXY_STATE_INIT_FAIL)
  {
    return HAL_ERROR;
  }
  if ((corexy_state == COREXY_STATE_HOMING) || (corexy_cmd_queue == NULL))
  {
    return HAL_BUSY;
  }

  previous_state = corexy_state;
  corexy_state = COREXY_STATE_HOMING;
  if (osMessageQueuePut(corexy_cmd_queue, &cmd, 0U, 0U) != osOK)
  {
    corexy_state = previous_state;
    return HAL_BUSY;
  }

  return HAL_OK;
}

/**
 * @brief 请求中止回零或停止电机。
 * @details
 * - 设计意图：init:0 需要在不与回零任务抢 UART 的前提下尽快生效。
 * - 实现要点：回零中只置中止标志，由回零任务在轮询点撤销并停止；非回零状态直接停止两电机。
 * - 注意事项：中止标志为 volatile，回零任务每轮都会检查。
 * @retval HAL_OK。
 */
HAL_StatusTypeDef CoreXY_RequestAbort(void)
{
  if ((stepper_motor_a1.huart == NULL) || (stepper_motor_b1.huart == NULL))
  {
    return HAL_ERROR;
  }

  corexy_abort_requested = 1U;

  if (corexy_state != COREXY_STATE_HOMING)
  {
    CoreXY_StopAll();
    corexy_abort_requested = 0U;
    corexy_state = COREXY_STATE_IDLE;
  }

  return HAL_OK;
}

/**
 * @brief 阻塞等待一条 CoreXY 命令。
 * @details
 * - 设计意图：让 CoreXYTask 以阻塞方式等待命令，不占用 CPU 轮询。
 * - 实现要点：队列未创建时延时返回 NONE，避免任务空转或解引用空句柄。
 * - 注意事项：只能在 FreeRTOS 任务中调用。
 * @retval 收到的命令；队列不可用时返回 COREXY_CMD_NONE。
 */
CoreXY_Cmd_t CoreXY_WaitCommand(void)
{
  CoreXY_Cmd_t cmd = COREXY_CMD_NONE;

  if (corexy_cmd_queue == NULL)
  {
    osDelay(1000U);
    return cmd;
  }

  (void)osMessageQueueGet(corexy_cmd_queue, &cmd, NULL, osWaitForever);
  return cmd;
}

/**
 * @brief 执行两段式碰撞回零（先 X 后 Y）。
 * @details
 * - 设计意图：X/Y 都有硬限位，使用 EMM_V5 模式 2 靠电流/速度检测碰撞，不需要限位开关。
 * - 实现要点：先 X 轴 A/B 同向撞 X 限位，再 Y 轴 A/B 反向撞 Y 限位；任一段失败立即撤销两路回零并停止。
 * - 数据流：0x3B 状态 bit2=1 表示正在回零，bit3=1 表示失败；两路都完成后软件坐标置 (0,0)。
 * - 注意事项：本函数阻塞数秒，只能在 CoreXYTask 中调用；中止标志在 HomePhase 轮询点检查。
 * @retval HAL_OK 成功；HAL_TIMEOUT 超时；HAL_ERROR 失败或中止。
 */
HAL_StatusTypeDef CoreXY_HomeTorque(void)
{
  HAL_StatusTypeDef status;

  if ((corexy_state == COREXY_STATE_INIT_FAIL) ||
      (stepper_motor_a1.huart == NULL) || (stepper_motor_b1.huart == NULL))
  {
    return HAL_ERROR;
  }
  if (corexy_abort_requested)
  {
    corexy_abort_requested = 0U;
    corexy_state = COREXY_STATE_IDLE;
    return HAL_ERROR;
  }

  corexy_state = COREXY_STATE_HOMING;
  corexy_abort_requested = 0U;

  status = CoreXY_HomePhase(COREXY_HOME_X_DIR_A, COREXY_HOME_X_DIR_B);
  if (status == HAL_OK)
  {
    if (corexy_abort_requested)
    {
      status = HAL_ERROR;
    }
    else
    {
      status = CoreXY_HomePhase(COREXY_HOME_Y_DIR_A, COREXY_HOME_Y_DIR_B);
    }
  }

  if (status == HAL_OK)
  {
    corexy_target_x = 0.0f;
    corexy_target_y = 0.0f;
    corexy_state = COREXY_STATE_READY;
  }
  else
  {
    CoreXY_AbortHoming();
    corexy_state = corexy_abort_requested ? COREXY_STATE_IDLE : COREXY_STATE_ERROR;
  }

  corexy_abort_requested = 0U;
  return status;
}

/**
 * @brief CoreXY 回零任务入口。
 * @details
 * - 设计意图：把阻塞数秒的回零过程从 UART1 命令接收任务中隔离出来。
 * - 实现要点：永久等待命令队列；收到 HOME 后执行两段式碰撞回零，并用 UART1 回复结果。
 * - 注意事项：Uart1Proto_Send 内部有互斥锁，可与其他任务并发调用。
 * @param argument 未使用。
 */
void CoreXYTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    CoreXY_Cmd_t cmd = CoreXY_WaitCommand();

    if (cmd == COREXY_CMD_HOME)
    {
      HAL_StatusTypeDef status = CoreXY_HomeTorque();

      if (status == HAL_OK)
      {
        Uart1Proto_Send("OK HOME DONE\r\n");
      }
      else if (status == HAL_TIMEOUT)
      {
        Uart1Proto_Send("ERR HOME TIMEOUT\r\n");
      }
      else
      {
        Uart1Proto_Send("ERR HOME\r\n");
      }
    }
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
    case COREXY_STATE_HOMING:
      return "HOMING";
    case COREXY_STATE_READY:
      return "READY";
    case COREXY_STATE_ERROR:
      return "ERROR";
    case COREXY_STATE_INIT_FAIL:
      return "INIT_FAIL";
    default:
      return "UNKNOWN";
  }
}
