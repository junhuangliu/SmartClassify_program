#include "Grab.h"
#include <math.h>
#include <stdio.h>

/* 电机每转的皮带行程：20 齿 × 2mm 齿距 = 40mm */
#define GRAB_BELT_MM_PER_REV \
  (COREXY_BELT_TOOTH_PITCH_MM * COREXY_PULLEY_TEETH)

/**
 * @brief 按行程估算一段 CoreXY 运动需要等待的时间
 * @details 详细思路：
 * - 设计意图：CoreXY_MoveTo 只下发位置命令就返回，两段运动之间必须等前一段走完；
 *   固定 500ms 在长行程时会提前覆盖目标，短行程又白等，所以按实际距离换算
 * - 实现要点：与 CoreXY_Move 的取速方式保持一致——两轴同时运动时按最长的那条皮带行程
 *   计时（|dx+dy| 与 |dx-dy| 取大者），再按 速度(RPM) × 每转皮带行程 折算成毫秒，
 *   最后乘系数 K 补偿加速度段和驱动器启动延时
 * - 数据流：dx/dy(mm，由 CoreXY_GetPositionX/Y 与目标点相减得到) -> 最长皮带行程(mm) -> 时间(ms)
 * - 注意事项：只是估算值；K=1 为匀速理论时间，实际含加减速，通常取 1.2~1.5
 *
 * @param dx X 方向位移，单位 mm
 * @param dy Y 方向位移，单位 mm
 * @param speed_rpm 该段使用的最高速度，单位 RPM
 * @param K 时间系数，1.0 表示理论匀速时间
 * @retval 估算的等待时间，单位 ms；参数非法或零位移时返回 0
 */
static uint32_t Grab_MoveTimeMs(float dx, float dy, uint16_t speed_rpm, float K)
{
  float max_belt = fmaxf(fabsf(dx + dy), fabsf(dx - dy));
  float time_ms;

  if ((speed_rpm == 0U) || !isfinite(max_belt) || !(K > 0.0f))
  {
    return 0U;
  }

  /* mm ÷ (RPM × mm/转) = 分钟，再 × 60000 换成毫秒 */
  time_ms = (K * max_belt * 60000.0f) /
            ((float)speed_rpm * GRAB_BELT_MM_PER_REV);
  return (uint32_t)(time_ms + 0.5f);
}

/**
 * @brief 初始化抓取机构步进电机通信。
 * @details 详细思路：
 * - 设计意图：把 UART6 的驱动初始化收敛到一个入口，调用方只需看返回值。
 * - 实现要点：先使能 USART6 全局中断（HAL 的 DMA 发送完成流程靠 TC 中断把
 *   huart6.gState 从 BUSY_TX 恢复为 READY，缺了它 StepperMotor_WaitTx 必然超时），
 *   再调用 StepperMotor_Init 绑定电机对象并启动应答接收。
 * - 注意事项：本函数在 osKernelStart() 之前调用，调度器/节拍尚未正常运行，
 *   因此这里绝不能用 HAL_Delay/osDelay 做上电自检——那会卡死启动流程，
 *   自检请用 Grab_PowerOnTest() 在任务里做。
 *
 * @retval HAL 状态码；UART6 未配置 RX DMA 或接收启动失败时返回对应错误状态。
 */
HAL_StatusTypeDef Grab_Init(void)
{
  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);

  return StepperMotor_Init(&stepper_motor_a2, &huart6);
}

/**
 * @brief 抓取机构上电自检：只执行一次，让 stepper_motor_a2 一直转，读到堵转标志后
 *        停机、清除标志并把驱动器位置记为 0。
 *
 * @details 详细思路：
 * - 设计意图：抓取机构没有限位开关，行程终点只能靠撞到死点判断；原先"固定 200RPM 转 5s"
 *   既可能还没撞到、也可能一直顶着死点，改成让驱动器自己的堵转标志决定何时收工，
 *   同时把撞停点当作后续相对动作的零点。
 * - 实现要点：
 *   1) static 标志位保证整机上电只执行一次，重复调用直接返回，调用方不用改；
 *   2) 起转前先发 0x0E 解除堵转保护：驱动器会锁存上次的堵转标志，不清掉会一上电
 *      就被旧标志判成已经撞停；
 *   3) 速度模式 200RPM 持续运行，中间不发停止命令，让电机一直转到撞停为止；
 *   4) 轮询 0x3A 状态标志：Emm_V5_Read_Sys_Params(S_FLAG) 下发查询，应答由 USART6
 *      空闲中断收进 stepper_motor_a2 的原始接收快照，这里直接在快照里找
 *      "地址 0x01 + 功能码 0x3A + ... + 0x6B"，把校验字节前一个字节当作标志字节，
 *      判 (标志 & 0x0C) != 0（bit2 Cgi_TF 堵转标志、bit3 Cgp_TF 堵转保护）；
 *   5) 撞停后按 0xFE 急停 -> 0x0E 清标志解保护 -> 0x0A 位置清零 的顺序收尾：
 *      保护状态下驱动器对所有命令回 E2，必须先解保护才能再发位置/速度命令。
 * - 数据流：USART6 -> 0xF6 速度命令 -> 0x3A 状态查询 -> 原始应答帧 -> 堵转判定
 *   -> 0xFE 停止 -> 0x0E 清标志 -> 0x0A 位置清零。
 * - 注意事项：
 *   * 阻塞执行（内部 osDelay），只能在 FreeRTOS 任务上下文调用，禁止在 Grab_Init 里调；
 *   * 起转时转速要穿过驱动器的"堵转检测转速"，因此起步后 grace_ms 内不判标志，
 *     避开加速斜坡上的误触发；标志在驱动器里是锁存的，过了窗口照样能读到；
 *   * 单次查询应答超时只放弃本轮，整体 test_timeout_ms 兜底；无论撞停还是超时，
 *     退出前一定先停机，绝不把电机留在自转状态；
 *   * 驱动器应答有 4 字节（地址+功能码+标志+校验）和 5 字节（多一个命令状态字节）
 *     两种格式，统一取"校验字节前一个字节"当标志，两种都能解析；
 *   * 只读 stepper_motor_a2 已有的接收快照字段，不改电机模块的帧解析逻辑；
 *   * 速度命令失败时先补发一次再放弃：StepperMotor_WaitTx 失败只代表"等待发送完成
 *     超时"，命令本身可能已发出，直接 return 会让电机静默不动作；
 *   * 全程把关键状态（速度命令返回值、UART gState、标志字节、应答超时）打印到 UART1，
 *     实机不动作时可直接看出卡在哪一步。
 *
 * @param 无
 * @retval 无
 */
void Grab_PowerOnTest(void)
{
  static bool test_done = false;
  const uint8_t test_direction = 1U;
  const uint16_t test_speed_rpm = 200U;
  const uint8_t test_acceleration = 50U;
  const uint8_t stall_flag_mask = 0x0CU;
  const uint8_t frame_end = 0x6BU;
  const uint8_t cmd_status_flag = 0x3AU;
  const uint32_t grace_ms = 500U;
  const uint32_t reply_timeout_ms = 100U;
  const uint32_t test_timeout_ms = 20000U;
  const uint32_t poll_ms = 20U;
  const uint32_t command_gap_ms = 5U;
  uint32_t start_tick;
  uint32_t reply_miss = 0U;
  HAL_StatusTypeDef speed_status;
  char msg[64];
  uint8_t last_flag = 0U;
  uint8_t reported_flag = 0xFFU;
  bool stalled = false;

  if (test_done)
  {
    return;
  }
  /* 先置位：即使中途失败也不重复自检，避免反复顶死点或一直空转 */
  test_done = true;

  (void)snprintf(msg, sizeof(msg),
                 "GRAB TEST: enter gState=%d rxStart=%d rxErr=%lu\r\n",
                 (int)huart6.gState, (int)stepper_motor_a2.rx_start_status,
                 (unsigned long)stepper_motor_a2.rx_error_count);
  Uart1Proto_Send(msg);

  Emm_V5_Reset_Clog_Pro(&huart6);
  osDelay(command_gap_ms);

  speed_status = StepperMotor_SetSpeed(&stepper_motor_a2, test_direction,
                                       test_speed_rpm, test_acceleration, false);
  if (speed_status != HAL_OK)
  {
    /* 启动最早的一次发送偶发超时，隔一段再补发一次；命令没发出去补发是安全的 */
    osDelay(command_gap_ms);
    speed_status = StepperMotor_SetSpeed(&stepper_motor_a2, test_direction,
                                         test_speed_rpm, test_acceleration, false);
  }
  (void)snprintf(msg, sizeof(msg), "GRAB TEST: speed st=%d gState=%d\r\n",
                 (int)speed_status, (int)huart6.gState);
  Uart1Proto_Send(msg);
  if (speed_status != HAL_OK)
  {
    /* 速度命令可能已经发出去了（只是等待发送完成超时），补一次停机 */
    (void)StepperMotor_Stop(&stepper_motor_a2);
    Uart1Proto_Send("GRAB TEST: abort, speed cmd not sent\r\n");
    return;
  }

  start_tick = HAL_GetTick();
  while (!stalled)
  {
    uint32_t tx_start = HAL_GetTick();
    uint32_t reply_start;
    uint32_t event_id;
    uint16_t length;
    uint16_t i;

    /* Emm_V5 的查询命令共用静态发送缓冲区，上一条没发完不能重装，否则会发半条命令 */
    while (huart6.gState == HAL_UART_STATE_BUSY_TX)
    {
      if ((HAL_GetTick() - tx_start) >= reply_timeout_ms)
      {
        break;
      }
      osDelay(1U);
    }

    event_id = stepper_motor_a2.rx_event_count;
    if (Emm_V5_Read_Sys_Params(&huart6, S_FLAG) != HAL_OK)
    {
      break;
    }

    reply_start = HAL_GetTick();
    while (stepper_motor_a2.rx_event_count == event_id)
    {
      if ((HAL_GetTick() - reply_start) >= reply_timeout_ms)
      {
        break;
      }
      osDelay(1U);
    }

    if (stepper_motor_a2.rx_event_count == event_id)
    {
      if (reply_miss == 0U)
      {
        Uart1Proto_Send("GRAB TEST: no flag reply\r\n");
      }
      reply_miss++;
    }
    else
    {
      length = stepper_motor_a2.rx_last_length;
      if ((length >= 4U) &&
          (stepper_motor_a2.rx_last_data[length - 1U] == frame_end))
      {
        for (i = 0U; (i + 2U) < length; ++i)
        {
          if ((stepper_motor_a2.rx_last_data[i] == STEPPER_MOTOR_ADDRESS) &&
              (stepper_motor_a2.rx_last_data[i + 1U] == cmd_status_flag))
          {
            last_flag = stepper_motor_a2.rx_last_data[length - 2U];
            if (last_flag != reported_flag)
            {
              (void)snprintf(msg, sizeof(msg), "GRAB TEST: flag=0x%02X\r\n",
                             (unsigned int)last_flag);
              Uart1Proto_Send(msg);
              reported_flag = last_flag;
            }
            /* 起步斜坡阶段转速低于堵转检测转速，这段窗口内的标志不可信 */
            stalled = (((last_flag & stall_flag_mask) != 0U) &&
                       ((HAL_GetTick() - start_tick) >= grace_ms));
            break;
          }
        }
      }
    }

    if (!stalled)
    {
      if ((HAL_GetTick() - start_tick) >= test_timeout_ms)
      {
        break;
      }
      osDelay(poll_ms);
    }
  }

  (void)snprintf(msg, sizeof(msg), "GRAB TEST: %s after %lums, flag=0x%02X, miss=%lu\r\n",
                 stalled ? "stall" : "timeout",
                 (unsigned long)(HAL_GetTick() - start_tick),
                 (unsigned int)last_flag, (unsigned long)reply_miss);
  Uart1Proto_Send(msg);

  (void)StepperMotor_Stop(&stepper_motor_a2);
  osDelay(command_gap_ms);
  /* 保护状态下驱动器对后续命令一律回 E2，必须先清掉堵转标志才能继续动作 */
  Emm_V5_Reset_Clog_Pro(&huart6);
  osDelay(command_gap_ms);
  Emm_V5_Reset_CurPos_To_Zero(&huart6);
  osDelay(command_gap_ms);
  stepper_motor_a2.position_pulses = 0;
}

/**
 * @brief USART6 全局中断入口，负责分发抓取电机的发送完成与应答事件。
 *
 * @details 详细思路：
 * - 设计意图：CubeMX 未使能 USART6 全局中断，而 HAL 的 DMA 发送流程必须靠 TC 中断
 *   把 gState 从 BUSY_TX 恢复为 READY，否则 StepperMotor_WaitTx 全部超时——表现为
 *   命令发出去后电机一直转、后续 Stop 命令再也发不出来。空闲中断的 IDLE 分支同样
 *   在这里分发，用于解析驱动器应答。
 * - 实现要点：直接转给 HAL_UART_IRQHandler，业务逻辑全部复用电机模块，本函数不写业务代码。
 * - 注意事项：中断上下文，禁止阻塞；若后续在 CubeMX 中勾选 USART6 global interrupt，
 *   需删除本函数，避免与 stm32f4xx_it.c 生成的定义重复。
 *
 * @param 无
 * @retval 无
 */
void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}
void Grab_To_Zero(){
  Emm_V5_Reset_Clog_Pro(&huart6);
if(StepperMotor_SetSpeed(&stepper_motor_a2, 0, 200,50, false) != HAL_OK)
   {
     /* 速度命令可能已经发出去了（只是等待发送完成超时），这里补一次停机，
        避免函数提前返回后电机一直自转 */
     StepperMotor_Stop(&stepper_motor_a2);
     return;
   }
osDelay(5000);
StepperMotor_Stop(&stepper_motor_a2);
Emm_V5_Reset_Clog_Pro(&huart6);
StepperMotor_Stop(&stepper_motor_a2);
}
void Grab_Begin()
{
 
 uint16_t time=4250;

   if(StepperMotor_SetSpeed(&stepper_motor_a2, 0, 200,50, false) != HAL_OK)
   {
     /* 速度命令可能已经发出去了（只是等待发送完成超时），这里补一次停机，
        避免函数提前返回后电机一直自转 */
     StepperMotor_Stop(&stepper_motor_a2);
     return;
   }
   osDelay(time);
  StepperMotor_Stop(&stepper_motor_a2);
  WaterPump_Start();
  osDelay(200U);
   if(StepperMotor_SetSpeed(&stepper_motor_a2, 1, 200,50, false) != HAL_OK)
   {
     /* 速度命令可能已经发出去了（只是等待发送完成超时），这里补一次停机，
        避免函数提前返回后电机一直自转 */
     StepperMotor_Stop(&stepper_motor_a2);
     return;
   }
   osDelay(time);
StepperMotor_Stop(&stepper_motor_a2);




}
void Grab_End()
{


   WaterPump_Stop();

   }
void Grab_XY(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
   /* K 为时间系数：1.0 = 按 100RPM 的理论匀速时间，加减速明显时调大即可 */
   const float K=1.9f; const uint16_t speed_rpm=100U;  //1.9-2.1  
   const uint8_t accel=40U;
   /* 当前位置直接取 CoreXY 模块跟踪的规划坐标，不再在这里另存一份 */
   float dx=(float)x1-CoreXY_GetPositionX();
   float dy=(float)y1-CoreXY_GetPositionY();

   CoreXY_MoveTo(x1,y1,speed_rpm,accel);
   osDelay(Grab_MoveTimeMs(dx,dy,speed_rpm,K));

   Grab_Begin();

   dx=(float)x2-CoreXY_GetPositionX();
   dy=(float)y2-CoreXY_GetPositionY();
   CoreXY_MoveTo(x2,y2,speed_rpm,accel);
   osDelay(Grab_MoveTimeMs(dx,dy,speed_rpm,K));

   Grab_End();
   }

