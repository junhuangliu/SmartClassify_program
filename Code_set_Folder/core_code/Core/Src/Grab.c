#include "Grab.h"
#include "servo.h"
#include  "TurnTable.h"
#include <math.h>
#include <stdio.h>

/* 常用坐标点：运行期由 UART1 的 Set,<名称>,<数值>/ 修改，MoveTo 帧可按名称引用 */
volatile int16_t Destination_X = 70;
volatile int16_t Destination_Y = 245;

/* 标定特殊点：成对表示 (X0,Y0)/(X1,Y1)/(X2,Y2) */
volatile int16_t X0 = 0;
volatile int16_t Y0 = 0;
volatile int16_t X1 = 180;
volatile int16_t Y1 = 0;
volatile int16_t X2 = 0;
volatile int16_t Y2 = 180;
volatile int16_t Wait_X = 0;
volatile int16_t Wait_Y = 0;
/* 电机每转的皮带行程：20 齿 × 2mm 齿距 = 40mm */
#define GRAB_BELT_MM_PER_REV \
  (COREXY_BELT_TOOTH_PITCH_MM * COREXY_PULLEY_TEETH)

/* 舵机1 抓取行程：下降 10 齿，吸住后回升 10 齿（Servo1_MoveTeeth 内部按 167us/齿换算） */
#define GRAB_SERVO1_TEETH         5
/* 与 Servo1_MoveTeeth 内部的 SERVO1_MOVE_SETTLE_MS 保持一致，用于和电机动作并行计时 */
#define GRAB_SERVO1_MOVE_MS       1500U
/* 舵机到位后再多等一会儿，确保完全到抓取高度后才开泵 */
#define GRAB_SUCTION_DELAY_MS     300U

/* 抓取电机单次动作时间：抓取段下降行程需要更深，比上升多走 550ms */
#define GRAB_MOTOR_DOWN_MS        5000U
#define GRAB_MOTOR_UP_MS          5000U   
/* 等待余量：GrabTask 动作结束后再多等 50ms 确保停稳 */
#define GRAB_MOTOR_WAIT_MARGIN_MS 50U
/* 放置后的附加动作：抓取电机先下降 2.5s 再上升 2.5s（仍由 GrabTask 按标志位执行） */
#define GRAB_RELEASE_DOWN_MS      2500U
#define GRAB_RELEASE_UP_MS        2500U
/* 等待 GrabTask 完成动作的超时：固定动作时长 + 余量 + 1s 容错 */
#define GRAB_MOTOR_TIMEOUT_MS \
  (GRAB_MOTOR_DOWN_MS + GRAB_MOTOR_WAIT_MARGIN_MS + 1000U)


volatile uint8_t Grab_MotorBeginFlag=0;
volatile uint8_t Grab_MotorEndFlag=0;

/* GrabTask 忙/完成标志：Grab_XY 据此等待真实动作结束，
   不再依赖固定延时，避免 GrabTask 延迟响应时电机与舵机时序错位 */
static volatile uint8_t Grab_MotorBusy=0U;
static volatile uint8_t Grab_MotorDone=1U;

/* 一次性动作时长覆盖：非 0 时 GrabTask 用该值代替 GRAB_MOTOR_DOWN_MS/UP_MS，
   执行完自动清零，不影响后续正常抓取流程 */
static volatile uint16_t Grab_MotorRunMs=0U;

/**
 * @brief 启动一次抓取电机动作
 *
 * @details
 * - 设计意图：把"等上一次动作结束 + 清完成标志 + 置方向标志"收在一个入口，
 *   保证 Grab_XY 不会在上一次动作尚未停止时覆盖标志
 * - 实现要点：先等待 Grab_MotorBusy 清零，再清 Grab_MotorDone，最后按方向置位
 * - 注意事项：必须在任务上下文调用；超时后仍会继续下发，避免死等导致整机卡死
 *
 * @param direction 0=下降（BeginFlag），1=上升（EndFlag）
 * @retval 无
 */
static void Grab_MotorStart(uint8_t direction)
{
  uint32_t start = osKernelGetTickCount();

  while (Grab_MotorBusy != 0U)
  {
    if ((osKernelGetTickCount() - start) >= GRAB_MOTOR_TIMEOUT_MS)
    {
      break;
    }
    osDelay(10U);
  }

  Grab_MotorDone = 0U;
  if (direction == 0U)
  {
    Grab_MotorBeginFlag = 1U;
  }
  else
  {
    Grab_MotorEndFlag = 1U;
  }
}

/**
 * @brief 等待 GrabTask 完成当前电机动作
 *
 * @details
 * - 设计意图：用完成标志代替固定延时，GrabTask 实际停稳后才继续后续舵机/水泵时序
 * - 实现要点：轮询 Grab_MotorDone，超过 GRAB_MOTOR_TIMEOUT_MS 返回失败
 * - 注意事项：只能在任务上下文调用；超时只上报结果，不在这里强停电机
 *
 * @param 无
 * @retval 1 动作完成；0 等待超时
 */
static uint8_t Grab_MotorWaitDone(void)
{
  uint32_t start = osKernelGetTickCount();

  while (Grab_MotorDone == 0U)
  {
    if ((osKernelGetTickCount() - start) >= GRAB_MOTOR_TIMEOUT_MS)
    {
      return 0U;
    }
    osDelay(10U);
  }

  return 1U;
}

/**
 * @brief 启动抓取电机下降（不等待完成）
 *
 * @details 供"移动与下降并行"的通用流程使用，调用后由 Grab_MotorWaitDone 收尾。
 *
 * @param 无
 * @retval 无
 */
static void Grab_MotorDownStart(void)
{
  Grab_MotorStart(0U);
}

/**
 * @brief 启动抓取电机上升（不等待完成）
 *
 * @details 供"舵机回升与电机上升并行"的流程使用，调用后由 Grab_MotorWaitDone 收尾。
 *
 * @param 无
 * @retval 无
 */
static void Grab_MotorUpStart(void)
{
  Grab_MotorStart(1U);
}

/* 舵机1 下降/回升：定义在下方，Grab_MotorRunFor 需要先声明才能并行调用 */
static void Grab_ServoDown(void);
static void Grab_ServoUp(void);

/**
 * @brief 按指定时长让抓取电机下降/上升，同时驱动舵机1 同向转动，等待两者都完成
 *
 * @details
 * - 设计意图：放置后的附加动作只需要短时上下（2.5s），与抓取段的 5s 行程不同；
 *   电机仍走"置方向标志 + GrabTask 执行"的既有机制，不绕过任务直接操作串口；
 *   舵机与电机并行动作，缩短整段时序
 * - 实现要点：
 *   1. 先把本次时长写进 Grab_MotorRunMs（GrabTask 消费标志时读走并清零）
 *   2. Grab_MotorStart 负责等上一次动作结束、清完成标志、置方向标志
 *   3. 紧接着调用 Grab_ServoDown/Up，电机与舵机同时动作；
 *      Servo1_MoveTeeth 内部 osDelay 到舵机走完，期间电机一直在跑
 *   4. 最后 Grab_MotorWaitDone 等 GrabTask 停机，保证两者都结束才返回
 * - 数据流：run_ms -> Grab_MotorRunMs -> GrabTask -> osDelay(run_ms) + 停机；
 *   direction -> Grab_ServoDown/Up -> Servo1_MoveTeeth(∓GRAB_SERVO1_TEETH)
 * - 注意事项：必须在任务上下文调用；run_ms 为 0 时 GrabTask 会退回默认宏时长
 *
 * @param direction 0=下降（BeginFlag + 舵机下降），1=上升（EndFlag + 舵机回升）
 * @param run_ms 电机本次动作时长，单位 ms
 * @retval 无
 */
static void Grab_MotorRunFor(uint8_t direction, uint16_t run_ms)
{
  Grab_MotorRunMs = run_ms;
  Grab_MotorStart(direction);

  /* 与电机并行：下降时舵机1 下降，上升时回升相同齿数 */
  if (direction == 0U)
  {
    Grab_ServoDown();
  }
  else
  {
    Grab_ServoUp();
  }

  (void)Grab_MotorWaitDone();
}

/**
 * @brief 舵机1 下降指定齿数（抓取段）
 *
 * @details 直接复用 Servo1_MoveTeeth 的齿数换算，负齿数为下降
 *
 * @param 无
 * @retval 无
 */
static void Grab_ServoDown(void)
{
   Servo1_MoveTeeth(-GRAB_SERVO1_TEETH);
}

/**
 * @brief 舵机1 回升指定齿数（吸取后）
 *
 * @details 与下降齿数对称，用负齿数把舵机转回原位
 *
 * @param 无
 * @retval 无
 */
static void Grab_ServoUp(void)
{
   Servo1_MoveTeeth(GRAB_SERVO1_TEETH);
}



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
 *   自检请放到 GrabTask 里做。
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
 * @brief 抓取机构任务入口（当前为占位实现，后续抓取动作放在这里执行）
 *
 * @details 详细思路：
 * - 设计意图：Grab_To_Zero/Grab_Begin/Grab_End 内部都是 osDelay 轮询等待，
 *   只能在任务上下文跑，不能塞进 UART 解析或中断里
 * - 实现要点：占位阶段每轮 osDelay(100) 主动让出 CPU，绝不能写空转死循环
 * - 注意事项：优先级与 ServoTask 相同（osPriorityBelowNormal(16)），
 *   同优先级按 FreeRTOS 时间片轮转，因此本循环必须阻塞
 * - 调用关系：由 main.c 的 osThreadNew 创建
 *
 * @param argument 未使用
 * @retval 无（任务不返回）
 */
void GrabTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    uint8_t direction = 0U;
    uint8_t has_job = 0U;

    if (Grab_MotorBeginFlag == 1U)
    {
      Grab_MotorBeginFlag = 0U;
      direction = 0U;
      has_job = 1U;
    }
    else if (Grab_MotorEndFlag == 1U)
    {
      Grab_MotorEndFlag = 0U;
      direction = 1U;
      has_job = 1U;
    }

    if (has_job != 0U)
    {
      uint16_t run_ms;

      Grab_MotorBusy = 1U;
      Grab_MotorDone = 0U;

      /* 一次性时长覆盖（Grab_MotorRunFor 用），用完即清，后续抓取仍用默认宏 */
      run_ms = Grab_MotorRunMs;
      Grab_MotorRunMs = 0U;
      if (run_ms == 0U)
      {
        run_ms = (direction == 0U) ? GRAB_MOTOR_DOWN_MS : GRAB_MOTOR_UP_MS;
      }

      if (StepperMotor_SetSpeed(&stepper_motor_a2, direction, 200, 50, false) != HAL_OK)
      {
        /* 速度命令可能已经发出去了（只是等待发送完成超时），这里补一次停机；
           绝不能 return，否则整个 GrabTask 被删除，后续抓取命令全部失效 */
        StepperMotor_Stop(&stepper_motor_a2);
      }
      else
      {
        osDelay(run_ms);
        StepperMotor_Stop(&stepper_motor_a2);
      }

      Grab_MotorBusy = 0U;
      Grab_MotorDone = 1U;
    }

    osDelay(10U);
  }
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
if(StepperMotor_SetSpeed(&stepper_motor_a2, 1, 200,50, false) != HAL_OK)
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
/**
 * @brief 启动水泵吸水（只置标志，由 WaterPumpTask 执行硬件时序）
 *
 * @details
 * - 设计意图：抓取流程与 UART1 的 switch/WaterpumpOn 命令共用同一个执行任务，
 *   避免 CoreXYTask 与 WaterPumpTask 同时操作 TIM4 造成竞争
 * - 实现要点：置 OnFlag 并清 Off/Duty 标志，等待 1000ms 让 WaterPumpTask 完成
 *   开阀、启泵时序（任务每 100ms 轮询一次，启泵本身含 200ms 阀门延时）
 * - 注意事项：必须在任务上下文调用；本函数返回时泵已运行约 700~800ms
 *
 * @param 无
 * @retval 无
 */
void Grab_Begin(void)
{
  WaterPump_OffFlag = 0U;
  WaterPump_DutyUpdateFlag = 0U;
  WaterPump_OnFlag = 1U;
  osDelay(1000U);
}

/**
 * @brief 停止水泵放料（只置标志，由 WaterPumpTask 执行硬件时序）
 *
 * @details
 * - 设计意图：与 Grab_Begin 对称，把停泵时序统一交给 WaterPumpTask
 * - 实现要点：置 OffFlag 并清 On/Duty 标志；Grab_XY 随后还会等待 700ms 让物体落稳，
 *   足够 WaterPumpTask 完成"先停泵再关阀"的 200ms 时序
 * - 注意事项：必须在任务上下文调用；调用后不要立即再次启泵
 *
 * @param 无
 * @retval 无
 */
void Grab_End(void)
{
  WaterPump_OnFlag = 0U;
  WaterPump_DutyUpdateFlag = 0U;
  WaterPump_OffFlag = 1U;
}
/**
 * @brief 完成一次「抓取 + 放置」完整动作（统一使用非球流程）
 *
 * @details
 * - 设计意图：把取物点和放置点的动作串成固定时序，CoreXYTask 只需给两组坐标；
 *   串口已不再下发形状，所有物料统一走"移动与下降重叠"的通用流程
 * - 实现要点：
 *   1. 抓取：移到 (x1,y1) 与电机下降同时开始 -> 等移动和电机都完成 ->
 *      舵机1下降 6 齿 -> 开泵吸住
 *   2. 抬起：电机上升与舵机1回升 6 齿同时进行，用完成标志等待电机停稳
 *   3. 放置：移到 (x2,y2) -> 停泵松开
 *   4. 松件：抓取电机下降 2.5s 与舵机1 下降 5 齿同时进行，再一起上升 2.5s / 5 齿
 *   5. 收尾：回到等待点 (Wait_X,Wait_Y) 后再等 300ms
 * - 数据流：坐标(mm) -> CoreXY_MoveTo；标志位 -> GrabTask 执行电机命令；
 *   Servo1_MoveTeeth 按 167us/齿写 TIM4 CH3；水泵命令交给 WaterPumpTask
 * - 注意事项：全程使用 osDelay，必须在任务上下文调用；不再直接写 Grab_MotorEndFlag，
 *   统一通过 Grab_Motor* 接口维护忙/完成标志，避免固定延时与实际动作错位
 * - 调用关系：由 CoreXYTask 取出 Grab 任务后调用
 *
 * @param x1 取物点 X 坐标，单位 mm
 * @param y1 取物点 Y 坐标，单位 mm
 * @param x2 放置点 X 坐标，单位 mm
 * @param y2 放置点 Y 坐标，单位 mm
 * @retval 无
 */
void Grab_XY(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
   /* K 为时间系数：1.0 = 按 100RPM 的理论匀速时间，加减速明显时调大即可 */
   const float K=1.9f; const uint16_t speed_rpm=200U;  //1.9-2.1
   const uint8_t accel=40U;
   /* 当前位置直接取 CoreXY 模块跟踪的规划坐标，不再在这里另存一份 */
   float dx=(float)x1-CoreXY_GetPositionX();
   float dy=(float)y1-CoreXY_GetPositionY();

   /* ===== 抓取阶段：移动到取物点与电机下降同时开始 ===== */
   CoreXY_MoveTo(x1,y1,speed_rpm,accel);

   uint32_t time=Grab_MoveTimeMs(dx,dy,speed_rpm,K);
   Grab_MotorDownStart();
   osDelay(time);
   /* 行程走完后仍要等电机真正停稳；若电机先到位则立即返回 */
   (void)Grab_MotorWaitDone();

   Grab_ServoDown();
   /* 舵机走完 5 齿后再留出余量，确认完全到位才开泵 */
   osDelay(GRAB_SUCTION_DELAY_MS);
   Grab_Begin();

   /* 吸住后：电机上升与舵机回升 5 齿同时开始，最后等电机停稳 */
   Grab_MotorUpStart();
   Grab_ServoUp();
   (void)Grab_MotorWaitDone();

   /* ===== 放置阶段：移动到最终位置 -> 直接停泵松开（电机和舵机都不再动作） ===== */
   dx=(float)x2-CoreXY_GetPositionX();
   dy=(float)y2-CoreXY_GetPositionY();
   CoreXY_MoveTo(x2,y2,speed_rpm,accel);
   osDelay(Grab_MoveTimeMs(dx,dy,speed_rpm,K));

   Grab_End();

   /* 松泵后稍等物体落稳 */
  ////
  //在这边让电机下降2.5s在上升2.5s(标志位实现)
  /* 电机短时下降再上升：置标志位由 GrabTask 执行，每次都要等动作结束才继续 */
   Grab_MotorRunFor(0U, GRAB_RELEASE_DOWN_MS);
   Grab_MotorRunFor(1U, GRAB_RELEASE_UP_MS);
  ///

  CoreXY_MoveTo(Wait_X,Wait_Y,speed_rpm,accel);
   osDelay(300U);
   
}

