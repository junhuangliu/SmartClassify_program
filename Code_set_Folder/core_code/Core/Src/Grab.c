#include "Grab.h"
#include "servo.h"
#include  "TurnTable.h"
#include <math.h>
#include <stdio.h>

/* 电机每转的皮带行程：20 齿 × 2mm 齿距 = 40mm */
#define GRAB_BELT_MM_PER_REV \
  (COREXY_BELT_TOOTH_PITCH_MM * COREXY_PULLEY_TEETH)

/* 舵机1 抓取行程：下降 5 齿，吸住后回升 5 齿（Servo1_MoveTeeth 内部按 167us/齿换算） */
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


uint8_t Grab_MotorBeginFlag=0;
uint8_t Grab_MotorEndFlag=0;

/**
 * @brief 触发抓取电机下降到抓取位并等待动作完成
 *
 * @details
 * - 设计意图：CoreXYTask 只负责时序，实际电机命令由 GrabTask 轮询标志位执行
 * - 实现要点：置 Grab_MotorBeginFlag 后等待 GRAB_MOTOR_DOWN_MS 加余量，确保 GrabTask 完成动作
 * - 注意事项：内部 osDelay 只能在任务上下文调用；等待期间其它任务正常运行
 *
 * @param 无
 * @retval 无
 */
static void Grab_MotorDown(void)
{
   Grab_MotorBeginFlag = 1U;
   osDelay(GRAB_MOTOR_DOWN_MS + GRAB_MOTOR_WAIT_MARGIN_MS);
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
    if(Grab_MotorBeginFlag==1){
      Grab_MotorBeginFlag=0;
       if(StepperMotor_SetSpeed(&stepper_motor_a2, 0, 200,50, false) != HAL_OK)
   {
     /* 速度命令可能已经发出去了（只是等待发送完成超时），这里补一次停机，
        避免函数提前返回后电机一直自转 */
     StepperMotor_Stop(&stepper_motor_a2);
      return;
    }
    osDelay(GRAB_MOTOR_DOWN_MS);
   StepperMotor_Stop(&stepper_motor_a2);
     }
     if(Grab_MotorEndFlag==1){
      Grab_MotorEndFlag=0;
       if(StepperMotor_SetSpeed(&stepper_motor_a2, 1, 200,50, false) != HAL_OK)
   {
     /* 速度命令可能已经发出去了（只是等待发送完成超时），这里补一次停机，
        避免函数提前返回后电机一直自转 */
     StepperMotor_Stop(&stepper_motor_a2);
      return;
    }
    osDelay(GRAB_MOTOR_UP_MS);
   StepperMotor_Stop(&stepper_motor_a2);
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
void Grab_Begin()
{
 
 //uint16_t time=4250;

  WaterPump_Start();
  osDelay(1000U);
  
}
void Grab_End()
{


   WaterPump_Stop();

   }
/**
 * @brief 完成一次「抓取 + 放置」完整动作
 *
 * @details
 * - 设计意图：把取物点和放置点的动作串成固定时序，CoreXYTask 只需给两组坐标
 * - 实现要点：
 *   1. 抓取：移到 (x1,y1) -> 电机下降 -> 舵机1下降 5 齿 -> 开泵吸住
 *   2. 抬起：电机上升与舵机1回升 5 齿同时进行
 *   3. 放置：移到 (x2,y2) -> 直接停泵松开（电机和舵机都不再动作）
 *   4. 收尾：等待 300ms 让物体落稳
 * - 数据流：坐标(mm) -> CoreXY_MoveTo；标志位 -> GrabTask 执行电机命令；
 *   Servo1_MoveTeeth 按 167us/齿写 TIM4 CH3
 * - 注意事项：全程使用 osDelay，必须在任务上下文调用；电机动作按 GRAB_MOTOR_DOWN_MS/UP_MS
 *   等待，不能与 GrabTask 的电机动作并发下发
 * - 调用关系：由 CoreXYTask 在回零完成后调用
 * - 形状分派：SHAPE_SPHERE(球) 走先横移再下降的吸球流程；其余形状（长方体、正方体、
 *   圆锥、六棱柱、五棱柱）共用横移与下降重叠的通用流程；SHAPE_NULL 直接返回不动作
 *
 * @param x1 取物点 X 坐标，单位 mm
 * @param y1 取物点 Y 坐标，单位 mm
 * @param x2 放置点 X 坐标，单位 mm
 * @param y2 放置点 Y 坐标，单位 mm
 * @retval 无
 */
void Grab_XY(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,TurnTable_Shape_t Shape)
{
  if(Shape==SHAPE_NULL){
  return;

  }
  
   /* K 为时间系数：1.0 = 按 100RPM 的理论匀速时间，加减速明显时调大即可 */
   const float K=1.9f; const uint16_t speed_rpm=100U;  //1.9-2.1  
   const uint8_t accel=40U;
   /* 当前位置直接取 CoreXY 模块跟踪的规划坐标，不再在这里另存一份 */
   float dx=(float)x1-CoreXY_GetPositionX();
   float dy=(float)y1-CoreXY_GetPositionY();
if(Shape==SHAPE_SPHERE){
   /* ===== 抓取阶段：移动到取物点 -> 电机下降 -> 舵机下降 5 齿 -> 开泵吸住 ===== */
   CoreXY_MoveTo(x1,y1,speed_rpm,accel);
   osDelay(Grab_MoveTimeMs(dx,dy,speed_rpm,K));

   Grab_MotorDown();
   Grab_ServoDown();
   /* 舵机走完 5 齿后再留出余量，确认完全到位才开泵 */
   osDelay(GRAB_SUCTION_DELAY_MS);
   Grab_Begin();

   /* 吸住后：电机上升与舵机回升 5 齿同时开始 */
   Grab_MotorEndFlag = 1U;
   Grab_ServoUp();
   osDelay((GRAB_MOTOR_UP_MS > GRAB_SERVO1_MOVE_MS ?
            (GRAB_MOTOR_UP_MS - GRAB_SERVO1_MOVE_MS) : 0U) + GRAB_MOTOR_WAIT_MARGIN_MS);

   /* ===== 放置阶段：移动到最终位置 -> 直接停泵松开（电机和舵机都不再动作） ===== */
   dx=(float)x2-CoreXY_GetPositionX();
   dy=(float)y2-CoreXY_GetPositionY();
   CoreXY_MoveTo(x2,y2,speed_rpm,accel);
   osDelay(Grab_MoveTimeMs(dx,dy,speed_rpm,K));

   Grab_End();

   /* 松泵后稍等物体落稳 */
   osDelay(700U);
   Turn_Flag=1;
  GrabShape=Shape;
  }
  /* 其余形状（长方体、正方体、圆锥、六棱柱、五棱柱）动作流程一致，统一走这一支 */
 else {
  CoreXY_MoveTo(x1,y1,speed_rpm,accel);

  uint32_t time=Grab_MoveTimeMs(dx,dy,speed_rpm,K);
  Grab_MotorBeginFlag=1;
  if(time>=GRAB_MOTOR_DOWN_MS){
  osDelay(time);
     }
     else if(time<GRAB_MOTOR_DOWN_MS){

    osDelay(GRAB_MOTOR_DOWN_MS);
     }
    Grab_ServoDown();
   /* 舵机走完 5 齿后再留出余量，确认完全到位才开泵 */
   osDelay(GRAB_SUCTION_DELAY_MS);
   Grab_Begin();

   /* 吸住后：电机上升与舵机回升 5 齿同时开始 */
   Grab_MotorEndFlag = 1U;
   Grab_ServoUp();
   osDelay((GRAB_MOTOR_UP_MS > GRAB_SERVO1_MOVE_MS ?
            (GRAB_MOTOR_UP_MS - GRAB_SERVO1_MOVE_MS) : 0U) + GRAB_MOTOR_WAIT_MARGIN_MS);

   /* ===== 放置阶段：移动到最终位置 -> 直接停泵松开（电机和舵机都不再动作） ===== */
   dx=(float)x2-CoreXY_GetPositionX();
   dy=(float)y2-CoreXY_GetPositionY();
   CoreXY_MoveTo(x2,y2,speed_rpm,accel);
   osDelay(Grab_MoveTimeMs(dx,dy,speed_rpm,K));

   Grab_End();

   /* 松泵后稍等物体落稳 */
   osDelay(700U); 
  Turn_Flag=1;
  GrabShape=Shape;
 }

}

