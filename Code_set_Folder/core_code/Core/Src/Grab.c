#include "Grab.h"
#include <math.h>
#include <stdio.h>

/* 电机每转的皮带行程：20 齿 × 2mm 齿距 = 40mm */
#define GRAB_BELT_MM_PER_REV \
  (COREXY_BELT_TOOTH_PITCH_MM * COREXY_PULLEY_TEETH)


uint8_t Grab_MotorBeginFlag=0;
uint8_t Grab_MotorEndFlag=0;



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
   osDelay(4250);
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
   osDelay(4250);
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
void Grab_XY(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
   /* K 为时间系数：1.0 = 按 100RPM 的理论匀速时间，加减速明显时调大即可 */
   const float K=1.9f; const uint16_t speed_rpm=100U;  //1.9-2.1  
   const uint8_t accel=40U;
   /* 当前位置直接取 CoreXY 模块跟踪的规划坐标，不再在这里另存一份 */
   float dx=(float)x1-CoreXY_GetPositionX();
   float dy=(float)y1-CoreXY_GetPositionY();

   CoreXY_MoveTo(x1,y1,speed_rpm,accel);
   uint32_t time=Grab_MoveTimeMs(dx,dy,speed_rpm,K);
   Grab_MotorBeginFlag=1;
   if(time>4250){
   osDelay(time);
  }
  else if(time<4250){
  osDelay(4250-time);

  }
   Grab_Begin();

   dx=(float)x2-CoreXY_GetPositionX();
   dy=(float)y2-CoreXY_GetPositionY();
   CoreXY_MoveTo(x2,y2,speed_rpm,accel);
   time=Grab_MoveTimeMs(dx,dy,speed_rpm,K);
   //osDelay(time);
  Grab_MotorEndFlag=1;

   if(time>4250){
   osDelay(time);
  }
  else if(time<4250){
  osDelay(4250-time);

  }
   Grab_End();
   }

