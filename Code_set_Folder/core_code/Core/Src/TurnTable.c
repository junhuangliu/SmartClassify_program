#include "TurnTable.h"
#include "servo.h"
#include "main.h"
#include "cmsis_os2.h"
#include <math.h>
extern UART_HandleTypeDef huart5;
volatile TurnTable_Shape_t GrabShape=SHAPE_NULL;
volatile uint8_t Turn_Flag;
static uint8_t CurTable=0;

/**
 * @brief 初始化转台步进电机通信。
 * @details 复用统一的步进电机初始化流程，将 stepper_motor_b2 关联到 huart5，
 *          清理接收状态并启动应答 DMA 接收。应在 CubeMX 完成 UART5 初始化后调用。
 *          这里同时使能 UART5 全局中断：HAL_UARTEx_ReceiveToIdle_DMA 依赖 UART
 *          中断里的 IDLE 分支切分不定长应答帧，.ioc 中尚未勾选该中断。
 * @retval HAL 状态码；UART5 未配置 RX DMA 或接收启动失败时返回对应错误状态。
 */
HAL_StatusTypeDef TurnTable_Init(void)
{
  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
  return StepperMotor_Init(&stepper_motor_b2, &huart5);
}
void TurnTable_Set(uint8_t Table){
if(Table>5){

  return ;
}
else if(Table==CurTable){
  return ;
}
/* 必须用有符号类型：uint8_t 相减回绕成 251~255，dis<0 分支永远进不去 */
int8_t dis=Table-CurTable;
int8_t absdis=fabs(Table-CurTable);
 if(dis>0 && absdis<=3){
 
StepperMotor_SetPosition(&stepper_motor_b2, 1, 300, 20,absdis*533.333, true, false);


}
else if(dis>0 && absdis>3){
StepperMotor_SetPosition(&stepper_motor_b2, 0, 300, 20,(6-absdis)*533.333, true, false);

}
else if(dis<0 && absdis<=3){
 
StepperMotor_SetPosition(&stepper_motor_b2, 0, 300, 20,absdis*533.333, true, false);


}else if(dis<0 && absdis>3){
 
StepperMotor_SetPosition(&stepper_motor_b2, 1, 300, 20,(6-absdis)*533.333, true, false);


}
CurTable=Table;

}

/**
 * @brief 转台任务入口（当前为占位实现，等待后续接入转台动作逻辑）
 *
 * @details 详细思路：
 * - 设计意图：给转台模块保留独立任务，后续把 TurnTable_Set 等阻塞动作放到这里执行，
 *   避免在 UART 解析等上下文里直接调用阻塞接口
 * - 实现要点：占位阶段每轮 osDelay(100) 主动让出 CPU，绝不能写空转死循环；
 *   否则它永远处于就绪态，会饿死同优先级任务
 * - 注意事项：优先级与 ServoTask 相同（osPriorityBelowNormal(16)），
 *   同优先级任务按 FreeRTOS 时间片（1 个 tick）轮转，因此本循环必须阻塞
 * - 调用关系：仅由 main.c 的 osThreadNew 创建，运行在任务上下文
 *
 * @param argument 未使用
 * @retval 无（任务不返回）
 */
void TurnTableTask(void *argument)
{
  
  (void)argument;
  for (;;)
  {
    /************电机测试 */
    #if 0
    TurnTable_Set(i);
    osDelay(2000U);
    i++;
    if(i>5){
      i=0;
    }
    #endif
if(Turn_Flag==1){
Turn_Flag=0;
TurnTable_Set((uint8_t)GrabShape);
osDelay(700);
Servo2_OpenFlag=1;

}
 osDelay(50);
  }
}

/**
 * @brief UART5 全局中断入口，负责分发转台驱动器应答。
 *
 * @details 详细思路：
 * - 设计意图：转台驱动器应答帧长度不固定（转速 6 字节、位置 8 字节），
 *   StepperMotor 模块用空闲中断 + DMA 接收，需要 UART 全局中断把 IDLE 事件
 *   交给 HAL 处理，再由 HAL 回调到 StepperMotor_RxEventCallback 完成解帧。
 * - 实现要点：直接转给 HAL_UART_IRQHandler，其余解析逻辑全部复用电机模块，
 *   本函数不写任何业务代码；CubeMX 未生成该入口，故在此提供。
 * - 注意事项：中断上下文，禁止阻塞；若后续在 CubeMX 中勾选 UART5 global interrupt，
 *   需删除本函数，避免与 stm32f4xx_it.c 生成的定义重复。
 *
 * @param 无
 * @retval 无
 */
void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}
