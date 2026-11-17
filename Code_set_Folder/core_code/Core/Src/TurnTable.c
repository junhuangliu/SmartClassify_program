#include "TurnTable.h"
#include "main.h"

extern UART_HandleTypeDef huart5;

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

else if(Table>CurTable){
 
StepperMotor_SetPosition(&stepper_motor_b2, 1, 300, 20,(Table-CurTable)*533.333, true, false);


}
else if(Table<CurTable){
StepperMotor_SetPosition(&stepper_motor_b2, 0, 300, 20,(CurTable-Table)*533.333, true, false);

}
CurTable=Table;

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
