#ifndef GRAB_H
#define GRAB_H

#include "stepper_motor.h"
#include "main.h"
#include  "WaterPump.h"
#include "uart1_proto.h"
#include "CoreXY.h"
#include "TurnTable.h"
extern UART_HandleTypeDef huart6;

/**
 * @brief 初始化抓取机构步进电机通信。
 * @details 复用步进电机模块统一的初始化流程，将 stepper_motor_a2 绑定到 UART6，
 *          使能 USART6 全局中断并启动应答 DMA 接收。本函数在 osKernelStart()
 *          之前调用，因此只做纯初始化，任何需要延时的动作都放到任务里做
 *          （见 GrabTask）。
 * @retval HAL 状态码；UART6 未配置 RX DMA 或接收启动失败时返回对应错误状态。
 */
HAL_StatusTypeDef Grab_Init(void);

/**
 * @brief 抓取机构任务入口，当前占位循环 osDelay。
 * @details 与 ServoTask 同为 osPriorityBelowNormal，按时间片轮转共享 CPU；
 *          后续抓取动作在任务上下文执行（内部 osDelay 不能在中断里调用）。
 * @param argument 未使用。
 * @retval 无（任务不返回）。
 */
void GrabTask(void *argument);

void Grab_To_Zero(void);
void Grab_Begin(void);
void Grab_End(void);
void Grab_XY(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,TurnTable_Shape_t Shape);

/* 常用坐标点：运行期可通过 UART1 的 Set,<名称>,<数值>/ 动态修改；
   因特殊点存在负值（如 Y0=-15），统一用有符号类型存储 */
extern volatile int16_t Destination_X;
extern volatile int16_t Destination_Y;

/* 执行标志位：Uart1RxTask 收到 MDown/MUp 后置位，GrabTask 消费后清零 */
extern volatile uint8_t Grab_MotorBeginFlag;
extern volatile uint8_t Grab_MotorEndFlag;

/* 标定特殊点：成对使用 (X0,Y0)/(X1,Y1)/(X2,Y2)，可用 MoveTo,X0,Y0/ 直达 */
extern volatile int16_t X0;
extern volatile int16_t Y0;
extern volatile int16_t X1;
extern volatile int16_t Y1;
extern volatile int16_t X2;
extern volatile int16_t Y2;

#endif
