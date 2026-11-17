#ifndef GRAB_H
#define GRAB_H

#include "stepper_motor.h"
#include "main.h"
#include  "WaterPump.h"
#include "uart1_proto.h"
#include "CoreXY.h"
extern UART_HandleTypeDef huart6;

/**
 * @brief 初始化抓取机构步进电机通信。
 * @details 复用步进电机模块统一的初始化流程，将 stepper_motor_a2 绑定到 UART6，
 *          使能 USART6 全局中断并启动应答 DMA 接收。本函数在 osKernelStart()
 *          之前调用，因此只做纯初始化，任何需要延时的动作都放到任务里做
 *          （见 Grab_PowerOnTest）。
 * @retval HAL 状态码；UART6 未配置 RX DMA 或接收启动失败时返回对应错误状态。
 */
HAL_StatusTypeDef Grab_Init(void);

/**
 * @brief 抓取机构上电自检：只执行一次，让 stepper_motor_a2 一直转，读到驱动器
 *        堵转标志后停机、清标志并把当前位置记为 0。
 * @details 必须在 FreeRTOS 任务上下文调用（内部用 osDelay 轮询等待，最长约 20s）。
 *          调度器启动前调用会卡死，禁止在 Grab_Init 中直接调用。
 * @param 无
 * @retval 无
 */
void Grab_PowerOnTest(void);
void Grab_To_Zero(void);
void Grab_Begin(void);
void Grab_End(void);
void Grab_XY(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
#define Destination_X 95
#define Destination_Y 255
#define X0 0
#define Y0 -15
#define X1 190
#define Y1 -15
#define X2 0
#define Y2 165
#endif
