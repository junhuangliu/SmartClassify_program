/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart1_comm.h
  * @brief          : USART1 串口通信模块（DMA + 空闲中断）
  ******************************************************************************
  * @attention
  *
  * 本模块用于实现板子与上位机通过 USART1 的底层收发：
  * - 接收：采用 DMA 循环 + UART 空闲中断（HAL_UARTEx_ReceiveToIdle_DMA）
  * - 发送：采用 DMA（HAL_UART_Transmit_DMA）
  * - 解耦：通过 FreeRTOS 消息队列将 ISR 数据块传递给任务处理
  *
  * 全部自定义逻辑均放置于 USER CODE 区域，符合项目约束。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __UART1_COMM_H__
#define __UART1_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief USART1 接收消息结构体
 * @details 用于在 UART 空闲中断回调中，将“本次空闲前实际接收到的数据块”
 *          通过 FreeRTOS 消息队列传递给 Uart1RxTask。
 */
typedef struct
{
  uint8_t *pData;  /* 指向 DMA 接收缓冲区中的数据起始地址 */
  uint16_t len;    /* 本次空闲中断触发时的有效数据长度 */
} Uart1RxMsg_t;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 初始化 USART1 通信模块
 * 
 * @details
 * - 创建接收消息队列
 * - 启动 DMA 循环接收并使能 UART 空闲中断：HAL_UARTEx_ReceiveToIdle_DMA
 * 
 * @param 无
 * @retval 无
 */
void Uart1Comm_Init(void);

/**
 * @brief 通过 USART1 发送数据（DMA 方式）
 * 
 * @details
 * - 使用 HAL_UART_Transmit_DMA 启动一次性 DMA 发送
 * - 发送完成会触发 HAL_UART_TxCpltCallback
 * - 当前为最小实现，若后续多任务并发发送再考虑加发送队列串行化
 * 
 * @param pData 待发送数据缓冲区指针
 * @param len   待发送数据长度
 * @retval HAL_StatusTypeDef HAL_OK 成功，其他值表示失败
 */
HAL_StatusTypeDef Uart1Comm_Send(const uint8_t *pData, uint16_t len);

/**
 * @brief 获取 USART1 接收消息队列句柄（供任务使用）
 * 
 * @details 主要用于 freertos.c 中创建 Uart1RxTask 后，从队列中获取接收到的数据块
 * 
 * @param 无
 * @retval osMessageQueueId_t 接收消息队列句柄
 */
osMessageQueueId_t Uart1Comm_GetRxQueueHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART1_COMM_H__ */
