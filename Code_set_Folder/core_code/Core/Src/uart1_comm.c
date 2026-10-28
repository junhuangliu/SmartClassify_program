/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart1_comm.c
  * @brief          : USART1 串口通信模块实现（DMA + 空闲中断）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "uart1_comm.h"
#include "string.h"
#include "cmsis_os.h"
#include "queue.h"
#include "FreeRTOS.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART1_RX_DMA_BUF_SIZE  256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;

static uint8_t rx_dma_buf[UART1_RX_DMA_BUF_SIZE];
static osMessageQueueId_t uart1_rx_queue = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 初始化 USART1 通信模块
 * 
 * @details
 * - 设计意图：搭建 USART1 DMA+空闲中断底层通路，将接收数据安全地从 ISR 传递给 FreeRTOS 任务
 * - 实现要点：
 *   1. 创建消息队列 uart1_rx_queue（深度 16，每个元素为 Uart1RxMsg_t）
 *   2. 调用 HAL_UARTEx_ReceiveToIdle_DMA 开启 DMA 循环接收 + 空闲中断
 * - 注意事项：
 *   1. 队列元素只传递“指针+长度”，指针指向 rx_dma_buf。任务需尽快处理，避免覆盖
 *   2. USART1 中断优先级已在 CubeMX 配置为 5，满足 configMAX_SYSCALL_INTERRUPT_PRIORITY，允许在回调中使用 xxxFromISR API
 * 
 * @param 无
 * @retval 无
 */
void Uart1Comm_Init(void)
{
  /* USER CODE BEGIN Uart1Comm_Init 0 */

  /* USER CODE END Uart1Comm_Init 0 */

  /* 创建接收消息队列：用于 ISR -> 任务解耦 */
  if (uart1_rx_queue == NULL)
  {
    uart1_rx_queue = osMessageQueueNew(16, sizeof(Uart1RxMsg_t), NULL);
  }

  /* 启动 DMA 循环接收 + UART 空闲中断 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, UART1_RX_DMA_BUF_SIZE);

  /* 关闭半传输中断（大多数串口场景不需要 HT 中断） */
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

  /* USER CODE BEGIN Uart1Comm_Init 1 */

  /* USER CODE END Uart1Comm_Init 1 */
}

/**
 * @brief 通过 USART1 发送数据（DMA 方式）
 * 
 * @details
 * - 设计意图：提供最简洁的底层发送接口，供上位机调试回显或后续业务模块调用
 * - 实现要点：直接调用 HAL_UART_Transmit_DMA 启动发送
 * - 注意事项：
 *   1. 若 DMA 正忙，HAL_UART_Transmit_DMA 可能返回 HAL_BUSY
 *   2. 发送缓冲区需在发送完成前保持有效
 * 
 * @param pData 待发送数据缓冲区指针
 * @param len   待发送数据长度
 * @retval HAL_StatusTypeDef HAL_OK 成功，HAL_BUSY 正忙，其他值错误
 */
HAL_StatusTypeDef Uart1Comm_Send(const uint8_t *pData, uint16_t len)
{
  /* USER CODE BEGIN Uart1Comm_Send 0 */

  /* USER CODE END Uart1Comm_Send 0 */

  if ((pData == NULL) || (len == 0))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit_DMA(&huart1, (uint8_t *)pData, len);

  /* USER CODE BEGIN Uart1Comm_Send 1 */

  /* USER CODE END Uart1Comm_Send 1 */
}

/**
 * @brief 获取 USART1 接收消息队列句柄
 * 
 * @details 用于任务模块获取队列句柄以读取接收数据块
 * 
 * @param 无
 * @retval osMessageQueueId_t 接收消息队列句柄
 */
osMessageQueueId_t Uart1Comm_GetRxQueueHandle(void)
{
  /* USER CODE BEGIN Uart1Comm_GetRxQueueHandle 0 */

  /* USER CODE END Uart1Comm_GetRxQueueHandle 0 */

  return uart1_rx_queue;

  /* USER CODE BEGIN Uart1Comm_GetRxQueueHandle 1 */

  /* USER CODE END Uart1Comm_GetRxQueueHandle 1 */
}

/**
 * @brief UART 接收事件回调（空闲中断触发）
 * 
 * @details
 * - 设计意图：在总线空闲时获取本次实际接收长度，将数据块信息发送到 FreeRTOS 队列
 * - 实现要点：
 *   1. 仅处理 huart1
 *   2. 使用 xQueueSendFromISR 从中断安全地入队
 *   3. 正确使用 pxHigherPriorityTaskWoken 并调用 portYIELD_FROM_ISR 请求调度
 * - 注意事项：ISR 中绝对不能做阻塞、长循环、耗时解析
 * 
 * @param huart UART 句柄
 * @param Size  本次空闲前累计接收到的字节数
 * @retval 无
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* USER CODE BEGIN HAL_UARTEx_RxEventCallback 0 */

  /* USER CODE END HAL_UARTEx_RxEventCallback 0 */

  if (huart->Instance == USART1)
  {
    if ((uart1_rx_queue != NULL) && (Size > 0))
    {
      Uart1RxMsg_t msg;
      msg.pData = rx_dma_buf;
      msg.len   = Size;

      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xQueueSendFromISR(uart1_rx_queue, &msg, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }

  /* USER CODE BEGIN HAL_UARTEx_RxEventCallback 1 */

  /* USER CODE END HAL_UARTEx_RxEventCallback 1 */
}

/**
 * @brief UART 发送完成回调
 * 
 * @param huart UART 句柄
 * @retval 无
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* USER CODE BEGIN HAL_UART_TxCpltCallback 0 */

  /* USER CODE END HAL_UART_TxCpltCallback 0 */

  if (huart->Instance == USART1)
  {
    /* 发送完成，可在此处从发送队列取下一帧并发送（预留扩展） */
  }

  /* USER CODE BEGIN HAL_UART_TxCpltCallback 1 */

  /* USER CODE END HAL_UART_TxCpltCallback 1 */
}

/**
 * @brief UART 错误回调
 * 
 * @details 处理 UART/DMA 错误并尝试恢复接收
 * 
 * @param huart UART 句柄
 * @retval 无
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* USER CODE BEGIN HAL_UART_ErrorCallback 0 */

  /* USER CODE END HAL_UART_ErrorCallback 0 */

  if (huart->Instance == USART1)
  {
    /* 清除错误标志 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    /* 尝试重新开启 DMA 空闲接收，恢复通信 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, UART1_RX_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }

  /* USER CODE BEGIN HAL_UART_ErrorCallback 1 */

  /* USER CODE END HAL_UART_ErrorCallback 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
