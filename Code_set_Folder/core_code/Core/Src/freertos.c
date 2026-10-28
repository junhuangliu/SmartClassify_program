/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "uart1_comm.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/**
 * @brief USART1 接收验证任务
 * 
 * @details
 * - 设计意图：从 uart1_comm 消息队列取出接收数据块，用于底层通路联调验证
 * - 实现要点：从队列获取 (pData,len)，直接通过 Uart1Comm_Send 回显到上位机，便于观察收发是否正常
 * - 注意事项：当前仅用于调试，后续替换为 uart1_proto 解析逻辑
 * 
 * @param argument 未使用
 * @retval 无
 */
void Uart1RxTask(void *argument);

/**
 * @brief PA10闪烁测试任务
 * 
 * @details 
 * - 设计意图：验证PA10 GPIO输出功能
 * - 实现要点：每500ms调用HAL_GPIO_TogglePin翻转PA10电平
 * - 注意事项：使用osDelay保证延时，避免阻塞调度器
 * 
 * @param argument 未使用
 * @retval 无
 */
void PA10BlinkTask(void *argument);

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief USART1 接收验证任务实现
 * 
 * @details
 * - 设计意图：验证 DMA+空闲中断底层通路是否正常工作
 * - 实现要点：循环从 uart1_rx_queue 取消息，使用 Uart1Comm_Send 将原始数据回显
 * - 注意事项：
 *   1. 回显仅用于联调，等协议确定后需改为协议解析流程
 *   2. 队列元素中的 pData 指向 DMA 缓冲区，回显时直接用该指针即可
 * 
 * @param argument 未使用
 * @retval 无
 */
void Uart1RxTask(void *argument)
{
  (void)argument;
  osMessageQueueId_t rxQueue = Uart1Comm_GetRxQueueHandle();
  Uart1RxMsg_t msg;

  for (;;)
  {
    if (rxQueue != NULL)
    {
      if (osMessageQueueGet(rxQueue, &msg, NULL, osWaitForever) == osOK)
      {
        if ((msg.pData != NULL) && (msg.len > 0))
        {
          /* 底层联调：收到什么就发回去，用于验证收发通路 */
          Uart1Comm_Send(msg.pData, msg.len);
        }
      }
    }
    else
    {
      osDelay(10);
    }
  }
}

/**
 * @brief PA10闪烁测试任务实现
 * 
 * @details 
 * - 设计意图：持续翻转PA10实现闪烁效果，用于硬件验证
 * - 实现要点：循环调用HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10)，每500ms延时
 * - 注意事项：任务需长期运行，使用osDelay而非HAL_Delay
 * 
 * @param argument 未使用
 * @retval 无
 */
void PA10BlinkTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
    osDelay(200);
  }
}

/* USER CODE END Application */

