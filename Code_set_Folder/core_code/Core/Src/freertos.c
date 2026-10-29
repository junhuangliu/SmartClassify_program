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
#include "ws2812_spi.h"
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
 * @brief WS2812 显示效果任务（闪烁灯 + 彩虹流）
 * 
 * @details
 * - 设计意图：单个任务交替运行两种模式，避免多任务竞争 SPI 总线
 * - 实现要点：每轮先跑 4s 闪烁灯，再跑 7s 彩虹流动，循环往复
 * - 数据流：仅操作 RGB 缓冲区，由 WS2812_Show() 统一编码并通过 SPI1 发送
 * - 注意事项：亮度始终全开，osDelay 控制帧速率
 * 
 * @param argument 未使用
 * @retval 无
 */
void WS2812Task(void *argument);

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
 * @brief WS2812 彩虹灯任务实现
 * 
 * @details
 * - 设计意图：四颗灯珠依次显示红橙黄绿青蓝紫中的连续颜色，整体常亮
 * - 实现要点：7色彩虹调色板（GRB顺序），每帧四个位置分配相邻三种色相；
 *   phase 逐帧+1使彩虹颜色流动变化
 * - 数据流：仅操作 RGB 缓冲区，由 WS2812_Show() 统一编码并通过 SPI1 发送
 * - 注意事项：亮度始终全开，osDelay(100) = 100ms/帧
 * 
 * @param argument 未使用
 * @retval 无
 */
void WS2812Task(void *argument)
{
  (void)argument;

  /* 七色彩虹调色板（GRB 顺序） */
  static const uint8_t rainbow[7][3] = {
      {0, 255, 0},       /* 红   G=0, R=255, B=0 */
      {127, 255, 0},     /* 橙   G=127, R=255, B=0 */
      {255, 255, 0},     /* 黄   G=255, R=255, B=0 */
      {255, 0, 0},       /* 绿   G=255, R=0, B=0 */
      {255, 0, 255},     /* 青   G=255, R=0, B=255 */
      {0, 0, 255},       /* 蓝   G=0, R=0, B=255 */
      {200, 0, 200}      /* 紫   G=200, R=0, B=200 */
  };

  uint16_t phase = 0;  /* 彩虹相位 */
  uint16_t mode = 0;   /* 闪烁模式索引 */
  uint8_t running_rainbow = 0;  /* 当前是否为彩虹模式 */

  for (;;)
  {
    if (running_rainbow)
    {
      /* ===== 彩虹流动模式，持续 7s ===== */
      uint16_t frame_count = 0;
      for (; frame_count < 140; frame_count++)
      {
        for (uint16_t i = 0; i < WS2812_LED_NUM; i++)
        {
          uint8_t idx = (phase + i) % 7;
          uint8_t g = rainbow[idx][0];
          uint8_t r = rainbow[idx][1];
          uint8_t b = rainbow[idx][2];
          WS2812_SetPixel(i, r, g, b);
        }
        WS2812_Show();
        phase = (phase + 1) % 7;
        osDelay(50);
      }
      running_rainbow = 0;
    }
    else
    {
      /* ===== 闪烁灯模式，持续 4s ===== */
      switch (mode % 4)
      {
        case 0:
          /* 全灭/全亮交替 */
          for (uint16_t j = 0; j < 20; j++)
          {
            WS2812_SetAll(255, 255, 255);
            WS2812_Show();
            osDelay(20);
            WS2812_SetAll(0, 0, 0);
            WS2812_Show();
            osDelay(20);
          }
          break;
          
        case 1:
          /* 单色轮转：红→绿→蓝→白 */
          {
            static const uint8_t colors[4][3] = {{255,0,0},{0,255,0},{0,0,255},{255,255,255}};
            for (uint8_t c = 0; c < 4; c++)
            {
              WS2812_SetAll(colors[c][0], colors[c][1], colors[c][2]);
              WS2812_Show();
              osDelay(500);
            }
          }
          break;
          
        case 2:
          /* 红绿交替 */
          for (uint8_t s = 0; s < 4; s++)
          {
            if (s & 1)
            {
              WS2812_SetPixel(0, 255, 0, 0);
              WS2812_SetPixel(1, 255, 0, 0);
              WS2812_SetPixel(2, 0, 255, 0);
              WS2812_SetPixel(3, 0, 255, 0);
            }
            else
            {
              WS2812_SetPixel(0, 0, 255, 0);
              WS2812_SetPixel(1, 0, 255, 0);
              WS2812_SetPixel(2, 255, 0, 0);
              WS2812_SetPixel(3, 255, 0, 0);
            }
            WS2812_Show();
            osDelay(300);
          }
          break;
          
        default:
          /* 对角分组闪烁 */
          for (uint8_t g = 0; g < 4; g++)
          {
            if (g & 1)
            {
              WS2812_SetPixel(0, 0, 255, 255);
              WS2812_SetPixel(3, 0, 255, 255);
            }
            else
            {
              WS2812_SetPixel(1, 255, 0, 255);
              WS2812_SetPixel(2, 255, 0, 255);
            }
            WS2812_Show();
            osDelay(400);
          }
          break;
      }
      
      mode++;
      running_rainbow = 1;
    }
  }
}

/**
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

