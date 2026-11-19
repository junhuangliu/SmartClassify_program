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
#include "uart1_proto.h"
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
 * @brief WS2812 上电固定白光任务
 * 
 * @details
 * - 设计意图：上电后灯带保持固定白色常亮，不做闪烁或流水效果
 * - 实现要点：任务启动时设置全部灯珠为白色并刷新一次，之后仅延时让出 CPU
 * - 数据流：操作 RGB 缓冲区后由 WS2812_Show() 编码并通过 SPI1 发送
 * - 注意事项：只需刷新一次，WS2812 会自行锁存显示，无需周期性重发
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
void WaterPumpTask(void *argument);

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief USART1 上位机命令处理任务
 * 
 * @details
 * - 设计意图：把串口收到的数据交给 uart1_proto，分别处理水泵开关和 CoreXY 命令
 * - 实现要点：循环从 uart1_rx_queue 取数据块，调用 Uart1Proto_Feed 做按行解析
 * - 注意事项：
 *   1. 解析/动作在任务上下文执行，ISR 只负责入队，符合项目约束
 *   2. switch 命令只调用原有水泵启停函数；init 命令只控制 CoreXY
 *   3. 队列元素是数据拷贝，不依赖 DMA 缓冲区生命周期
 * 
 * @param argument 未使用
 * @retval 无
 */
void Uart1RxTask(void *argument)
{
  (void)argument;
  osMessageQueueId_t rxQueue = Uart1Comm_GetRxQueueHandle();
  Uart1RxMsg_t msg;

  /* 上电后主动发送一条启动提示，便于确认串口发送通路正常 */
  Uart1Proto_Send("SmartClassify UART1 ready\r\n");

  if (rxQueue == NULL)
  {
    Uart1Proto_Send("ERR UART1 QUEUE\r\n");
    for (;;)
    {
      osDelay(1000);
    }
  }

  for (;;)
  {
    if (osMessageQueueGet(rxQueue, &msg, NULL, osWaitForever) == osOK)
    {
      Uart1Proto_Feed(msg.data, msg.len);
    }
  }
}

/**
 * @brief WS2812 上电固定白光任务实现
 * 
 * @details
 * - 设计意图：上电后灯带固定白色常亮，不做闪烁或流水灯效果
 * - 实现要点：设置全部灯珠为白色并刷新一次，随后进入延时循环
 * - 数据流：仅操作 RGB 缓冲区，由 WS2812_Show() 统一编码并通过 SPI1 发送
 * - 注意事项：
 *   1. WS2812 自身锁存颜色，刷新一次即可保持，不需要周期性重发
 *   2. 循环内必须 osDelay 让出 CPU，否则会饿死低优先级任务
 * 
 * @param argument 未使用
 * @retval 无
 */
void WS2812Task(void *argument)
{
  (void)argument;

  /* 上电即点亮全白，刷新一次后保持常亮 */
  WS2812_SetAll(255, 255, 255);
  WS2812_Show();

  for (;;)
  {
    osDelay(1000U);
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

/**
 * @brief 水泵任务入口，供后续添加水泵控制逻辑
 *
 * @details 详细思路：
 * - 设计意图：保留独立的水泵任务循环，不与 CoreXY 电机任务联动
 * - 实现要点：占位阶段每轮 osDelay(100) 主动让出 CPU，绝不能写空转死循环
 * - 注意事项：本任务优先级为 osPriorityNormal(24)。若循环体不阻塞，它永远处于就绪态，
 *   抢占式调度下所有低优先级任务（如 osPriorityBelowNormal(16) 的 ServoTask）将永远拿不到时间片，
 *   表现为舵机等低优先级任务完全不执行；同时空闲任务也跑不到，CPU 占用 100%
 * - 调用关系：仅由 main.c 的 osThreadNew 创建，运行在任务上下文
 *
 * @param argument 未使用
 * @retval 无（任务不返回）
 */
void WaterPumpTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    osDelay(100U);
  }
}

/* USER CODE END Application */

