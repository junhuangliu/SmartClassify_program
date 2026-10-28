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

