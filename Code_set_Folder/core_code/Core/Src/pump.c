/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pump.c
  * @brief          : 水泵控制模块实现（TIM4 CH1 运行 / CH2 停止脉冲）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "pump.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim4;

static uint8_t pump_on = 0U;
static uint8_t pump_duty = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static uint32_t Pump_DutyToCcr(uint8_t duty_percent);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 占空比百分比转换为 CCR 比较值
 *
 * @details
 * - 实现要点：CCR = (ARR + 1) * duty / 100，并限制在 ARR 以内
 * - 注意事项：duty_percent 超过 PUMP_MAX_DUTY 时按上限处理，避免 CCR > ARR
 *   造成 PWM 输出恒定高电平、驱动失去开关沿
 *
 * @param duty_percent 占空比 0~100（%）
 * @retval CCR 比较值
 */
static uint32_t Pump_DutyToCcr(uint8_t duty_percent)
{
  uint32_t ccr;

  if (duty_percent > PUMP_MAX_DUTY)
  {
    duty_percent = PUMP_MAX_DUTY;
  }

  ccr = (((uint32_t)PUMP_PWM_ARR + 1U) * (uint32_t)duty_percent) / 100U;
  if (ccr > PUMP_PWM_ARR)
  {
    ccr = PUMP_PWM_ARR;
  }

  return ccr;
}

/**
 * @brief 初始化水泵控制
 *
 * @details
 * - 设计意图：把 TIM4 的两路 PWM 固定为“运行 + 停止脉冲”的硬件动作
 * - 实现要点：
 *   1. 覆盖 ARR 为 20kHz 周期（PSC 保持 0，TIM4 时钟 60MHz）
 *   2. 两路 CCR 先清 0，保证上电默认关泵
 *   3. 启动 CH1/CH2 的 PWM 输出
 * - 注意事项：调用前必须完成 MX_TIM4_Init()，否则通道未配置为 PWM 模式
 *
 * @param 无
 * @retval 无
 */
void Pump_Init(void)
{
  __HAL_TIM_SET_PRESCALER(&htim4, PUMP_PWM_PSC);
  __HAL_TIM_SET_AUTORELOAD(&htim4, PUMP_PWM_ARR);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  __HAL_TIM_SET_COMPARE(&htim4, PUMP_RUN_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, PUMP_STOP_CHANNEL, 0U);

  HAL_TIM_PWM_Start(&htim4, PUMP_RUN_CHANNEL);
  HAL_TIM_PWM_Start(&htim4, PUMP_STOP_CHANNEL);

  pump_on = 0U;
  pump_duty = PUMP_RUN_DUTY_DEFAULT;
}

/**
 * @brief 开泵
 *
 * @details
 * - 实现要点：先确保停止通道 CH2 为 0，再按已保存的占空比写 CH1 的 CCR
 * - 注意事项：占空比由 Pump_SetDuty() 预先保存，默认 50%
 *
 * @param 无
 * @retval 无
 */
void Pump_On(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, PUMP_STOP_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, PUMP_RUN_CHANNEL, Pump_DutyToCcr(pump_duty));

  pump_on = 1U;
}

/**
 * @brief 设置运行占空比
 *
 * @details
 * - 实现要点：运行中立即更新 CH1 的 CCR；关闭时只保存，等下次开泵生效
 *
 * @param duty_percent 目标占空比 0~100（%），超过 PUMP_MAX_DUTY 按上限处理
 * @retval 无
 */
void Pump_SetDuty(uint8_t duty_percent)
{
  if (duty_percent > PUMP_MAX_DUTY)
  {
    duty_percent = PUMP_MAX_DUTY;
  }

  pump_duty = duty_percent;

  if (pump_on != 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim4, PUMP_RUN_CHANNEL, Pump_DutyToCcr(pump_duty));
  }
}

/**
 * @brief 关泵
 *
 * @details
 * - 设计意图：按硬件要求用 CH2 的短脉冲完成停止动作
 * - 实现要点：CH1 先回 0，随后 CH2 输出停止占空比，保持一段时间后回 0
 * - 注意事项：内部使用 osDelay，只能在 FreeRTOS 任务中调用，不可在 ISR 中调用；
 *   关泵后保留当前占空比，方便下次开泵沿用
 *
 * @param 无
 * @retval 无
 */
void Pump_Off(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, PUMP_RUN_CHANNEL, 0U);

  __HAL_TIM_SET_COMPARE(&htim4, PUMP_STOP_CHANNEL, Pump_DutyToCcr(PUMP_STOP_DUTY));
  osDelay(PUMP_STOP_PULSE_MS);
  __HAL_TIM_SET_COMPARE(&htim4, PUMP_STOP_CHANNEL, 0U);

  pump_on = 0U;
}

/**
 * @brief 查询水泵是否处于开启状态
 *
 * @param 无
 * @retval 1 开启，0 关闭
 */
uint8_t Pump_IsOn(void)
{
  return pump_on;
}

/**
 * @brief 查询最近一次设置的运行占空比
 *
 * @param 无
 * @retval 占空比 0~100（%）
 */
uint8_t Pump_GetDuty(void)
{
  return pump_duty;
}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
