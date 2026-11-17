#include "WaterPump.h"

/**
 * @brief 初始化水泵/阀门 PWM 输出（50Hz），并保证上电时为关闭状态
 *
 * @details 详细思路：
 * - 设计意图：TIM4 由水泵 CH1、阀门 CH2、舵机 CH4 共用，时基统一在这里配置成 50Hz，
 *   其它模块不再重复改 PSC/ARR；先把比较值清 0 再开输出，避免上电瞬间误动作
 * - 实现要点：写 PSC/ARR 后必须产生更新事件让分频立即生效（PSC 带缓冲），
 *   然后清 CCR1/CCR2，再启动两路 PWM（HAL_TIM_PWM_Start 会同时使能计数器）
 * - 数据流：PSC/ARR -> 20ms 周期；CCR1/CCR2 -> 水泵/阀门占空比
 * - 注意事项：必须在 MX_TIM4_Init 之后、Servo_Init 之前调用
 *
 * @param 无
 * @retval 无
 */
void WaterPump_Init(void){
/* 50Hz 时基只配置这一次：1MHz 计数 + 20000 计数 = 20ms 周期 */
__HAL_TIM_SET_PRESCALER(&htim4, PUMP_PWM_PRESCALER);
__HAL_TIM_SET_AUTORELOAD(&htim4, PUMP_PWM_PERIOD);
htim4.Instance->EGR = TIM_EGR_UG;

__HAL_TIM_SET_COMPARE(&htim4, PUMP_CH, 0);
__HAL_TIM_SET_COMPARE(&htim4, VALVE_CH, 0);
HAL_TIM_PWM_Start(&htim4, PUMP_CH);
HAL_TIM_PWM_Start(&htim4, VALVE_CH);

}

/**
 * @brief 打开水路：先开阀门再开水泵
 *
 * @details 详细思路：
 * - 设计意图：先建立通路再启泵，防止先启泵憋压
 * - 实现要点：阀门占空比拉到 WATER_PUMP_DUTY_CCR，延时 100ms 等阀门到位后再以同一占空比启泵
 * - 注意事项：内部使用 osDelay，只能在 FreeRTOS 任务上下文调用，不能在中断里调用
 *
 * @param 无
 * @retval 无
 */
void WaterPump_Start(void){
__HAL_TIM_SET_COMPARE(&htim4, VALVE_CH, 0);
osDelay(200);
__HAL_TIM_SET_COMPARE(&htim4, PUMP_CH, WATER_PUMP_DUTY_CCR);
}

/**
 * @brief 关闭水路：先停泵再关阀门
 *
 * @details 详细思路：
 * - 设计意图：与 WaterPump_Start 顺序相反，先停泵泄压再关阀，避免管路残压
 * - 实现要点：水泵占空比先写 0，延时 100ms 后阀门写 0
 * - 注意事项：内部使用 osDelay，只能在 FreeRTOS 任务上下文调用，不能在中断里调用
 *
 * @param 无
 * @retval 无
 */
void WaterPump_Stop(void){
__HAL_TIM_SET_COMPARE(&htim4, PUMP_CH, 0);
osDelay(100);
__HAL_TIM_SET_COMPARE(&htim4, VALVE_CH, WATER_PUMP_DUTY_CCR);
osDelay(100);
__HAL_TIM_SET_COMPARE(&htim4, VALVE_CH, 0);

}
