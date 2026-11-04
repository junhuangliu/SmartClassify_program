#include "WaterPump.h"

/**
 * @brief 初始化水泵/阀门 PWM 输出，并保证上电时为关闭状态
 *
 * @details 详细思路：
 * - 设计意图：把 TIM4 两路 PWM 通道（阀门 CH2、水泵 CH1）置零启动，
 *   避免上电瞬间因比较值为随机/默认值导致阀门或水泵误动作
 * - 实现要点：先启动计数器，再分别启动两路 PWM，最后把两路比较值写 0
 * - 数据流：直接操作 htim4 的比较寄存器占空比，无返回值
 * - 注意事项：必须在 MX_TIM4_Init 完成（含 MSP 的 GPIO 复用配置）之后调用
 *
 * @param 无
 * @retval 无
 */
void WaterPump_Init(void){

HAL_TIM_Base_Start(&htim4);
HAL_TIM_PWM_Start(&htim4, PUMP_CH);
HAL_TIM_PWM_Start(&htim4, VALVE_CH);
__HAL_TIM_SET_COMPARE(&htim4, VALVE_CH, 0);
__HAL_TIM_SET_COMPARE(&htim4, PUMP_CH, 0);


}

/**
 * @brief 打开水路：先开阀门再开水泵
 *
 * @details 详细思路：
 * - 设计意图：先建立通路再启泵，防止先启泵憋压
 * - 实现要点：阀门占空比拉到 VALVE_DUTY_ON，延时 100ms 等阀门到位后再以 PUMP_DUTY_ON 启泵
 * - 注意事项：内部使用 osDelay，只能在 FreeRTOS 任务上下文调用，不能在中断里调用
 *
 * @param 无
 * @retval 无
 */
void WaterPump_Start(void){
__HAL_TIM_SET_COMPARE(&htim4, VALVE_CH, 0);
osDelay(200);
__HAL_TIM_SET_COMPARE(&htim4, PUMP_CH, PUMP_DUTY_ON);
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
__HAL_TIM_SET_COMPARE(&htim4, VALVE_CH, VALVE_DUTY_ON);
osDelay(100);
__HAL_TIM_SET_COMPARE(&htim4, VALVE_CH, 0);

}
