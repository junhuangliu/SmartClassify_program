#ifndef WATERPUMP_H
#define WATERPUMP_H
#include "main.h"
#include "cmsis_os.h"

#define PUMP_CH    TIM_CHANNEL_1
#define VALVE_CH   TIM_CHANNEL_2

/* TIM4（水泵 CH1 / 阀门 CH2 / 舵机 CH4 共用）统一按 50Hz 输出：
   计数时钟 = APB1 定时器时钟 = 60MHz，PSC=59 -> 1us 计数，ARR=19999 -> 20ms 周期。
   时基只在 WaterPump_Init 里配置一次，舵机不再重复修改。 */
#define PUMP_PWM_PRESCALER 59U
#define PUMP_PWM_PERIOD   19999U

/* TIM4 句柄定义在 main.c，此处仅作外部声明供 WaterPump.c 使用 */
extern TIM_HandleTypeDef htim4;

/* 占空比按百分比换算，ARR 变化时输出自动跟随；
   PUMP/VALVE 目标占空比约 88%（50Hz 下比较值 17600） */
#define WATER_PUMP_DUTY_PERCENT 67U
#define WATER_PUMP_DUTY_CCR \
  ((uint32_t)(((__HAL_TIM_GET_AUTORELOAD(&htim4) + 1U) * WATER_PUMP_DUTY_PERCENT) / 100U))

void WaterPump_Init(void);
void WaterPump_Start(void);
void WaterPump_Stop(void);

#endif /* WATERPUMP_H */
