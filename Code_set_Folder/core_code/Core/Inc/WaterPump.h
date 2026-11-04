#ifndef WATERPUMP_H
#define WATERPUMP_H
#include "main.h"
#include "cmsis_os.h"

#define PUMP_CH    TIM_CHANNEL_1
#define VALVE_CH   TIM_CHANNEL_2

/* TIM4 计数时钟 = APB1 定时器时钟 = 2 × PCLK1(30MHz) = 60MHz
   PWM 频率 = 60MHz / (PSC+1=1) / (ARR+1=30000) = 2kHz
   PUMP/VALVE 占空比 20000/30000 ≈ 67% */
#define PUMP_PWM_PERIOD   29999U
#define PUMP_DUTY_ON      20000U
#define VALVE_DUTY_ON     20000U

/* TIM4 句柄定义在 main.c，此处仅作外部声明供 WaterPump.c 使用 */
extern TIM_HandleTypeDef htim4;

void WaterPump_Init(void);
void WaterPump_Start(void);
void WaterPump_Stop(void);

#endif /* WATERPUMP_H */
