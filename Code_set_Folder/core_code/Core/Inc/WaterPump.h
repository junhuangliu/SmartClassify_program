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

/* 水泵默认占空比百分比；运行期可通过 UART1 的 Waterpump,<0~95>/ 帧修改。
   实测该 PWM 电子开关 100% 会恒定高电平导致泵不转，故上限取 95% */
#define WATER_PUMP_DUTY_DEFAULT 90U
#define WATER_PUMP_DUTY_MAX     95U

/* 关闭水路时阀门需要的驱动占空比：与水泵速度解耦，避免把水泵占空比调低后关不严阀 */
#define WATER_PUMP_VALVE_DUTY_PERCENT 67U

/* 运行期可变的水泵占空比（0~95），由 UART1 解析任务直接写入 */
extern volatile uint8_t WaterPump_DutyPercent;

/* 命令标志位：Uart1RxTask 置位，WaterPumpTask 消费后清零，实际 PWM 操作只在任务里做 */
extern volatile uint8_t WaterPump_OnFlag;
extern volatile uint8_t WaterPump_OffFlag;
extern volatile uint8_t WaterPump_DutyUpdateFlag;


/* 占空比按百分比换算，ARR 变化时输出自动跟随；
   PUMP 通道使用运行期占空比，VALVE 通道固定用关阀占空比 */
#define WATER_PUMP_DUTY_CCR \
  ((uint32_t)(((__HAL_TIM_GET_AUTORELOAD(&htim4) + 1U) * (uint32_t)WaterPump_DutyPercent) / 100U))
#define WATER_PUMP_VALVE_CCR \
  ((uint32_t)(((__HAL_TIM_GET_AUTORELOAD(&htim4) + 1U) * WATER_PUMP_VALVE_DUTY_PERCENT) / 100U))

void WaterPump_Init(void);
void WaterPump_Start(void);
void WaterPump_Stop(void);


/**
 * @brief 把当前 WaterPump_DutyPercent 下发到水泵 PWM 通道
 *
 * @details
 * - 设计意图：运行期修改占空比时无需重启水泵，直接刷新 CCR 即可改变转速
 * - 实现要点：仅在泵处于运行状态时写 CCR，泵未启动时不写，避免把停着的泵误开
 * - 注意事项：内部无阻塞延时，可在任意任务上下文调用；由 WaterPumpTask 消费标志位时调用
 *
 * @param 无
 * @retval 无
 */
void WaterPump_ApplyDuty(void);

#endif /* WATERPUMP_H */
