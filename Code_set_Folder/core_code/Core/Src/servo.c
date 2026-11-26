#include "servo.h"

 volatile uint8_t Servo1_FallFlag=0;
 volatile uint8_t Servo1_RiseFlag=0;
 volatile uint8_t Servo2_OpenFlag=0;
 volatile uint8_t Servo2_CloseFlag=0;
float duty;

/* 舵机测试模式：
   0 = 关闭测试，恢复 Servo_Flag1 逻辑（抓取流程通过 Servo_SetPulseUs 控制舵机1）
   1 = 只测舵机1（CH3 / PD14）：调用 Servo1_MoveTeeth(+5/-5) 验证齿数接口
   2 = 只测舵机2（CH4 / PD15，360° 连续旋转舵机，1.0ms 正转 / 1.5ms 停 / 2.0ms 反转）
   3 = 两路同时测

   两路都基于 1us 计数（PSC=59, ARR=19999），CCR 直接等于脉宽微秒数 */
#define SERVO_TEST_MODE          0
#define SERVO_TEST_INTERVAL_MS   1500U

/* 180° 位置舵机脉宽（us）：
   测试用 0.5ms <-> 2.5ms 全行程，便于肉眼确认舵机是否能正常转动 */
#define SERVO180_PULSE_MIN_US    500U
#define SERVO180_PULSE_MAX_US    2500U

/* 360° 连续旋转舵机脉宽（us） */
#define SERVO360_PULSE_CW_US     1000U
#define SERVO360_PULSE_STOP_US   1500U
#define SERVO360_PULSE_CCW_US    2000U

/* 舵机2 齿轮传动：输出齿 16 齿，需要转 10 齿后回转 10 齿
   连续旋转舵机没有角度反馈，只能按「整圈时间 × 齿数比例」估算运行时长；
   实测 1800ms/圈 × 9/16（1012ms）只走了 8 齿，折合整圈约 2025ms，
   改按 10 齿校准，单次约 1266ms */
#define SERVO2_GEAR_TEETH_TOTAL  16U
#define SERVO2_GEAR_TEETH_MOVE   10U
#define SERVO2_FULL_REV_MS       2025U
#define SERVO2_MOVE_MS \
  ((SERVO2_FULL_REV_MS * SERVO2_GEAR_TEETH_MOVE) / SERVO2_GEAR_TEETH_TOTAL)

/* 舵机1 位置换算：1.5ms 为基准位，实测 5 齿 ≈ 833us，即 1 齿 ≈ 167us */
#define SERVO1_HOME_PULSE_US       1500U
#define SERVO1_PULSE_PER_TOOTH_US  167
#define SERVO1_MOVE_SETTLE_MS      1500U

/* 舵机2 往返测试停顿时长 */
#define SERVO360_TEST_STOP_MS    300U

/**
 * @brief 舵机2 执行「打开」动作：反转 10 个齿距（16 齿齿轮，约 225°）后停止
 *
 * @details
 * - 设计意图：
 *   把机构打开动作封装成独立函数，业务层只需调用 Servo2_Open()，
 *   不用关心脉宽、运行时间和停止时序
 * - 实现要点：
 *   1. 连续旋转舵机没有位置反馈，无法直接指定角度，只能按运行时间估算
 *   2. 先输出反转脉宽 2.0ms，保持 SERVO2_MOVE_MS 后输出 1.5ms 停止脉宽
 *   3. SERVO2_MOVE_MS = 整圈时间 SERVO2_FULL_REV_MS × 10/16，当前约 1266ms
 * - 数据流：无输入；输出为 TIM4 CH4（PD15）的 PWM 脉宽序列
 * - 注意事项：
 *   1. 内部使用 osDelay，只能在 FreeRTOS 任务上下文调用，禁止在中断里调用
 *   2. 实际转角受供电电压、负载和舵机个体差异影响，偏大/偏小改 SERVO2_FULL_REV_MS
 *   3. 必须与 Servo2_Return() 成对使用，中间不要插入其它舵机2 动作
 * - 调用关系：测试模式下循环调用；后续抓取/放置流程可直接复用
 *
 * @param 无
 * @retval 无
 */
void Servo2_Open(void)
{
  Servo_SetPulseUs(SERVO360_PULSE_CCW_US, TIM_CHANNEL_4);
  osDelay(SERVO2_MOVE_MS);
  Servo_SetPulseUs(SERVO360_PULSE_STOP_US, TIM_CHANNEL_4);
}

/**
 * @brief 舵机2 执行「回位」动作：正转 10 个齿距，回到 Servo2_Open() 之前的位置
 *
 * @details
 * - 设计意图：
 *   与 Servo2_Open() 配对的复位动作，保证机构每次打开后都能回到初始位置
 * - 实现要点：
 *   1. 输出正转脉宽 1.0ms，运行与打开完全相同的时间 SERVO2_MOVE_MS
 *   2. 时间相同即可近似转回原位，最后输出 1.5ms 停止脉宽，避免持续堵转
 * - 数据流：无输入；输出为 TIM4 CH4（PD15）的 PWM 脉宽序列
 * - 注意事项：
 *   1. 内部使用 osDelay，只能在 FreeRTOS 任务上下文调用
 *   2. 打开和回位之间存在累计误差，多次循环后如有偏移需重新上电或手动校正
 *   3. 不要在两次调用之间改动 SERVO2_FULL_REV_MS，否则回位会不匹配
 * - 调用关系：必须紧跟在 Servo2_Open() 之后使用；测试模式下循环调用
 *
 * @param 无
 * @retval 无
 */
void Servo2_Return(void)
{
  Servo_SetPulseUs(SERVO360_PULSE_CW_US, TIM_CHANNEL_4);
  osDelay(SERVO2_MOVE_MS);
  Servo_SetPulseUs(SERVO360_PULSE_STOP_US, TIM_CHANNEL_4);
}

/* 舵机1 当前指令脉宽，用于按齿数增量累加（上电默认在 1.5ms 基准位） */
static int32_t servo1_pulse_us = (int32_t)SERVO1_HOME_PULSE_US;

/**
 * @brief 舵机1 按齿数转动：正数增大脉宽，负数减小脉宽
 *
 * @details
 * - 设计意图：
 *   舵机1 是 180° 位置舵机，角度由脉宽决定；业务层只关心「转几个齿」，
 *   这里统一把齿数换算成脉宽增量，避免各处手写 1.5ms/2.333ms 之类魔法值
 * - 实现要点：
 *   1. 实测 5 齿 ≈ 833us，即 1 齿 ≈ 167us
 *   2. 以内部记录的当前脉宽为基准做增量：pulse += teeth × 167
 *   3. 结果先钳位到 500~2500us，再调用 Servo_SetPulseUs 输出
 *   4. 等待 SERVO1_MOVE_SETTLE_MS 让舵机走完，再更新内部记录
 * - 数据流：teeth（齿数，带符号）-> 目标脉宽 -> TIM4 CH3（PD14）
 * - 注意事项：
 *   1. 内部使用 osDelay，只能在 FreeRTOS 任务上下文调用
 *   2. 超出 ±5 齿左右就会被 500/2500us 钳位，无法到达
 *   3. 若其它代码直接写 CH3，内部记录会失配，需要重新调用本函数前先回基准位
 * - 调用关系：测试模式 1 用 +5 / -5 验证；抓取流程可直接复用
 *
 * @param teeth 要转动的齿数，正数增大脉宽（实机为上升），负数下降；范围约 -5~+5
 * @retval 无
 */
void Servo1_MoveTeeth(int8_t teeth)
{
  int32_t target = servo1_pulse_us + (int32_t)teeth * SERVO1_PULSE_PER_TOOTH_US;

  if (target < 500)
  {
    target = 500;
  }
  if (target > 2500)
  {
    target = 2500;
  }

  Servo_SetPulseUs((uint32_t)target, TIM_CHANNEL_3);
  osDelay(SERVO1_MOVE_SETTLE_MS);
  servo1_pulse_us = target;
}

/**
 * @brief 直接按脉宽（微秒）驱动指定 TIM4 通道
 *
 * @details
 * - 设计意图：位置舵机用绝对脉宽控制更直观，抓取流程需要精确的上升/下降位置
 * - 实现要点：TIM4 已由 WaterPump_Init 配成 1us 计数、20ms 周期，CCR 数值即脉宽微秒数
 * - 注意事项：可在任务上下文直接调用；pulse_us 会被钳位到 300~2500us
 *   （下限放宽到 300us 以增大抓取下降行程，长期顶限位会堵转发热）
 *
 * @param pulse_us 高电平脉宽（微秒）
 * @param channel  TIM4 通道（TIM_CHANNEL_3 或 TIM_CHANNEL_4）
 * @retval 无
 */
void Servo_SetPulseUs(uint32_t pulse_us, uint32_t channel)
{
  if (pulse_us < 300U)
  {
    pulse_us = 300U;
  }
  if (pulse_us > 2500U)
  {
    pulse_us = 2500U;
  }
  __HAL_TIM_SET_COMPARE(&htim4, channel, pulse_us);
}

void Servo_Init(void){
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
}

void ServoTask(void *argument){
  
 Servo_SetAngle(70, TIM_CHANNEL_3);
for(;;){

if(Servo1_FallFlag){
  Servo1_FallFlag=0;
  Servo1_MoveTeeth(-5);
}
 
if(Servo1_RiseFlag){
  Servo1_RiseFlag=0;
  Servo1_MoveTeeth(5);

}
if(Servo2_OpenFlag){
  
Servo2_Open();
osDelay(3000);
Servo2_CloseFlag=1;
Servo2_OpenFlag=0;
}
if(Servo2_CloseFlag){
Servo2_Return();
Servo2_CloseFlag=0;
}
osDelay(20);
}







}

void Servo_SetAngle(float Angle, uint32_t CHANNEL){
if(Angle>=0 && Angle<=180.0f){
duty=Angle/1800.0f+0.025;
__HAL_TIM_SET_COMPARE(&htim4,CHANNEL,19999U*duty);	

}
else Servo_SetAngle(0, CHANNEL);

}
