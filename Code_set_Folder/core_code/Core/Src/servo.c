#include "servo.h"
float duty;

void Servo_Init(void){
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  //HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);	
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);	
 // Servo_SetAngle(0.0f);
}

void ServoTask(void *argument){
  for(;;){
    Servo_SetAngle(90.0f);
    osDelay(2000U);
    Servo_SetAngle(0.0f);
    osDelay(2000U);
  }
}

void Servo_SetAngle(float Angle){
if(Angle>=0 && Angle<=180.0f){
duty=Angle/1800.0f+0.025;
//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,19999U*duty);	
__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,19999U*duty);
}
else Servo_SetAngle(0);

}
