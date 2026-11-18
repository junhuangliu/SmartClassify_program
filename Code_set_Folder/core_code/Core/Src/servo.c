#include "servo.h"
uint8_t Servo_Flag1;
uint8_t Servo_Angle1;
float duty;

void Servo_Init(void){
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);	
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);	
 
}

void ServoTask(void *argument){
__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,19999U*(45/1800.0f+0.025));;

  for(;;){
    /********舵机测试 */
    #if 0
   Servo_SetAngle(90.0f,TIM_CHANNEL_3);
    osDelay(2000U);
   Servo_SetAngle(0.0f,TIM_CHANNEL_3);
    osDelay(2000U);
    #endif
if(Servo_Flag1==1){
Servo_Flag1=0;
Servo_SetAngle(Servo_Angle1, TIM_CHANNEL_3);

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
