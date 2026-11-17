#ifndef __SERVO_H
#define __SERVO_H
#include "main.h"
#include "corexy.h"
#include "ws2812_spi.h"
#include "cmsis_os2.h"
#include "uart1_proto.h"
#include "turntable.h"
#include "Servo.h"
#include "WaterPump.h"
#include "Grab.h"

extern float duty;
void Servo_Init(void);
void Servo_SetAngle(float Angle);
void ServoTask(void *argument);
#endif
