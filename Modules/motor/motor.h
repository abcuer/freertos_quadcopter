#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f1xx_hal.h"

void Motor_Init(void);
void SetMotorPWM(float m1, float m2, float m3, float m4);
void MotorTest(void);
void MotorLock(void);
#endif