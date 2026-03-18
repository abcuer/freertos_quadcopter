#ifndef __CONTROL_H_
#define __CONTROL_H_
#include "bmi088.h"
#include "remote.h"

void PIDParam_Init(void);
void Flight_Calculate_PID(Gyro_Acc_Struct *gyro_acc, EulerAngle_Struct *euler_angle, Remote_Data_Struct *rc_data, float dt);
void FlyControl(void);

extern Gyro_Acc_Struct gyro_acc;
extern EulerAngle_Struct euler_angle;

#endif