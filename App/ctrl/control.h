#ifndef __CONTROL_H_
#define __CONTROL_H_
#include "bmi088.h"
#include "remote.h"
#include "pid.h"

void PIDParam_Init(void);
void Flight_Calculate_PID(Gyro_Acc_Struct *gyro_acc, EulerAngle_Struct *euler_angle, RX_Data_Struct *rc_data, float dt);
void FlyControl(void);

extern Gyro_Acc_Struct gyro_acc;
extern EulerAngle_Struct euler_angle;

extern PID_Struct pid_gyro_y;

#endif