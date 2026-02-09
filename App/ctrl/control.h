#ifndef __CONTROL_H_
#define __CONTROL_H_
#include "bmi088.h"

void PIDParam_Init(void);
void IMU_Get_Gyro_Acc(Gyro_Acc_Struct *gyro_acc);
void IMU_Get_EulerAngle(Gyro_Acc_Struct *gyro_acc, EulerAngle_Struct *euler_angle, float dt);

extern Gyro_Acc_Struct gyro_acc;
extern EulerAngle_Struct euler_angle;

#endif