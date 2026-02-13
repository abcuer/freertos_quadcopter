#ifndef __CONTROL_H_
#define __CONTROL_H_
#include "bmi088.h"

#define Limit(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

void PIDParam_Init(void);
void FlyControl(void);

extern Gyro_Acc_Struct gyro_acc;
extern EulerAngle_Struct euler_angle;

#endif