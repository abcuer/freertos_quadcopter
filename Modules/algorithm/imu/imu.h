#ifndef __IMU_H__
#define __IMU_H__
#include "bmi088.h"

#define PI          3.1415926535f
#define RTOM        0.0174532925f  // 度转弧度 (PI/180)
#define MTOR        57.29577951f   // 弧度转度 (180/PI)
#define squa(Sq) (((float)Sq) * ((float)Sq)) /* 计算平方 */
/* 表示四元数的结构体 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

extern float RtA;
extern float Gyro_G;
extern float Gyro_Gr;

void IMU_Get_GyroAcc(Gyro_Acc_Struct *gyro_acc);
void IMU_Get_EulerAngle(Gyro_Acc_Struct  *gyroAccel,
                        EulerAngle_Struct *eulerAngle,
                        float              dt);
float IMU_GetNormAccZ(void);

#endif /* __COM_IMU_H__ */
