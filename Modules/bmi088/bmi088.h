#ifndef __BMI088_H
#define __BMI088_H

#include "stdint.h"

/* BMI088 I2C 地址 */
#define BMI088_ACC_ADDR     0x19    
#define BMI088_GYRO_ADDR    0x69    

/* 加速度计寄存器 */
#define ACC_CHIP_ID         0x00
#define ACC_CONF            0x40
#define ACC_RANGE           0x41
#define ACC_PWR_CONF        0x7C
#define ACC_PWR_CTRL        0x7D
#define ACC_SOFTRESET       0x7E

/* 陀螺仪寄存器 */
#define GYRO_CHIP_ID        0x00
#define GYRO_RANGE          0x0F
#define GYRO_BANDWIDTH      0x10
#define GYRO_LPM1           0x11
#define GYRO_SOFTRESET      0x14

#define ABS(x) ((x) > 0 ? (x) : -(x)) // 绝对值

// 或者在 bmi088.c 开头定义
#define INSTALL_ORIENTATION  1      // 0:正面安装，1:背面安装

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
}Acc_Struct;

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
}Gyro_Struct;

typedef struct
{
    Acc_Struct acc;  
    Gyro_Struct gyro;  
}Gyro_Acc_Struct;
typedef struct
{
    float roll;
    float pitch;
    float yaw;
}EulerAngle_Struct;

uint8_t BMI088_Init(void);
uint8_t BMI088_Read(Gyro_Acc_Struct *gyro_acc);
void BMI088_Convert_To_Physical(Gyro_Acc_Struct *gyro_acc, 
                                Acc_Struct *gyro_dps_f,  // 用于存储浮点角速度
                                Acc_Struct *acc_g_f);     // 用于存储浮点加速度
void BMI088_Calibrate(void);
uint8_t BMI088_Verify_Installation(void);
#endif