#ifndef __BMI088_H
#define __BMI088_H

#include "stdint.h"

/* BMI088 I2C 地址 */
#define BMI088_ACC_ADDR     0x19    /* 0x18 0x19 0x30 0x32  ==0x1E*/ 
#define BMI088_GYRO_ADDR    0x69    /* 0x68 0x69 0xD0 0xD2  ==0x0F*/ 

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

typedef struct
{
    int16_t acc_raw[3];  // 加速度计原始值
    int16_t gyro_raw[3];  // 陀螺仪原始值
    float acc[3];        
    float gyro[3];       
    float roll, pitch, yaw;
}IMU_t;

uint8_t BMI088_Init(void);
void BMI088_GetData(IMU_t *imu);

#endif