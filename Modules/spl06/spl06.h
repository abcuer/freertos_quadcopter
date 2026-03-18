#ifndef _SPL06_H
#define _SPL06_H

#include "bsp_iic.h"
#include <stdint.h>

// SPL06 I2C地址
#define SPL06_ADDR          0x77

// SPL06 主要寄存器地址
#define SPL06_PRS_B2        0x00
#define SPL06_TMP_B2        0x03
#define SPL06_PRS_CFG       0x06
#define SPL06_TMP_CFG       0x07
#define SPL06_MEAS_CFG      0x08
#define SPL06_CFG_REG       0x09
#define SPL06_ID_REG        0x0D

// 校准参数结构体
typedef struct {
    int16_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
} SPL06_Calib_t;

// SPL06 数据结构体
typedef struct {
    float temperature;
    float pressure;
    float height;
    float altitude;
    uint8_t chip_id;
    int32_t raw_pressure;
    int32_t raw_temperature;
    int32_t kp;
    int32_t kt;
    SPL06_Calib_t calib;
} SPL_t;

extern SPL_t spl;

// 函数声明
uint8_t SPL06_Init(void);
void SPL06_Update(void);
float SPL06_Get_Height(void);

#endif