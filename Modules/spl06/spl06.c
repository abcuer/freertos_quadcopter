#include "spl06.h"
#include "kalman.h"
#include "bsp_delay.h"
#include <math.h>
#include "led.h"

SPL_t spl;

static iic_bus_t spl06_bus = {
    .IIC_SDA_PORT = SDA_GPIO_Port,
    .IIC_SDA_PIN = SDA_Pin,
    .IIC_SCL_PORT = SCL_GPIO_Port,
    .IIC_SCL_PIN = SCL_Pin
};

// 底层写寄存器封装
static uint8_t spl06_write_reg(uint8_t reg, uint8_t val) 
{
    return IIC_Write_One_Byte(&spl06_bus, SPL06_ADDR, reg, val);
}

// 底层多字节读封装
static uint8_t spl06_read_regs(uint8_t reg, uint8_t len, uint8_t* buf) 
{
    return IIC_Read_Multi_Byte(&spl06_bus, SPL06_ADDR, reg, len, buf);
}

// 获取校准参数
static void SPL06_Get_Calib_Param(void) 
{
    uint8_t buf[18];
    spl06_read_regs(0x10, 18, buf);

    spl.calib.c0 = (int16_t)buf[0] << 4 | buf[1] >> 4;
    spl.calib.c0 = (spl.calib.c0 & 0x0800) ? (0xF000 | spl.calib.c0) : spl.calib.c0;

    spl.calib.c1 = (int16_t)(buf[1] & 0x0F) << 8 | buf[2];
    spl.calib.c1 = (spl.calib.c1 & 0x0800) ? (0xF000 | spl.calib.c1) : spl.calib.c1;

    spl.calib.c00 = (int32_t)buf[3] << 12 | (int32_t)buf[4] << 4 | (int32_t)buf[5] >> 4;
    spl.calib.c00 = (spl.calib.c00 & 0x080000) ? (0xFFF00000 | spl.calib.c00) : spl.calib.c00;

    spl.calib.c10 = (int32_t)(buf[5] & 0x0F) << 16 | (int32_t)buf[6] << 8 | buf[7];
    spl.calib.c10 = (spl.calib.c10 & 0x080000) ? (0xFFF00000 | spl.calib.c10) : spl.calib.c10;

    spl.calib.c01 = (int16_t)buf[8] << 8 | buf[9];
    spl.calib.c11 = (int16_t)buf[10] << 8 | buf[11];
    spl.calib.c20 = (int16_t)buf[12] << 8 | buf[13];
    spl.calib.c21 = (int16_t)buf[14] << 8 | buf[15];
    spl.calib.c30 = (int16_t)buf[16] << 8 | buf[17];
}

// 内部采样率及系数计算
static void SPL06_Config_Rate(uint8_t sensor, uint8_t rate, uint8_t osr) 
{
    uint8_t reg = 0;
    int32_t k = 0;

    // 映射过采样率系数 (kP, kT)
    switch(osr) 
    {
        case 2:   reg |= 1; k = 1572864; break;
        case 4:   reg |= 2; k = 3670016; break;
        case 8:   reg |= 3; k = 7864320; break;
        case 16:  reg |= 4; k = 253952;  break;
        case 32:  reg |= 5; k = 516096;  break;
        case 64:  reg |= 6; k = 1040384; break;
        case 128: reg |= 7; k = 2088960; break;
        default:  reg |= 0; k = 524288;  break;
    }
    
    // 速率映射 (简化处理)
    reg |= (5 << 5); // 默认 32Hz

    if(sensor == 0) { // Pressure
        spl.kp = k;
        spl06_write_reg(SPL06_PRS_CFG, reg);
        if(osr > 8) spl06_write_reg(0x09, IIC_Read_One_Byte(&spl06_bus, SPL06_ADDR, 0x09) | 0x04);
    } else { // Temperature
        spl.kt = k;
        spl06_write_reg(SPL06_TMP_CFG, reg | 0x80);
        if(osr > 8) spl06_write_reg(0x09, IIC_Read_One_Byte(&spl06_bus, SPL06_ADDR, 0x09) | 0x08);
    }
}

// 初始化函数
uint8_t SPL06_Init(void) 
{
    IICInit(&spl06_bus);
    delay_ms(50);

    // 读取ID
    uint8_t id = IIC_Read_One_Byte(&spl06_bus, SPL06_ADDR, SPL06_ID_REG);
    if (id != 0x10) return 1; // 失败
    else 
    {
        SetLedMode(bLEDL, LED_ON);
        SetLedMode(bLEDR, LED_ON);
    }
    SPL06_Get_Calib_Param();
    SPL06_Config_Rate(0, 32, 32); // 气压: 32Hz, 32倍过采样
    SPL06_Config_Rate(1, 32, 8);  // 温度: 32Hz, 8倍过采样
    
    // 开启连续测量模式
    spl06_write_reg(SPL06_MEAS_CFG, 0x07); 
    
    return 0; // 成功
}

// 数据更新
void SPL06_Update(void) 
{
    uint8_t p_buf[3], t_buf[3];
    
    spl06_read_regs(SPL06_PRS_B2, 3, p_buf);
    spl06_read_regs(SPL06_TMP_B2, 3, t_buf);

    spl.raw_pressure = (int32_t)p_buf[0] << 16 | (int32_t)p_buf[1] << 8 | p_buf[2];
    if(spl.raw_pressure & 0x800000) spl.raw_pressure |= 0xFF000000;

    spl.raw_temperature = (int32_t)t_buf[0] << 16 | (int32_t)t_buf[1] << 8 | t_buf[2];
    if(spl.raw_temperature & 0x800000) spl.raw_temperature |= 0xFF000000;

    // 计算实际温度和气压
    float fTsc = spl.raw_temperature / (float)spl.kt;
    float fPsc = spl.raw_pressure / (float)spl.kp;

    spl.temperature = spl.calib.c0 * 0.5f + spl.calib.c1 * fTsc;
    
    float qua2 = spl.calib.c10 + fPsc * (spl.calib.c20 + fPsc * spl.calib.c30);
    float qua3 = fTsc * fPsc * (spl.calib.c11 + fPsc * spl.calib.c21);
    spl.pressure = spl.calib.c00 + fPsc * qua2 + fTsc * spl.calib.c01 + qua3;
}

// 获取高度（带简单气压公式）
// float SPL06_Get_Height(void) 
// {
//     // 采用标准海平面 101325 Pa 为基准
//     spl.height = 44330.0f * (1.0f - powf(spl.pressure / 101325.0f, 0.190295f));
//     return spl.height;
// }
float SPL06_Get_Height(void) 
{
    float alt_offset = (101000.0f - spl.pressure) / 1000.0f;
    spl.height = 0.82f * alt_offset * alt_offset * alt_offset + 0.09f * (101000.0f - spl.pressure) * 100.0f;
    return spl.height;
}

// 等待修改606行之后的程序