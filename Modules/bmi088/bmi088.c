#include "bmi088.h"
#include "bsp_iic.h"
#include "bsp_delay.h"
#include "kalman.h"
#include "gpio.h"

// 静态定义总线对象（关联硬件引脚）
static iic_bus_t bmi088_bus = {
    .IIC_SDA_PORT = SDA_GPIO_Port,
    .IIC_SDA_PIN = SDA_Pin,
    .IIC_SCL_PORT = SCL_GPIO_Port,
    .IIC_SCL_PIN = SCL_Pin
};

/**
 * @brief  BMI088 初始化 (bsp_iic 风格)
 * @return 0:成功, 1:ACC错误, 2:GYRO错误, 3:配置校验失败
 */
uint8_t BMI088_Init(void)
{
    uint8_t status = 0;
    uint8_t read_val = 0;
    IICInit(&bmi088_bus);
    // --- 1. 加速度计初始化 ---
    // 软复位
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_SOFTRESET, 0xB6);
    delay_ms(50);
    
    // 开启电源 (顺序极其关键)
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_PWR_CTRL, 0x04);
    delay_ms(10); 
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00);
    delay_ms(10);

    // 校验 ACC ID
    if(IIC_Read_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CHIP_ID)!= 0x1E) return 1;

    // 配置 ACC：±6g, ODR 400Hz, OSR4 滤波
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_RANGE, 0x01);
    // IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CONF, 0x8A);
      IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CONF, 0x88);
    
    // 验证配置是否写入成功
    if(IIC_Read_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CONF) != 0x8A) return 3;

    // --- 2. 陀螺仪初始化 ---
    // 软复位
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_SOFTRESET, 0xB6);
    delay_ms(50);

    // 校验 GYRO ID
    if(IIC_Read_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_CHIP_ID)!= 0x0F) return 2;

    // 配置 GYRO：±2000dps, ODR 1000Hz, BW 116Hz, Normal Mode
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_RANGE, 0x00);
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_BANDWIDTH, 0x00);
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_LPM1, 0x00);
    
    return 0; // 全部成功
}

static uint8_t BMI088_Get_Acc_ID(void)
{
    return IIC_Read_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CHIP_ID);
}

static uint8_t BMI088_Get_Gyro_ID(void)
{
    return IIC_Read_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_CHIP_ID);
}

/**
 * @brief 读取加速度计数据并存入结构体
 * @param imu 接收数据的结构体指针
 */
static void BMI088_Read_Acc(IMU_t *imu)
{
    uint8_t acc_raw[7]; 
    if(IIC_Read_Multi_Byte(&bmi088_bus, BMI088_ACC_ADDR, 0x12, 7, acc_raw) == 0)
    {
        int16_t raw[3];
        // 拼接原始数据
        raw[0] = (int16_t)((acc_raw[2] << 8) | acc_raw[1]); // X
        raw[1] = (int16_t)((acc_raw[4] << 8) | acc_raw[3]); // Y
        raw[2] = (int16_t)((acc_raw[6] << 8) | acc_raw[5]); // Z

        // 根据背面安装要求取反 Y, Z
        imu->acc_raw[0] = raw[0];
        imu->acc_raw[1] = -raw[1]; 
        imu->acc_raw[2] = -raw[2]; 
    }
}
/**
 * @brief 读取陀螺仪数据并存入结构体
 */
static void BMI088_Read_Gyro(IMU_t *imu)
{
    int16_t gyro_raw[6];
    if(IIC_Read_Multi_Byte(&bmi088_bus, BMI088_GYRO_ADDR, 0x02, 6, gyro_raw) == 0) 
    {
        // 解析原始数据
        imu->gyro_raw[0] = (int16_t)((gyro_raw[1] << 8) | gyro_raw[0]); // X轴
        imu->gyro_raw[1] = (int16_t)((gyro_raw[3] << 8) | gyro_raw[2]); // Y轴
        imu->gyro_raw[2] = (int16_t)((gyro_raw[5] << 8) | gyro_raw[4]); // Z轴
        // 根据背面安装要求取反 Y, Z
        imu->gyro_raw[1] = -imu->gyro_raw[1];
        imu->gyro_raw[2] = -imu->gyro_raw[2];
    }
}

/**
 * @brief 读取 BMI088 传感器数据（加速度计、陀螺仪、温度）
 * @param imu 指向 IMU 结构体的指针
 */
void BMI088_GetData(IMU_t *imu)
{
    BMI088_Read_Acc(imu);
    BMI088_Read_Gyro(imu);
}

