#include "bmi088.h"
#include "bsp_iic.h"
#include "bsp_delay.h"
#include "led.h"
#include "memory.h"
#include <math.h>

// 静态定义总线
static iic_bus_t bmi088_bus = {
    .IIC_SDA_PORT = SDA_GPIO_Port,
    .IIC_SDA_PIN = SDA_Pin,
    .IIC_SCL_PORT = SCL_GPIO_Port,
    .IIC_SCL_PIN = SCL_Pin
};

// 校准偏移量
static Gyro_Struct gyro_offset = {0};
static Acc_Struct acc_offset = {0};

// 用于姿态解算的静态变量
static EulerAngle_Struct attitude = {0, 0, 0};

// ==================== 基础读写函数 ====================
/**
  * @brief 读取原始加速度计数据（不应用校准）
  * @param acc 加速度计数据结构体指针
  * @retval 0:成功, 1:失败
  */
uint8_t BMI088_Read_Acc_Raw(Acc_Struct *acc)
{
    uint8_t acc_data[6] = {0};
    uint8_t ret;
    
    ret = IIC_Read_Multi_Byte(&bmi088_bus, BMI088_ACC_ADDR, 0x12, 6, acc_data);
    if (ret != 0) {
        return 1;
    }
    
    // 只组合数据，不应用校准
    acc->x = ((int16_t)acc_data[1] << 8) | acc_data[0];
    acc->y = ((int16_t)acc_data[3] << 8) | acc_data[2];
    acc->z = ((int16_t)acc_data[5] << 8) | acc_data[4];
    
    return 0;
}

/**
  * @brief 读取原始陀螺仪数据（不应用校准）
  * @param gyro 陀螺仪数据结构体指针
  * @retval 0:成功, 1:失败
  */
uint8_t BMI088_Read_Gyro_Raw(Gyro_Struct *gyro)
{
    uint8_t gyro_data[6] = {0};
    uint8_t ret;
    
    ret = IIC_Read_Multi_Byte(&bmi088_bus, BMI088_GYRO_ADDR, 0x02, 6, gyro_data);
    if (ret != 0) {
        return 1;
    }
    
    // 只组合数据，不应用校准
    gyro->x = ((int16_t)gyro_data[1] << 8) | gyro_data[0];
    gyro->y = ((int16_t)gyro_data[3] << 8) | gyro_data[2];
    gyro->z = ((int16_t)gyro_data[5] << 8) | gyro_data[4];
    
    return 0;
}

/**
  * @brief 读取BMI088加速度计数据（支持背面安装）
  * @param acc 加速度计数据结构体指针
  * @retval 0:成功, 1:失败
  */
uint8_t BMI088_Read_Acc(Acc_Struct *acc)
{
    uint8_t acc_data[6] = {0};
    uint8_t ret;
    
    ret = IIC_Read_Multi_Byte(&bmi088_bus, BMI088_ACC_ADDR, 0x12, 6, acc_data);
    if (ret != 0) {
        return 1;
    }
    
    // 组合数据 (MSB先传)
    int16_t raw_x = ((int16_t)acc_data[1] << 8) | acc_data[0];
    int16_t raw_y = ((int16_t)acc_data[3] << 8) | acc_data[2];
    int16_t raw_z = ((int16_t)acc_data[5] << 8) | acc_data[4];
    
    // 应用校准偏移
    raw_x -= acc_offset.x;
    raw_y -= acc_offset.y;
    raw_z -= acc_offset.z;
    
    // 根据安装方向进行坐标系转换
#if (INSTALL_ORIENTATION == 1)  // 背面安装
    // 绕x轴进行背面安装
    acc->x = raw_x;
    acc->y = -raw_y;
    acc->z = -raw_z;
#else  // 正面安装
    // 正面安装：保持原样
    acc->x = raw_x;
    acc->y = raw_y;
    acc->z = raw_z;
#endif
    
    return 0;
}

/**
  * @brief 读取BMI088陀螺仪数据（支持背面安装）
  * @param gyro 陀螺仪数据结构体指针
  * @retval 0:成功, 1:失败
  */
uint8_t BMI088_Read_Gyro(Gyro_Struct *gyro)
{
    uint8_t gyro_data[6] = {0};
    uint8_t ret;
    
    ret = IIC_Read_Multi_Byte(&bmi088_bus, BMI088_GYRO_ADDR, 0x02, 6, gyro_data);
    if (ret != 0) {
        return 1;
    }
    
    // 组合数据 (MSB先传)
    int16_t raw_x = ((int16_t)gyro_data[1] << 8) | gyro_data[0];
    int16_t raw_y = ((int16_t)gyro_data[3] << 8) | gyro_data[2];
    int16_t raw_z = ((int16_t)gyro_data[5] << 8) | gyro_data[4];
    
    // 应用校准偏移
    raw_x -= gyro_offset.x;
    raw_y -= gyro_offset.y;
    raw_z -= gyro_offset.z;
    
    // 根据安装方向进行坐标系转换
#if (INSTALL_ORIENTATION == 1)  // 背面安装
    // 绕x轴进行背面安装
    gyro->x = raw_x;
    gyro->y = -raw_y;
    gyro->z = -raw_z;
#else  // 正面安装
    // 正面安装：保持原样
    gyro->x = raw_x;
    gyro->y = raw_y;
    gyro->z = raw_z;
#endif
    
    return 0;
}

/**
  * @brief 读取BMI088所有数据（支持安装方向）
  * @param gyro_acc 包含加速度和陀螺仪数据的结构体指针
  * @retval 0:成功, 1:失败
  */
uint8_t BMI088_Read(Gyro_Acc_Struct *gyro_acc)
{
    uint8_t ret1 = BMI088_Read_Acc(&gyro_acc->acc);
    uint8_t ret2 = BMI088_Read_Gyro(&gyro_acc->gyro);
    
    return (ret1 || ret2) ? 1 : 0;
}

// ==================== 初始化与校准 ====================

/**
 * @brief  BMI088 初始化
 * @return 0:成功, 1:ACC错误, 2:GYRO错误
 */
uint8_t BMI088_Init(void)
{
    uint8_t chip_id = 0;
    uint8_t retry = 0;

    IICInit(&bmi088_bus);
    // 初始化加速度计
    for (retry = 0; retry < 5; retry++)
    {
        chip_id = IIC_Read_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CHIP_ID);
        if (chip_id == 0x1E) break;
        delay_ms(10);
    }
    if (chip_id != 0x1E) return 1;
    // 软复位加速度计
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_SOFTRESET, 0xB6);
    delay_ms(10);
    // 配置加速度计电源模式：主动模式
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00);
    delay_ms(5);
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_PWR_CTRL, 0x04);
    delay_ms(10);
    // 配置加速度计参数：±6g, ODR 400Hz, OSR4
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CONF, 0x1A); // 0x1A = OSR4 + 400Hz
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_RANGE, 0x01); // ±6g
    delay_ms(10);

    // 初始化陀螺仪
    for (retry = 0; retry < 5; retry++)
    {
        chip_id = IIC_Read_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_CHIP_ID);
        if (chip_id == 0x0F) break;
        delay_ms(10);
    }
    if (chip_id != 0x0F) return 2;
    // 软复位陀螺仪
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_SOFTRESET, 0xB6);
    delay_ms(30);
    // 配置陀螺仪参数：±2000dps, ODR 1000Hz, BW 116Hz, Normal Mode
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_RANGE, 0x02);      // ±2000dps
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_BANDWIDTH, 0x02);  // 1000Hz, BW 116Hz
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_LPM1, 0x00);       // Normal Mode
    delay_ms(10);

    return 0;
}
/**
  * @brief 校准BMI088传感器（绕X轴背面安装版本）
  * @param None
  * @retval None
  */
void BMI088_Calibrate(void)
{
    Gyro_Struct gyro_temp;
    Acc_Struct acc_temp;
    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    int32_t acc_sum_x = 0, acc_sum_y = 0, acc_sum_z = 0;
    // 采样次数
    uint16_t sample_count = 500; 
    uint16_t i;
    
    // 等待传感器稳定
    delay_ms(20);
    
    // 校准陀螺仪（零偏）
    for (i = 0; i < sample_count; i++)
    {
        BMI088_Read_Gyro_Raw(&gyro_temp);
        gyro_sum_x += gyro_temp.x;
        gyro_sum_y += gyro_temp.y;
        gyro_sum_z += gyro_temp.z;
        delay_ms(2);
    }
    
    gyro_offset.x = (int16_t)(gyro_sum_x / sample_count);
    gyro_offset.y = (int16_t)(gyro_sum_y / sample_count);
    gyro_offset.z = (int16_t)(gyro_sum_z / sample_count);
    
    // 校准加速度计（绕X轴背面安装）
    for (i = 0; i < sample_count; i++)
    {
        BMI088_Read_Acc_Raw(&acc_temp);
        acc_sum_x += acc_temp.x;
        acc_sum_y += acc_temp.y;
        acc_sum_z += acc_temp.z;
        delay_ms(2);
    }
    
    // 绕X轴旋转180度安装分析：
    // 原始传感器：水平放置时，重力在+Z方向（+8192 LSB）
    // 绕X轴旋转180度后：Y变-Y，Z变-Z
    // 所以水平放置时，重力应该在-Z方向（-8192 LSB）
    
    // 但是！在应用坐标系转换后，我们会执行：acc->z = -raw_z
    // 所以校准后的raw_z应该是：测量值 - 偏移量
    // 我们期望转换后的z为+1g，即+8192 LSB
    // 所以：-raw_z = +8192  => raw_z = -8192
    // 因此偏移量应该是：测量值 - (-8192) = 测量值 + 8192
    
    acc_offset.x = (int16_t)(acc_sum_x / sample_count);
    acc_offset.y = (int16_t)(acc_sum_y / sample_count);
    acc_offset.z = (int16_t)(acc_sum_z / sample_count + 8192); 
    
    // 重置姿态
    attitude.roll = 0;
    attitude.pitch = 0;
    attitude.yaw = 0;
}

// ==================== 数据转换函数 ====================

/**
  * @brief 获取传感器原始数据到物理值的转换系数
  * @param acc_sens 加速度灵敏度 (LSB/g)
  * @param gyro_sens 陀螺仪灵敏度 (LSB/(°/s))
  * @retval None
  */
void BMI088_Get_Sensitivity(float *acc_sens, float *gyro_sens)
{
    // ±6g量程下，灵敏度为 8192 LSB/g
    *acc_sens = 8192.0f;
    
    // ±2000dps量程下，灵敏度为 16.384 LSB/(°/s)
    *gyro_sens = 16.384f;
}

/**
  * @brief 将原始数据转换为物理值
  * @param gyro_acc 原始数据结构体
  * @param gyro_dps 输出：陀螺仪角速度 (°/s)，使用Acc_Struct类型存储浮点数
  * @param acc_g 输出：加速度 (g)，使用Acc_Struct类型存储浮点数
  * @retval None
  */
void BMI088_Convert_To_Physical(Gyro_Acc_Struct *gyro_acc, 
                                Acc_Struct *gyro_dps_f,  // 用于存储浮点角速度
                                Acc_Struct *acc_g_f)     // 用于存储浮点加速度
{
    float acc_sens, gyro_sens;
    BMI088_Get_Sensitivity(&acc_sens, &gyro_sens);
    
    if (gyro_dps_f)
    {
        gyro_dps_f->x = (float)gyro_acc->gyro.x / gyro_sens;
        gyro_dps_f->y = (float)gyro_acc->gyro.y / gyro_sens;
        gyro_dps_f->z = (float)gyro_acc->gyro.z / gyro_sens;
    }
    
    if (acc_g_f)
    {
        acc_g_f->x = (float)gyro_acc->acc.x / acc_sens;
        acc_g_f->y = (float)gyro_acc->acc.y / acc_sens;
        acc_g_f->z = (float)gyro_acc->acc.z / acc_sens;
    }
}

/**
  * @brief 验证安装方向是否正确
  * @retval 1:安装方向正确，0:安装方向错误
  */
uint8_t BMI088_Verify_Installation(void)
{
    Acc_Struct acc;
    float acc_sens = 8192.0f;
    float z_g;
    
    // 读取几次数据取平均
    int32_t sum_z = 0;
    uint8_t samples = 20;
    
    for (int i = 0; i < samples; i++)
    {
        BMI088_Read_Acc(&acc);
        sum_z += acc.z;
        delay_ms(10);
    }
    
    z_g = (float)(sum_z / samples) / acc_sens;
    
#if (INSTALL_ORIENTATION == 1)  // 背面安装
    // 背面安装时，水平放置飞控，Z轴应该接近+1g
    if (z_g > 0.7f && z_g < 1.3f) {
        return 1; // 安装方向正确
    } else {
        return 0; // 安装方向错误
    }
#else  // 正面安装
    // 正面安装时，水平放置飞控，Z轴应该接近+1g
    if (z_g > 0.7f && z_g < 1.3f) {
        return 1; // 安装方向正确
    } else {
        return 0; // 安装方向错误
    }
#endif
}