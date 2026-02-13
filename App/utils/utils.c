#include "headfile.h"

/**
 * @brief 系统初始化函数，初始化所有模块和外设
 * @param 无
 * @retval 无
 */
void System_Init(void)
{
    LedDevice_Init();
    if (BMI088_Init() == 0)
    {
        // 零偏校准
        BMI088_Calibrate();
    }
    Power_Init();
    NRF24L01_Init();  
    Motor_Init();
    PIDParam_Init();
    // SPL06_Init();
    // Flow_Init();  
    SetLedALL(LED_ON);
    delay_ms(2000);
    SetLedALL(LED_OFF);
}

void LedScan(void)
{
    if(flight_rc_data.CONNECT)
    {
        SetLedMode(bLEDL, LED_TOGGLE);     // 连接成功，蓝红双闪
        SetLedMode(bLEDR, LED_TOGGLE);
        SetLedMode(rLEDL, LED_TOGGLE);
        SetLedMode(rLEDR, LED_TOGGLE);
    }
    else 
    { 
        SetLedMode(bLEDL, LED_OFF);        // 连接失败，红灯闪
        SetLedMode(bLEDR, LED_OFF);
        SetLedMode(rLEDL, LED_TOGGLE);
        SetLedMode(rLEDR, LED_TOGGLE);
    }
}

/**
 * @brief 接收端等待连接
 * @return 0: 收到数据包(连接建立), 1: 等待中
 */
uint8_t NRF_RX_Wait_Connect(void)
{
    uint8_t temp_buf[32];
    
    // 轮询是否收到数据
    if(NRF24L01_RxPacket(temp_buf) == 0)
    {
        // 可以在这里判断 temp_buf 的内容是否为握手协议内容
        return 0; 
    }
    return 1;
}