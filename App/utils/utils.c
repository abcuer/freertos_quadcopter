#include "headfile.h"
#include "bsp_delay.h"
#include "led.h"

/**
 * @brief 系统初始化函数，初始化所有模块和外设
 * @param 无
 * @retval 无
 */
void System_Init(void)
{
    LedDevice_Init();
    BMI088_Init();

    // NRF24L01_init();

    // SPL06_Init();
    // Flow_Init();    
    Motor_Init();
    delay_ms(3000);
}
