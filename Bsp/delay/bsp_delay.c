#include "bsp_delay.h"
#include "stm32f1xx_hal.h"

void delay_us(uint32_t us)
{
    uint32_t delay = us * 6; 
    while (delay--)
    {
        __NOP(); // 产生一个周期空操作
    }
}

void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}