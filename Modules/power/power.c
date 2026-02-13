#include "power.h"
#include "adc.h"
#include "remote.h"

uint16_t ADC_ConvertedValue[2];
void Power_Init(void)
{
    // ADC校准
    HAL_ADCEx_Calibration_Start(&hadc1); 
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_ConvertedValue, 2);
}

int16_t voltage = 4000;
void Voltage_Check(void)
{
	static uint16_t cnt0, cnt1;

	voltage += 0.2f *(2.0f * ADC_ConvertedValue[0]/ADC_ConvertedValue[1] * 1.2f*1000 - voltage);
    //不飞行时的低压判断
    if(flight_rc_data.LOCK_KEY == 0) return;
    else
    {
        // 低压
        if(voltage < POWER0 && voltage > 3400)
        {
            if(cnt0 < 100) cnt0++;
			cnt1 = 0;
        }
        // 正常
        else if(voltage > POWER1) 
		{
			if(cnt1 < 100) cnt1++;
			cnt0 = 0;
		}
        else 
        {
            cnt0 = 0;
            cnt1 = 0;
        }
        // 实际动作触发逻辑
        if(cnt0 >= 100) {
            // 执行低压报警动作，例如 LED 闪烁
        }
    }
}