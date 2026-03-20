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

int16_t global_voltage = 4000; // 全局变量，初始值设为4000mV左右

int16_t Voltage_Check(void)
{
    // 1. 计算当前瞬时电压值 (单位: mV)
    // 公式: (ADC_DATA / ADC_VREFINT) * 1.2V * 分压倍数 * 1000
    // 假设 1.2f 是 STM32 内部参考电压的标准值
    float instant_volt = (2.0f * ADC_ConvertedValue[0] / (float)ADC_ConvertedValue[1] * 1.2f * 1000.0f);
    
    // 2. 补偿二极管压降 (参考之前的代码 +44)
    instant_volt += 44.0f;

    // 3. 一阶低通滤波
    // 使用全局变量进行迭代，系数 0.2f 决定了滤波强度（越小越平滑但响应越慢）
    global_voltage += 0.2f * (instant_volt - global_voltage);

    return global_voltage;
}