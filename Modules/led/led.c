#include "led.h"
#include "bsp_gpio.h"
#include "bsp_dwt.h"

static LEDInstance led[LED_NUM];

/**
 * @brief 获取任意一个LED的数据
 */
static const LEDInstance *GetLedInstance(LED_Type_e LedType)
{
	return &led[LedType];
}

/**
 * @brief 根据当前状态更新GPIO电平
 */
static void UpdatePinLevel(LED_Type_e LedType)
{
    LEDInstance *instance = &led[LedType];
    // 逻辑：状态与有效电平相同则置位，不同则复位
    GPIO_PinState pinState = (instance->RunningParam.CurrentMode == instance->StaticParam.ActiveLevel) ? 
                              GPIO_PIN_SET : GPIO_PIN_RESET;
    
    HAL_GPIO_WritePin(instance->StaticParam.GPIO_Port, instance->StaticParam.GPIO_Pin, pinState);
}

/**
 * @brief 设置LED状态
 */
void SetLedMode(LED_Type_e LedType, LED_Mode_e Mode)
{
    if (LedType >= LED_NUM) return;

    // 如果是翻转模式，直接操作硬件并更新内部状态记录
    if (Mode == LED_TOGGLE)
    {
        HAL_GPIO_TogglePin(led[LedType].StaticParam.GPIO_Port, led[LedType].StaticParam.GPIO_Pin);
        
        // 同步更新 RunningParam 里的状态，保持逻辑一致性
        led[LedType].RunningParam.CurrentMode = 
            (led[LedType].RunningParam.CurrentMode == LED_ON) ? LED_OFF : LED_ON;
    }
    else
    {
        // 普通的 ON/OFF 逻辑
        led[LedType].RunningParam.CurrentMode = Mode;
        UpdatePinLevel(LedType);
    }
}

static void Led_Init(LedStaticParam_s *config, LED_Type_e LedType)
{
    if (LedType >= LED_NUM) return; 
    // 复制配置
    led[LedType].StaticParam = *config;
	GPIO_Output(config->GPIO_Port, config->GPIO_Pin, BSP_GPIO_SPEED_LOW);
    // 初始化时关闭LED
    SetLedMode(LedType, LED_OFF);
}

void LedDevice_Init(void)
{ 
	LedStaticParam_s config;

	config.GPIO_Port = LED_Port;
	config.GPIO_Pin = rLEDL_Pin;
	config.ActiveLevel = LED_LOW_LEVEL_ON;
	Led_Init(&config, rLEDL);

	config.GPIO_Port = LED_Port;
	config.GPIO_Pin = rLEDR_Pin;
	config.ActiveLevel = LED_LOW_LEVEL_ON;
	Led_Init(&config, rLEDR);

	config.GPIO_Port = LED_Port;
	config.GPIO_Pin = bLEDL_Pin;
	config.ActiveLevel = LED_LOW_LEVEL_ON;
	Led_Init(&config, bLEDL);

	config.GPIO_Port = LED_Port;
	config.GPIO_Pin = bLEDR_Pin;
	config.ActiveLevel = LED_LOW_LEVEL_ON;
	Led_Init(&config, bLEDR);
}

void SetLedALL(LED_Mode_e Mode)
{
    SetLedMode(rLEDL, Mode);
    SetLedMode(rLEDR, Mode);
    SetLedMode(bLEDL, Mode);
    SetLedMode(bLEDR, Mode);
}

/**
 * @brief 所有LED同步闪烁一次
 * @note 需在定时处理函数或主循环中按周期调用
 */
void Led_SyncBlink_Process(void)
{
    static LED_Mode_e state = LED_ON;
    state = (state == LED_ON) ? LED_OFF : LED_ON;
    
    for (int i = 0; i < LED_NUM; i++) {
        SetLedMode((LED_Type_e)i, state);
			delay_ms(100);
    }
}

/**
 * @brief LED交替闪烁
 */
void Led_AltBlink_Process(void) 
{
    static LED_Mode_e baseState = LED_ON;
    baseState = (baseState == LED_ON) ? LED_OFF : LED_ON;
    SetLedMode(rLEDL, baseState);
    SetLedMode(rLEDR, baseState);
	delay_ms(100);
    LED_Mode_e altState = (baseState == LED_ON) ? LED_OFF : LED_ON;
    SetLedMode(bLEDL, altState);
    SetLedMode(bLEDR, altState);
}