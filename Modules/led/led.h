#ifndef _LED_H
#define _LED_H

#include "stm32f1xx_hal.h"

#define LED_NUM 4

#define LED_Port GPIOB
#define rLEDL_Pin GPIO_PIN_8        // 下方左红灯
#define rLEDR_Pin GPIO_PIN_9        // 下方右红灯
#define bLEDL_Pin GPIO_PIN_3        // 上方左蓝灯
#define bLEDR_Pin GPIO_PIN_1        // 上方右蓝灯

typedef enum
{
    rLEDL = 0,
    rLEDR,
    bLEDL,
    bLEDR
} LED_Type_e;

typedef enum
{
    LED_OFF = 0,
    LED_ON,
    LED_TOGGLE
} LED_Mode_e;

typedef enum
{
    LED_LOW_LEVEL_ON = 0,
    LED_HIGH_LEVEL_ON,
} LED_ActiveLevel_e;

typedef struct
{
    GPIO_TypeDef* GPIO_Port;
    uint16_t GPIO_Pin;
    LED_ActiveLevel_e ActiveLevel; // 工作时的电平
} LedStaticParam_s;

typedef struct
{
    LED_Mode_e CurrentMode;
} LedRunningParam_s;

typedef struct
{
    LedStaticParam_s StaticParam;
    LedRunningParam_s RunningParam;
} LEDInstance;

void LedDevice_Init(void);
void SetLedMode(LED_Type_e LEDType, LED_Mode_e Mode);
void Led_SyncBlink_Process(void);
void Led_AltBlink_Process(void);
void SetLedALL(LED_Mode_e Mode);

#endif