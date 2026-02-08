#include "motor.h"
#include "tim.h"
#include "bsp_delay.h"

void Motor_Init(void) 
{
    // 启动定时器 PWM 输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // M1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // M2
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // M3
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // M4
}

static void Update_Motor_PWM(uint8_t id, uint16_t pulse) 
{
    if(pulse < 1000) pulse = 1000;
    if(pulse > 2000) pulse = 2000;

    switch(id) 
    {
        case 0: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); break; // M1   右下
        case 1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse); break; // M2   右上   
        case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse); break; // M3   左上
        case 3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse); break; // M4   左下
    }
}

void MotorSetPWM(float m1, float m2, float m3, float m4)
{
    Update_Motor_PWM(0, m1 + 1000);
    Update_Motor_PWM(1, m2 + 1000);
    Update_Motor_PWM(2, m3 + 1000);
    Update_Motor_PWM(3, m4 + 1000);
}

void MotorTest(void)
{
    float test_pwm = 1100.0f; // 10% 油门
    // 依次测试每一个电机 (0 到 3)
    for(uint8_t i = 0; i < 4; i++)
    {
        Update_Motor_PWM(i, test_pwm);
        delay_ms(800); 
        // 停止当前电机
        Update_Motor_PWM(i, 1000);
        delay_ms(200);
    }
    for(uint8_t i = 0; i < 4; i++) Update_Motor_PWM(i, 1000);
}