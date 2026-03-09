#ifndef _FLOW_H
#define _FLOW_H

#include "stdint.h"
#include "bmi088.h"
#include "usart.h"

// 弧度与角度转换常量
#define ANG_2_RAD 0.01745329f

/**
 * @brief 光流融合输出数据结构体
 * 存储经过高度补偿、姿态补偿及滤波后的最终位移和速度
 */
typedef struct 
{
    // --- 位移相关 (单位: cm) ---
    float fix_x_i;      // x轴积分滤波值（不含姿态补偿）
    float fix_y_i;      // y轴积分滤波值
    float out_x_i;      // x轴姿态补偿后的最终位移值
    float out_y_i;      // y轴姿态补偿后的最终位移值
    float out_x_i_o;    // 上一次的位移值（用于计算微分速度）
    float out_y_i_o;    // 上一次的位移值
    
    float loc_x;        // 当前坐标测量值（提供给位置PID）
    float loc_y;        // 当前坐标测量值
    
    // --- 速度相关 (单位: cm/s) ---
    float x;            // x轴微分速度原始值
    float y;            // y轴微分速度原始值
    float fix_x;        // x轴滤波后的融合速度值
    float fix_y;        // y轴滤波后的融合速度值
    float loc_xs;       // 最终输出的x轴速度
    float loc_ys;       // 最终输出的y轴速度

    // --- 补偿相关 ---
    float ang_x;        // x轴姿态角度补偿值
    float ang_y;        // y轴姿态角度补偿值
    
    // --- 状态标志与调试 ---
    uint8_t flow_pause;      // 光流暂停标志位
    uint8_t err1_cnt;        // 异常计数器1（数据失效）
    uint8_t err2_cnt;        // 异常计数器2（逻辑异常）
} PIXEL_FLOW_Struct; // 定点结构体

/**
 * @brief 光流原始数据结构体
 * 直接对接串口解析出来的原始像素数据
 */
typedef struct 
{
    float flow_x;       // 单帧x轴像素移动量
    float flow_y;       // 单帧y轴像素移动量
    float flow_x_i;     // 累加的像素位移（高度融合后）
    float flow_y_i;
    
    float flow_x_iOUT;  // 仅供调试用的总像素位移（不复位）
    float flow_y_iOUT;
    
    float flow_High;    // 光流模块内置高度（单位: cm）
    uint8_t qual;            // 光流信号质量 (0-255)
    uint8_t ok;              // 光流状态是否有效标志
} FLOW_Struct;

// 外部全局变量声明
extern PIXEL_FLOW_Struct pixel_flow;
extern FLOW_Struct mini;            
extern float pixel_cpi;     

void Flow_Init(void);
void Flow_Parse_Data(uint8_t data);          
void Pixel_Flow_Fix(EulerAngle_Struct *euler_angle,float dT) ;        

/**
 * @brief 移动坐标轴归零函数
 * 在不产生物理抖动的情况下重置期望位置与当前测量坐标
 */
// void Pixel_Flow_set_zero(float desired_x, float desired_y);

#endif