#include "pid.h"
#include <math.h>

/**
 * @brief PID 参数初始化
 */
void PID_Init(PID_t *pid)
{
    // pid->max_iout = pid->max_out * 1.0f; // 默认积分限幅为总输出的 20%
    if(pid->deadband == 0.0f) pid->deadband = 0.5f; // 默认死区为 0.5
    PidReset(pid); 
}

/**
 * @brief 重置 PID 历史状态
 */
void PidReset(PID_t *pid)
{
    pid->now = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->pout = pid->iout = pid->dout = pid->out = 0.0f;
}

/**
 * @brief PID 输出限幅实现
 */
static void PidOutLimit(PID_t *pid)
{
    if (pid->out > pid->max_out)  pid->out = pid->max_out;
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;
}

/**
 * @brief PID 核心计算
 */
void PidCalculate(PID_t *pid)
{
    // 1. 计算当前误差
    pid->error[0] = pid->tar - pid->now;

    // 2. 死区处理：如果误差非常小，直接视为0，防止电机在高频率噪声下抖动
    if (fabs(pid->error[0]) < pid->deadband) 
    {
        pid->error[0] = 0.0f;
    }

    // 3. 进入不同模式的算法
    if (pid->mode == DELTA_PID)  // 增量式 PID
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        
        pid->out += pid->pout + pid->iout + pid->dout;
    }
    else if (pid->mode == POSITION_PID)  // 位置式 PID
    {
        // 比例项
        pid->pout = pid->p * pid->error[0];
        
        // 积分项（带抗饱和限幅）
        pid->iout += pid->i * pid->error[0];
        if (pid->iout > pid->max_iout)  pid->iout = pid->max_iout;
        if (pid->iout < -pid->max_iout) pid->iout = -pid->max_iout;
        
        // 微分项
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        
        pid->out = pid->pout + pid->iout + pid->dout;
    }
    // 4. 总输出限幅
    PidOutLimit(pid); 
    // 5. 更新误差历史
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}
