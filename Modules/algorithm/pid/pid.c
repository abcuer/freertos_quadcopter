#include "pid.h"

/**
 * @brief 重置 PID 历史状态
 */
/**
 * @brief 重置 PID 结构体中的历史状态量
 * @param pid 指向需要重置的 PID 结构体的指针
 */
void PID_Reset(PID_Struct *pid)
{
    // 清除测量值，防止逻辑残留
    pid->measure = 0.0f;
    // 清除历史状态，防止启动瞬间由于旧数据导致大幅跳变
    pid->integral = 0.0f;    // 清除积分累计
    pid->last_error = 0.0f;  // 清除上一次误差（影响微分项）
    // 清除当前输出值
    pid->output = 0.0f;     
}

void PID_Calculate(PID_Struct *pid, float dt)
{
    // 计算误差
    float error = pid->desire - pid->measure;
    // 计算积分项
    pid->integral += error * dt;
    // pid->integral = Limit(pid->integral, -50, 50);
    // 计算微分项
    float derivative = (error - pid->last_error) / dt;
    // 计算输出
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    // 保存上一次误差
    pid->last_error = error;
}

void PID_Cascade(PID_Struct *outter, PID_Struct *inner, float dt)
{
    // 计算外部PID
    PID_Calculate(outter, dt);
    // 计算内部PID
    inner->desire = outter->output;
    PID_Calculate(inner, dt);
}


