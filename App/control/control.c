#include "control.h"
#include "bmi088.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
#include "remote.h"

// 定义陀螺仪和加速度
Gyro_Acc_Struct gyro_acc;
// 定义欧拉角
EulerAngle_Struct euler_angle;

// PID_Struct pid_pitch = {.kp = 6.8f, .ki = 0.0f, .kd = 0.0f};
// PID_Struct pid_gyro_y = {.kp = 1.2f, .ki = 0.0f, .kd = 0.03f};
PID_Struct pid_pitch = {.kp = 6.9f, .ki = 0.0f, .kd = 0.0f};
PID_Struct pid_gyro_y = {.kp = 1.2f, .ki = 0.0f, .kd = 0.04f};

PID_Struct pid_roll = {.kp = 6.9f, .ki = 0.0f, .kd = 0.0f};
PID_Struct pid_gyro_x = {.kp = 1.2f, .ki = 0.0f, .kd = 0.04f};

PID_Struct pid_yaw = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f};
PID_Struct pid_gyro_z = {.kp = 1.2f, .ki = 0.0f, .kd = 0.04f};

void Flight_Calculate_PID(Gyro_Acc_Struct *gyro_acc, EulerAngle_Struct *euler_angle, Remote_Data_Struct *rc_data, float dt)
{
    // 计算俯仰角 (Pitch) PID
    pid_pitch.desire = (1500 - rc_data->PIT) * 0.03f;            // 期望值（来自遥控器）
    pid_pitch.measure = euler_angle->pitch;                     // 测量值（欧拉角）
    pid_gyro_y.measure = gyro_acc->gyro.y * Gyro_G;             // 测量值（角速度）
    PID_Cascade(&pid_pitch, &pid_gyro_y, dt);     // 串级 PID 计算

    // 计算横滚角 (Roll) PID
    pid_roll.desire = (1500 - rc_data->ROL) * 0.03f;             // 期望值
    pid_roll.measure = euler_angle->roll;                       // 测量值
    pid_gyro_x.measure = gyro_acc->gyro.x * Gyro_G;             // 测量值
    PID_Cascade(&pid_roll, &pid_gyro_x, dt);      // 串级 PID 计算

    // 计算偏航角 (Yaw) PID
    pid_yaw.desire = (1500 - rc_data->YAW) * 0.03f;              // 期望值
    pid_yaw.measure = euler_angle->yaw;                         // 测量值
    pid_gyro_z.measure = gyro_acc->gyro.z * Gyro_G;             // 测量值
    PID_Cascade(&pid_yaw, &pid_gyro_z, dt);       // 串级 PID 计算
}

PID_Struct pid_height = {.kp = 3.0f, .ki = 0.0f, .kd = 0.1f};

void HeightPidCtrl(FLOW_Struct *flow, Remote_Data_Struct *rc_data, float dt)
{
    pid_height.desire = ABS(rc_data->THR - 1000) * 0.4f;
    // vTaskSuspendAll(); // 暂停调度
    pid_height.measure = flow->flow_High;
    // xTaskResumeAll();  // 恢复调度
    PID_Calculate(&pid_height, dt);
}

/*
    roll        gx
    pitch       gy
    yaw         gz
*/
float motor_pwm[4];
void FlyControl(void)
{
    if(flight_rc_data.CONNECT && flight_rc_data.THR >= 1050)
    {
        // 左上
        motor_pwm[2] = Limit(flight_rc_data.THR, 1000.0f, 1800.0f) + (- pid_gyro_x.output + pid_gyro_y.output - pid_gyro_z.output) + pid_height.output;
        // 右上
        motor_pwm[1] = Limit(flight_rc_data.THR, 1000.0f, 1800.0f) + (pid_gyro_x.output + pid_gyro_y.output + pid_gyro_z.output) + pid_height.output;
        // 左下
        motor_pwm[3] = Limit(flight_rc_data.THR, 1000.0f, 1800.0f) + (- pid_gyro_x.output - pid_gyro_y.output + pid_gyro_z.output) + pid_height.output; 
        // 右下
        motor_pwm[0] = Limit(flight_rc_data.THR, 1000.0f, 1800.0f) + (pid_gyro_x.output - pid_gyro_y.output - pid_gyro_z.output) + pid_height.output;
 
        // 对最终的 pwm 进行约束
        motor_pwm[2] = Limit(motor_pwm[2], 1000, 2500);
        motor_pwm[1] = Limit(motor_pwm[1], 1000, 2500);
        motor_pwm[3] = Limit(motor_pwm[3], 1000, 2500);
        motor_pwm[0] = Limit(motor_pwm[0], 1000, 2500);
    }
    else 
    {
        motor_pwm[0] = motor_pwm[1] = motor_pwm[2] = motor_pwm[3] = 0;
        PID_Reset(&pid_pitch);
        PID_Reset(&pid_gyro_y);
        PID_Reset(&pid_roll);
        PID_Reset(&pid_gyro_x);
        PID_Reset(&pid_yaw);
        PID_Reset(&pid_gyro_z);
        PID_Reset(&pid_height);
    }

    SetMotorPWM(motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
}

