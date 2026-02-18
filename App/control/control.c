#include "control.h"
#include "bmi088.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
#include "remote.h"

/*
    你的内环目前把飞机调成了一个**“无静差阻尼系统”**。
    你把它拨到哪个角度，它就试图停在哪个角度（虽然因为零漂停不住）。
    这正是角速度环调好的标志——它只负责“稳”，不负责“正”。
*/
 
// 定义陀螺仪和加速度
Gyro_Acc_Struct gyro_acc;
// 定义欧拉角
EulerAngle_Struct euler_angle;
// 定义PID参数
// 调到能比较合适的跟随摇杆运动，到平衡点后不会往返震荡，但是无法很快回归零点的话就可以确定下来
/* 暂时只调整i */
// PID_Struct pid_pitch = {.kp = 7.0f, .ki = 0.0f, .kd = 0.0f};
// PID_Struct pid_gyro_y = {.kp = 1.9f, .ki = 0.002f, .kd = 0.075f};
// PID_Struct pid_pitch = {.kp = 7.0f, .ki = 0.0f, .kd = 0.0f};
// PID_Struct pid_gyro_y = {.kp = 1.9f, .ki = 0.001f, .kd = 0.07f};
// PID_Struct pid_pitch = {.kp = 7.0f, .ki = 0.0f, .kd = 0.0f};
// PID_Struct pid_gyro_y = {.kp = 1.9f, .ki = 0.00f, .kd = 0.07f};

// // 只有一边能比较顺滑的回正，但是到达平衡点会小幅震荡，回正速度没有很快
// PID_Struct pid_pitch = {.kp = 6.8f, .ki = 0.0f, .kd = 0.0f};
// PID_Struct pid_gyro_y = {.kp = 1.52f, .ki = 0.0f, .kd = 0.05f};
// 只有一边能比较顺滑的回正，但是到达平衡点会小幅震荡，回正速度没有很快

/*
    $K_d$ 合适：飞机迅速回到原位并瞬间纹丝不动（干净利落）。
    $K_d$ 过小：飞机回到原位后会多晃动 2-3 下（像果冻一样）。回正过冲。弹簧效应、
    $K_d$ 过大：飞机发出高频的嗡嗡声或金属震动声。
*/

// PID_Struct pid_pitch = {.kp = 6.8f, .ki = 0.0f, .kd = 0.0f};
// PID_Struct pid_gyro_y = {.kp = 1.2f, .ki = 0.0f, .kd = 0.03f};
PID_Struct pid_pitch = {.kp = 6.9f, .ki = 0.0f, .kd = 0.0f};
PID_Struct pid_gyro_y = {.kp = 1.2f, .ki = 0.0f, .kd = 0.04f};

// PID_Struct pid_pitch = {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f};
// PID_Struct pid_gyro_y = {.kp = 1.2f, .ki = 0.00f, .kd = 0.05f};
// PID_Struct pid_pitch = {.kp = 5.5f, .ki = 0.0f, .kd = 0.0f};
// PID_Struct pid_gyro_y = {.kp = 1.1f, .ki = 0.015f, .kd = 0.07f};

PID_Struct pid_roll = {.kp = 6.9f, .ki = 0.0f, .kd = 0.0f};
PID_Struct pid_gyro_x = {.kp = 1.2f, .ki = 0.0f, .kd = 0.04f};

PID_Struct pid_yaw = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f};
PID_Struct pid_gyro_z = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f};

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

PID_Struct pid_height = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f};

void HeightPidCtrl(FLOW_Struct *flow, float dt)
{
    pid_height.desire = ABS(flight_rc_data.THR - 1000) * 0.4f;
    // vTaskSuspendAll(); // 暂停调度，防止被 FlowTask 抢占
    pid_height.measure = flow->flow_High;
    PID_Calculate(&pid_height, dt);
    // xTaskResumeAll();  // 恢复调度
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
        // // 左上
        // motor_pwm[2] = Limit(flight_rc_data.THR, 1000.0f, 1800.0f) + (- pid_gyro_x.output + pid_gyro_y.output - pid_gyro_z.output);
        // // 右上
        // motor_pwm[1] = Limit(flight_rc_data.THR, 1000.0f, 1800.0f) + (pid_gyro_x.output + pid_gyro_y.output + pid_gyro_z.output);
        // // 左下
        // motor_pwm[3] = Limit(flight_rc_data.THR, 1000.0f, 1800.0f) + (- pid_gyro_x.output - pid_gyro_y.output + pid_gyro_z.output); 
        // // 右下
        // motor_pwm[0] = Limit(flight_rc_data.THR, 1000.0f, 1800.0f) + (pid_gyro_x.output - pid_gyro_y.output - pid_gyro_z.output);

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

