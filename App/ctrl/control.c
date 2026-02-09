#include "control.h"
#include "pid.h"
#include "motor.h"
#include "filter.h"
#include "imu.h"

float basepwm = 0.0f;

// 定义陀螺仪和加速度
Gyro_Acc_Struct gyro_acc;
// 定义欧拉角
EulerAngle_Struct euler_angle;

PID_t rate_pitch_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f, // PWM最大值 1000
    .mode = POSITION_PID, 
};

PID_t pitch_pid = 
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f, // PWM最大值 1000
    .mode = POSITION_PID,
};

PID_t rate_roll_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f, // PWM最大值 1000
    .mode = POSITION_PID, 
};

PID_t roll_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f,  // PWM最大值 1000
    .mode = POSITION_PID,
};

PID_t rate_yaw_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f,  // PWM最大值 1000
    .mode = POSITION_PID, 
};

PID_t yaw_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f,  // PWM最大值 1000
    .mode = POSITION_PID,
};

static Gyro_Struct last_gyro_filtered = {0};

void IMU_Get_Gyro_Acc(Gyro_Acc_Struct *gyro_acc)
{
    // 读取数据
    BMI088_Read(gyro_acc);
    
    // 陀螺仪低通滤波（使用浮点计算）
    float alpha = 0.18f;
    
    // 转换为浮点计算，再转回整数
    last_gyro_filtered.x = (int16_t)(alpha * gyro_acc->gyro.x + 
                                    (1.0f - alpha) * last_gyro_filtered.x);
    last_gyro_filtered.y = (int16_t)(alpha * gyro_acc->gyro.y + 
                                    (1.0f - alpha) * last_gyro_filtered.y);
    last_gyro_filtered.z = (int16_t)(alpha * gyro_acc->gyro.z + 
                                    (1.0f - alpha) * last_gyro_filtered.z);
    
    gyro_acc->gyro = last_gyro_filtered;
    
    // 加速度计卡尔曼滤波
    gyro_acc->acc.x = (int16_t)Filter_KalmanFilter(&kfs[0], (double)gyro_acc->acc.x);
    gyro_acc->acc.y = (int16_t)Filter_KalmanFilter(&kfs[1], (double)gyro_acc->acc.y);
    gyro_acc->acc.z = (int16_t)Filter_KalmanFilter(&kfs[2], (double)gyro_acc->acc.z);
}

void IMU_Get_EulerAngle(Gyro_Acc_Struct *gyro_acc, EulerAngle_Struct *euler_angle, float dt)
{
    IMU_GetEulerAngle(gyro_acc, euler_angle, dt);
}

void PIDParam_Init(void)
{
    PID_Init(&pitch_pid);
    PID_Init(&rate_pitch_pid);
    PID_Init(&roll_pid);
    PID_Init(&rate_roll_pid);
    PID_Init(&yaw_pid);
    PID_Init(&rate_yaw_pid);
}

void PitchPidCtrl(void)
{
    pitch_pid.now = euler_angle.pitch;
    PidCalculate(&pitch_pid);
}

void RatePitchPID(void)
{
    rate_pitch_pid.tar = pitch_pid.out;
    rate_pitch_pid.now = gyro_acc.gyro.x;
    PidCalculate(&rate_pitch_pid);
}

void RollPidCtrl(void)
{
    roll_pid.now = euler_angle.roll;
    PidCalculate(&roll_pid);
}

void RateRollPID(void)
{
    rate_roll_pid.tar = roll_pid.out;
    rate_roll_pid.now = gyro_acc.gyro.y;
    PidCalculate(&rate_roll_pid);
}

void YawPidCtrl(void)
{
    yaw_pid.now = euler_angle.yaw;
    PidCalculate(&yaw_pid);
}

void RateYawPID(void)
{
    rate_yaw_pid.tar = yaw_pid.out;
    rate_yaw_pid.now = gyro_acc.gyro.z;
    PidCalculate(&rate_yaw_pid);
}

void FlyControl(void)
{
    float motor_pwm[4];
    motor_pwm[0] = basepwm + rate_pitch_pid.out - rate_roll_pid.out - rate_yaw_pid.out;
    motor_pwm[1] = basepwm + rate_pitch_pid.out + rate_roll_pid.out + rate_yaw_pid.out;
    motor_pwm[2] = basepwm - rate_pitch_pid.out + rate_roll_pid.out - rate_yaw_pid.out;
    motor_pwm[3] = basepwm - rate_pitch_pid.out - rate_roll_pid.out + rate_yaw_pid.out;
    MotorSetPWM(motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
}

