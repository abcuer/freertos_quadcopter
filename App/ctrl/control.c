#include "control.h"
#include "pid.h"
#include "motor.h"
#include "remote.h"
#include "imu.h"

// 定义陀螺仪和加速度
Gyro_Acc_Struct gyro_acc;
// 定义欧拉角
EulerAngle_Struct euler_angle;


PID_t rate_roll_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f,
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
    .max_out = 800.0f,  
    .mode = POSITION_PID,
};

PID_t rate_pitch_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f, // PWM最大值（2500 - 1000）
    .mode = POSITION_PID, 
};

PID_t pitch_pid = 
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .tar = 0.0f,
    .deadband = 0.5f,
    .max_out = 800.0f, 
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
    .max_out = 800.0f,  
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
    .max_out = 800.0f,  
    .mode = POSITION_PID,
};

void PIDParam_Init(void)
{
    PID_Init(&pitch_pid);
    PID_Init(&rate_pitch_pid);
    PID_Init(&roll_pid);
    PID_Init(&rate_roll_pid);
    PID_Init(&yaw_pid);
    PID_Init(&rate_yaw_pid);
}

static void RollPidCtrl(void)
{
    // 1500处为0度，区间约+-30度
    roll_pid.tar  = (flight_rc_data.ROL - 1500) * 0.06f; 
    roll_pid.now = euler_angle.roll;
    PidCalculate(&roll_pid);
}

static void RateRollPID(void)
{
    rate_roll_pid.tar = roll_pid.out;
    rate_roll_pid.now = gyro_acc.gyro.x * Gyro_G;
    PidCalculate(&rate_roll_pid);
}

static void PitchPidCtrl(void)
{
    pitch_pid.tar = (flight_rc_data.PIT - 1500) * 0.06f;
    pitch_pid.now = euler_angle.pitch;
    PidCalculate(&pitch_pid);
}

static void RatePitchPID(void)
{
    rate_pitch_pid.tar = pitch_pid.out;
    rate_pitch_pid.now = gyro_acc.gyro.y * Gyro_G;
    PidCalculate(&rate_pitch_pid);
}

static void YawPidCtrl(void)
{
    yaw_pid.now = euler_angle.yaw;
    PidCalculate(&yaw_pid);
}

static void RateYawPID(void)
{
    // YAW 通常建议映射为内环的“期望旋转速度”，例   如 +-150度/秒
    rate_yaw_pid.tar = (flight_rc_data.YAW - 1500) * 0.3f + yaw_pid.out;
    rate_yaw_pid.now = gyro_acc.gyro.z * Gyro_G;
    PidCalculate(&rate_yaw_pid);
}

float motor_pwm[4];
void FlyControl(void)
{
    // 外环
    RollPidCtrl();  
    PitchPidCtrl(); 
    YawPidCtrl();   
    // 内环
    RateRollPID();  // x
    RatePitchPID(); // y
    RateYawPID();   // z

    motor_pwm[0] = flight_rc_data.THR + rate_roll_pid.out - rate_pitch_pid.out - rate_yaw_pid.out;
    motor_pwm[1] = flight_rc_data.THR + rate_roll_pid.out + rate_pitch_pid.out + rate_yaw_pid.out;
    motor_pwm[2] = flight_rc_data.THR - rate_roll_pid.out + rate_pitch_pid.out - rate_yaw_pid.out;
    motor_pwm[3] = flight_rc_data.THR - rate_roll_pid.out - rate_pitch_pid.out + rate_yaw_pid.out;

    motor_pwm[0] = Limit(motor_pwm[0], 1000, 2000);
    motor_pwm[1] = Limit(motor_pwm[1], 1000, 2000);
    motor_pwm[2] = Limit(motor_pwm[2], 1000, 2000);
    motor_pwm[3] = Limit(motor_pwm[3], 1000, 2000);

    SetMotorPWM(motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
}

