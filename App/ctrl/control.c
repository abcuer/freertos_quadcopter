#include "control.h"
#include "pid.h"
#include "motor.h"

IMU_t bmi;   
float basepwm = 0.0f;

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
    pitch_pid.now = bmi.pitch;
    PidCalculate(&pitch_pid);
}

void RatePitchPID(void)
{
    rate_pitch_pid.tar = pitch_pid.out;
    rate_pitch_pid.now = bmi.gyro[0];
    PidCalculate(&rate_pitch_pid);
}

void RollPidCtrl(void)
{
    roll_pid.now = bmi.roll;
    PidCalculate(&roll_pid);
}

void RateRollPID(void)
{
    rate_roll_pid.tar = roll_pid.out;
    rate_roll_pid.now = bmi.gyro[1];
    PidCalculate(&rate_roll_pid);
}

void YawPidCtrl(void)
{
    yaw_pid.now = bmi.yaw;
    PidCalculate(&yaw_pid);
}

void RateYawPID(void)
{
    rate_yaw_pid.tar = yaw_pid.out;
    rate_yaw_pid.now = bmi.gyro[2];
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

