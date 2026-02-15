#include "control.h"
#include "pid.h"
#include "motor.h"
#include "remote.h"
#include "imu.h"

// 定义陀螺仪和加速度
Gyro_Acc_Struct gyro_acc;
// 定义欧拉角
EulerAngle_Struct euler_angle;

PID_t roll_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.3f,
    .max_out = 800.0f,  
    .max_iout = 800.0f, 
    .mode = POSITION_PID,
};

PID_t pitch_pid = 
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .tar = 0.0f,
    .deadband = 0.3f,
    .max_out = 800.0f, 
    .max_iout = 800.0f, 
    .mode = POSITION_PID,
};

PID_t yaw_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.3f,
    .max_out = 800.0f,  
    .max_iout = 800.0f, 
    .mode = POSITION_PID,
};

PID_t rate_roll_pid =
{
    .p = 0.0f, 
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.3f,
    .max_out = 800.0f,
    .max_iout = 800.0f, 
    .mode = POSITION_PID, 
};

/*
rate_pitch:
    给固定油门
先i
    i值太小：松开手，飞机一直迅速朝一个方向下降
    i值太大，松开手，飞机大幅度范围内来回震荡
    理想i值，松开手，飞机小幅度范围内来回震荡（在30度范围）
再p：p抑制i的效果，
    p值为正值时，从小往上推无人机时，无人机本来会有向上的推力，但是无人机努力向下压，向我的手靠近
    p值为负值值，从小往上推无人机时，无人机有向上的趋势; 向下压无人机时，无人机有向下的趋势
    
    p值太小，松开手，会很缓慢朝一个方向下降
    p值太大，松开手，飞机接近平衡，施加干扰，飞机会在平衡点上来回震荡，硬
    理想p值，松开手，飞机接近平衡，施加干扰，飞机会凯苏回到在平衡点上，硬
*/ 
PID_t rate_pitch_pid =
{
    .p = 1.2f,
    .i = 0.0f,
    .d = 0.1f,
    .tar = 0.0f,
    .deadband = 0.3f,
    .max_out = 800.0f, // PWM最大值（2500 - 1000）
    .max_iout = 800.0f, 
    .mode = POSITION_PID, 
};

PID_t rate_yaw_pid =
{
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    // 机械零点
    .tar = 0.0f,
    .deadband = 0.3f,
    .max_out = 800.0f, 
    .max_iout = 800.0f, 
    .mode = POSITION_PID, 
};

void PIDParam_Init(void)
{
    PID_Init(&roll_pid);
    PID_Init(&rate_roll_pid);
    PID_Init(&pitch_pid);
    PID_Init(&rate_pitch_pid);
    PID_Init(&yaw_pid);
    PID_Init(&rate_yaw_pid);
}

static void RollPidCtrl(void)
{
    // 1500处为0度，区间约+-30度
    roll_pid.tar  = (flight_rc_data.ROL - 1500) * 0.08f; 
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
    pitch_pid.tar = (flight_rc_data.PIT - 1500) * 0.08f;
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

    if(flight_rc_data.CONNECT && flight_rc_data.THR >= 1050)
    {
        motor_pwm[0] = flight_rc_data.THR + rate_roll_pid.out - rate_pitch_pid.out - rate_yaw_pid.out;
        motor_pwm[1] = flight_rc_data.THR + rate_roll_pid.out + rate_pitch_pid.out + rate_yaw_pid.out;
        motor_pwm[2] = flight_rc_data.THR - rate_roll_pid.out + rate_pitch_pid.out - rate_yaw_pid.out;
        motor_pwm[3] = flight_rc_data.THR - rate_roll_pid.out - rate_pitch_pid.out + rate_yaw_pid.out;        
        
        motor_pwm[0] = Limit(motor_pwm[0], 1000, 2000);
        motor_pwm[1] = Limit(motor_pwm[1], 1000, 2000);
        motor_pwm[2] = Limit(motor_pwm[2], 1000, 2000);
        motor_pwm[3] = Limit(motor_pwm[3], 1000, 2000);
    }
    else 
    {
        motor_pwm[0] = motor_pwm[1] = motor_pwm[2] = motor_pwm[3] = 0;
        PIDParam_Init();
    }

    SetMotorPWM(motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
}

