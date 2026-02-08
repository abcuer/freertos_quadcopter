#include "imu.h"
#include "math.h"

#define sampleFreq 125.0f // sample frequency in Hz
/*采样周期的一半，用于求解四元数微分方程时计算角增量
请确定自己的姿态调用周期: 8ms,即上面的sampleFreq: 125Hz*/
#define halfT 0.004f

//这里的Kp,Ki是用于控制加速度计修正陀螺仪积分姿态的速度
float Kp = 1.8f;  //2.0f
float Ki = 0.0016f;  //0.002f

float gx=0, gy=0, gz=0; //由角速度计算的角速率
float ax=0, ay=0, az=0; //由加速度计算的加速度

float acc_sp[3]; //积分速度
float acc_sp_RHRH[3]; //处理后速度

float bd_gx=0, bd_gy=0, bd_gz=0;
float bd_ax=0, bd_ay=0, bd_az=0;

//初始姿态四元数(地理坐标系)，q(q0,q1,q2,q3)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    //最优估计四元数
float q0_yaw = 1.0f, q1_yaw = 0.0f, q2_yaw = 0.0f, q3_yaw = 0.0f;    //弥补Mahony算法在无地磁情况解算Yaw轴满足不了大扰动要求的现象

//定义姿态解算误差的积分
//当前加计测得的重力加速度在三轴(x,y,z)上的分量,与当前姿态计算得来的重力在三轴上的分量的误差的积分

float xErrorInt = 0.0f, yErrorInt = 0.0f, zErrorInt = 0.0f;

float yaw_last = 0.0f;       // 上一次计算出的原始 Yaw 值
float yaw_offset_sum = 0.0f; // 累加的误差补偿值
float yaw_final = 0.0f;      // 最终修正后的 Yaw 输出
/*
 * 姿态融合
 * 单位: m/s^2   rad/s
*/
static void ImuUpdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    static uint8_t yaw_init_f = 0; // 静态变量，用于初始化
    // --- 1. Yaw轴独立积分系统 ---
    // 为了节省开销，直接使用传入的角速度进行四元数更新
    float q0_y_old = q0_yaw, q1_y_old = q1_yaw, q2_y_old = q2_yaw, q3_y_old = q3_yaw;
    q0_yaw += (-q1_y_old * gx - q2_y_old * gy - q3_y_old * gz) * halfT;
    q1_yaw += ( q0_y_old * gx + q2_y_old * gz - q3_y_old * gy) * halfT;
    q2_yaw += ( q0_y_old * gy - q1_y_old * gz + q3_y_old * gx) * halfT;
    q3_yaw += ( q0_y_old * gz + q1_y_old * gy - q2_y_old * gx) * halfT;

    norm = sqrt(q0_yaw*q0_yaw + q1_yaw*q1_yaw + q2_yaw*q2_yaw + q3_yaw*q3_yaw);
    q0_yaw /= norm; q1_yaw /= norm; q2_yaw /= norm; q3_yaw /= norm;

    // --- 2. 主姿态(Pitch/Roll)解算系统 ---
    norm = sqrt(ax * ax + ay * ay + az * az); 
    if(norm < 0.1f) return;
    ax /= norm; ay /= norm; az /= norm;

    vx = 2 * (q1*q3 - q0*q2);                                            
    vy = 2 * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    ex = (ay * vz - az * vy);      
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    xErrorInt += ex * Ki; yErrorInt += ey * Ki; zErrorInt += ez * Ki;
    gx += Kp * ex + xErrorInt; gy += Kp * ey + yErrorInt; gz += Kp * ez + zErrorInt;            

    float q0_old = q0, q1_old = q1, q2_old = q2, q3_old = q3;
    q0 += (-q1_old * gx - q2_old * gy - q3_old * gz) * halfT;
    q1 += ( q0_old * gx + q2_old * gz - q3_old * gy) * halfT;
    q2 += ( q0_old * gy - q1_old * gz + q3_old * gx) * halfT;
    q3 += ( q0_old * gz + q1_old * gy - q2_old * gx) * halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    // --- 3. 欧拉角提取与动态死区补偿 ---
    bmi.pitch = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 57.3f - 3.3f;
    bmi.roll  = asin(2*(q0*q2 - q1*q3)) * 57.3f - 45.3f;

    // 提取原始Yaw
    float yaw_raw = atan2(2*(q1_yaw*q2_yaw + q0_yaw*q3_yaw), 1 - 2*(q2_yaw*q2_yaw + q3_yaw*q3_yaw)) * 57.3f;

    // 初始化处理
    if(!yaw_init_f) {
        yaw_last = yaw_raw;
        yaw_init_f = 1;
    }
    float error = yaw_raw - yaw_last;
    if (error > 300.0f) error -= 360.0f;
    else if (error < -300.0f) error += 360.0f;
    // 算法核心：忽略微小变化（死区判定）
    if (fabs(error) < 0.01f) {
        yaw_offset_sum += error;
    }

    yaw_last = yaw_raw;
    bmi.yaw = yaw_raw - yaw_offset_sum;

    // 角度标准化
    if (bmi.yaw > 180.0f) bmi.yaw -= 360.0f;
    else if (bmi.yaw < -180.0f) bmi.yaw += 360.0f;
}

void IMU_GetAngle(uint8_t on)
{
    BMI088_GetData(&bmi);
	//陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
	//陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

	//加速度计量程为:±2g        获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±4g        获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±8g        获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
	//加速度计量程为:±16g       获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)

	ax = (float)bmi.acc_raw[0] * 0.000183105f; 
	ay = (float)bmi.acc_raw[1] * 0.000183105f;
	az = (float)bmi.acc_raw[2] * 0.000183105f;

	bmi.gyro[0] = (float)bmi.gyro_raw[0] * 0.0609756f;
	bmi.gyro[1] = (float)bmi.gyro_raw[1] * 0.0609756f;
	bmi.gyro[2] = (float)bmi.gyro_raw[2] * 0.0609756f;

	bd_gx = bmi.gyro[0];
	bd_gy = bmi.gyro[1];
	bd_gz = bmi.gyro[2];

	bd_ax = 1.0064f*ax - 717.6192f;
	bd_ay = 1.0104f*ay + 39.3216f;
	bd_az = 0.9762f*az + 704.5120f;

	LPF_1_(1.8f, 0.002f, bd_gx*0.0174533f, gx);
	LPF_1_(1.8f, 0.002f, bd_gy*0.0174533f, gy);

	acc_sp[0] += (az-0.995f)/3.0f;
	acc_sp[1] += ax * 5.2f;
	acc_sp[2] -= ay * 5.2f;

	if(on == 1)
		ImuUpdate(bd_gx*0.0174533f, bd_gy*0.0174533f, bd_gz*0.0174533f, bd_ax, bd_ay, bd_az);
}