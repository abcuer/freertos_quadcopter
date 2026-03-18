#include "imu.h"
#include "filter.h"
#include "math.h"
/* ============================欧拉角计算================================== */
/* ===============================开始===================================== */

/* 计算欧拉角用到的3个参数 */
// float RtA = 57.2957795f;  // 弧度->度
// float Gyro_G = 4000.0 / 65536.0;  // 度/s
// float Gyro_Gr = 4000.0 / 65536.0 / 180.0 * 3.1415926; // 弧度/s

float Gyro_G  = 1.0f / 16.384f;               // 度/s每LSB
float Gyro_Gr = (1.0f / 16.384f) * RTOM;      // 弧度/s每LSB

/**
 * @description: 快速计算 1/sqrt(num)
 * @param {float} number
 */
static float Q_rsqrt(float number)
{
    long        i;
    float       x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = *(long *)&y;
    i  = 0x5f3759df - (i >> 1);
    y  = *(float *)&i;
    y  = y * (threehalfs - (x2 * y * y));   // 1st iteration （第一次牛顿迭代）
    return y;
}

static Gyro_Struct last_gyro_filtered = {0};
void IMU_Get_GyroAcc(Gyro_Acc_Struct *gyro_acc)
{
    // 读取数据
    BMI088_Read(gyro_acc);
    
    // 陀螺仪低通滤波（使用浮点计算）
    float alpha = 0.15f;
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

#define BMI088_GYRO_SENSITIVITY 16.384f
#define BMI088_ACC_SENSITIVITY 5461.0f

static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
static float yaw_drift_compensation = 0;  // Yaw漂移补偿值
static uint32_t stationary_samples = 0;
static float gz_history[10] = {0};  // 历史Gz值
static uint8_t gz_index = 0;

static double normAccz; /* z轴上的加速度 */
/**
 * @description: 修复版姿态解算（互补滤波 + 动态零偏补偿）
 * @param {Gyro_Acc_Struct} *gyroAccel 传感器原始数据
 * @param {EulerAngle_Struct} *eulerAngle 欧拉角输出
 * @param {float} dt 两次函数调用的时间间隔（秒）
 */
void IMU_Get_EulerAngle(Gyro_Acc_Struct *gyroAccel,
                        EulerAngle_Struct *eulerAngle,
                        float dt)
{
    // 1. 物理值转换
    // 加速度计单位: g, 陀螺仪单位: °/s
    float ax = (float)gyroAccel->acc.x / BMI088_ACC_SENSITIVITY;
    float ay = (float)gyroAccel->acc.y / BMI088_ACC_SENSITIVITY;
    float az = (float)gyroAccel->acc.z / BMI088_ACC_SENSITIVITY;
    
    float gx_raw = (float)gyroAccel->gyro.x / BMI088_GYRO_SENSITIVITY;
    float gy_raw = (float)gyroAccel->gyro.y / BMI088_GYRO_SENSITIVITY;
    float gz_raw = (float)gyroAccel->gyro.z / BMI088_GYRO_SENSITIVITY;

    // 2. 静态零偏动态估计 (针对静止状态微调，防止温漂)
    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    
    // 静止检测条件：加速度接近1g 且 角速度波动极小
    int is_stationary = (fabsf(acc_mag - 1.0f) < 0.05f) && 
                        (fabsf(gx_raw) < 0.2f) && 
                        (fabsf(gy_raw) < 0.2f);

    if (is_stationary) {
        stationary_samples++;
        if (stationary_samples > 200) { // 持续静止一段时间后微调零偏
            float alpha_bias = 0.001f; // 极小的更新权重，防止误调
            gyro_bias_x = (1.0f - alpha_bias) * gyro_bias_x + alpha_bias * gx_raw;
            gyro_bias_y = (1.0f - alpha_bias) * gyro_bias_y + alpha_bias * gy_raw;
            // Yaw轴零偏通常只在长时间完全静止时更新
            if (stationary_samples > 1000) {
                gyro_bias_z = (1.0f - alpha_bias) * gyro_bias_z + alpha_bias * gz_raw;
            }
        }
    } else {
        stationary_samples = 0;
    }

    // 3. 应用补偿后的角速度
    float gx = gx_raw - gyro_bias_x;
    float gy = gy_raw - gyro_bias_y;
    float gz = gz_raw - gyro_bias_z;

    // 4. 计算加速度计提供的观察角 (度)
    // 注意：BMI088 背面安装时，Z轴方向可能需要根据你的 Read_Acc 转换结果确认符号
    float acc_roll  = atan2f(ay, az) * 57.29578f;
    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.29578f;

    // 5. 互补滤波 (Complementary Filter)
    // 核心公式: Angle = K * (Angle + Gyro * dt) + (1-K) * Acc_Angle
    // K值越大，越信任陀螺仪，越离手（抗震好，但随时间漂移）；K值越小，越信任加速度计，越跟手（不漂移，但怕震动）
    float K = 0.85f; 

    // Roll & Pitch 融合解算
    eulerAngle->roll  = K * (eulerAngle->roll  + gy * dt) + (1.0f - K) * acc_roll;
    eulerAngle->pitch = K * (eulerAngle->pitch + gx * dt) + (1.0f - K) * acc_pitch;

    // 6. Yaw 轴处理
    // Yaw轴无法通过加速度计修正，只能纯积分。
    // 如果有磁力计，可在此处加入磁力计修正。
    
    // Gz 死区处理：消除静止时的微小爬行
    if (fabsf(gz) < 0.08f) gz = 0; 
    
    // Yaw 轴积分
    eulerAngle->yaw += gz * dt;

    // 7. 角度归一化 (-180 到 180 度)
    if (eulerAngle->yaw > 180.0f)  eulerAngle->yaw -= 360.0f;
    if (eulerAngle->yaw < -180.0f) eulerAngle->yaw += 360.0f;
    if (eulerAngle->pitch > 180.0f) eulerAngle->pitch -=360.0f;
    if (eulerAngle->pitch < -180.0f) eulerAngle->pitch +=360.0f;
    if (eulerAngle->roll > 180.0f) eulerAngle->roll -=360.0f;
    if (eulerAngle->roll < -180.0f) eulerAngle->roll +=360.0f;    
}

/* 稳定可用的pitch、roll，yaw变化太快，实际15度，显示一百多度但能稳定在某个值 */
// void IMU_GetEulerAngle(Gyro_Acc_Struct *gyroAccel,
//                              EulerAngle_Struct *eulerAngle,
//                              float dt)
// {
//     #define BMI088_GYRO_SENSITIVITY 16.384f
//     #define BMI088_ACC_SENSITIVITY 5461.0f
    
//     static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
//     static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
//     static uint32_t stationary_time = 0;
//     static float last_gz_raw = 0;
    
//     // ============ 转换为物理值 ============
//     float ax = (float)gyroAccel->acc.x / BMI088_ACC_SENSITIVITY;
//     float ay = (float)gyroAccel->acc.y / BMI088_ACC_SENSITIVITY;
//     float az = (float)gyroAccel->acc.z / BMI088_ACC_SENSITIVITY;
    
//     float gx_raw = (float)gyroAccel->gyro.x / BMI088_GYRO_SENSITIVITY;
//     float gy_raw = (float)gyroAccel->gyro.y / BMI088_GYRO_SENSITIVITY;
//     float gz_raw = (float)gyroAccel->gyro.z / BMI088_GYRO_SENSITIVITY;
    
//     // ============ 改进的零偏估计 ============
//     float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    
//     // 更严格的静止检测
//     int is_stationary = (fabsf(acc_mag - 1.0f) < 0.05f) &&      // 加速度接近1g
//                         (fabsf(gx_raw) < 0.5f) &&              // 角速度很小
//                         (fabsf(gy_raw) < 0.5f) &&
//                         (fabsf(gz_raw) < 0.5f);
    
//     if (is_stationary) {
//         stationary_time++;
        
//         // Roll/Pitch零偏估计（较快）
//         if (stationary_time > 50) {  // 0.3秒
//             float alpha_rp = 0.01f;  // 较快的收敛速度
//             gyro_bias_x = (1.0f - alpha_rp) * gyro_bias_x + alpha_rp * gx_raw;
//             gyro_bias_y = (1.0f - alpha_rp) * gyro_bias_y + alpha_rp * gy_raw;
            
//             // 限制零偏范围
//             gyro_bias_x = fmaxf(fminf(gyro_bias_x, 1.0f), -1.0f);
//             gyro_bias_y = fmaxf(fminf(gyro_bias_y, 1.0f), -1.0f);
//         }
        
//         // Yaw零偏估计（非常慢，需要长时间静止）
//         if (stationary_time > 500) {  // 3秒完全静止
//             float alpha_yaw = 0.0001f;  // 非常慢的收敛
//             gyro_bias_z = (1.0f - alpha_yaw) * gyro_bias_z + alpha_yaw * gz_raw;
//             gyro_bias_z = fmaxf(fminf(gyro_bias_z, 0.2f), -0.2f);  // 限制很小
//         }
//     } else {
//         stationary_time = 0;
//     }
    
//     // 应用零偏补偿
//     float gx = gx_raw - gyro_bias_x;
//     float gy = gy_raw - gyro_bias_y;
//     float gz = gz_raw - gyro_bias_z;
    
//     // ============ Yaw特殊处理 ============
//     // 1. 死区滤波（消除小信号噪声）
//     float gz_filtered = gz;
//     if (fabsf(gz_filtered) < 0.1f) {  // 0.1°/s死区
//         gz_filtered = 0;
//     }
    
//     // 2. 低通滤波减少噪声
//     static float gz_lpf = 0;
//     float lpf_alpha = 0.3f;  // 低通滤波系数
//     gz_lpf = (1.0f - lpf_alpha) * gz_lpf + lpf_alpha * gz_filtered;
    
//     // 3. 积分
//     gyro_yaw += gz_lpf * dt;
    
//     // ============ Roll/Pitch处理 ============
//     gyro_roll += gy * dt;
//     gyro_pitch += gx * dt;
    
//     // ============ 加速度计角度 ============
//     float acc_roll = atan2f(ay, az) * 57.29578f;
    
//     float denom = sqrtf(ay*ay + az*az);
//     float acc_pitch = (denom > 0.01f) ? atan2f(-ax, denom) * 57.29578f : 0;
    
//     // ============ 自适应权重 ============
//     float weight;
//     if (acc_mag > 1.2f || acc_mag < 0.8f) {
//         weight = 0.2f;  // 运动状态
//     } else {
//         weight = 0.98f; // 静止状态
//     }
    
//     // ============ 融合 ============
//     eulerAngle->roll = weight * acc_roll + (1.0f - weight) * gyro_roll;
//     eulerAngle->pitch = weight * acc_pitch + (1.0f - weight) * gyro_pitch;
//     eulerAngle->yaw = gyro_yaw;
    
//     // ============ 角度限制 ============
//     // Roll/Pitch限制
//     if (eulerAngle->roll > 180.0f) eulerAngle->roll -= 360.0f;
//     if (eulerAngle->roll < -180.0f) eulerAngle->roll += 360.0f;
//     if (eulerAngle->pitch > 180.0f) eulerAngle->pitch -= 360.0f;
//     if (eulerAngle->pitch < -180.0f) eulerAngle->pitch += 360.0f;
    
//     // Yaw限制和修正
//     if (eulerAngle->yaw > 180.0f) eulerAngle->yaw -= 360.0f;
//     if (eulerAngle->yaw < -180.0f) eulerAngle->yaw += 360.0f;
// }

/* 稳定可用的pitch和roll，yaw变化太快 */
// void IMU_GetEulerAngle(Gyro_Acc_Struct *gyroAccel,
//                         EulerAngle_Struct *eulerAngle,
//                                             float dt)
// {
//     #define BMI088_GYRO_SENSITIVITY 16.384f
//     #define BMI088_ACC_SENSITIVITY 5461.0f
    
//     // 静态变量保持状态
//     static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
//     static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
//     static uint32_t stationary_time = 0;
    
//     // 转换为物理值
//     float ax = (float)gyroAccel->acc.x / BMI088_ACC_SENSITIVITY;
//     float ay = (float)gyroAccel->acc.y / BMI088_ACC_SENSITIVITY;
//     float az = (float)gyroAccel->acc.z / BMI088_ACC_SENSITIVITY;
    
//     float gx_raw = (float)gyroAccel->gyro.x / BMI088_GYRO_SENSITIVITY;  // °/s
//     float gy_raw = (float)gyroAccel->gyro.y / BMI088_GYRO_SENSITIVITY;
//     float gz_raw = (float)gyroAccel->gyro.z / BMI088_GYRO_SENSITIVITY;
    
//     // ============ 零偏估计 ============
//     float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
//     float gyro_mag = sqrtf(gx_raw*gx_raw + gy_raw*gy_raw);
    
//     // 静止检测
//     if (fabsf(acc_mag - 1.0f) < 0.1f && gyro_mag < 1.0f) {
//         stationary_time++;
//         if (stationary_time > 100) {  // 0.6秒
//             float alpha = 0.0005f;
//             gyro_bias_x = (1.0f - alpha) * gyro_bias_x + alpha * gx_raw;
//             gyro_bias_y = (1.0f - alpha) * gyro_bias_y + alpha * gy_raw;
//             gyro_bias_z = (1.0f - alpha) * gyro_bias_z + alpha * gz_raw;
//         }
//     } else {
//         stationary_time = 0;
//     }
    
//     // 应用零偏补偿
//     float gx = gx_raw - gyro_bias_x;
//     float gy = gy_raw - gyro_bias_y;
//     float gz = gz_raw - gyro_bias_z;
    
//     // ============ 陀螺仪积分 ============
//     // 注意轴对应关系：
//     // - Roll (绕X轴旋转) 对应 Gy (Y轴角速度)
//     // - Pitch (绕Y轴旋转) 对应 Gx (X轴角速度)  
//     // - Yaw (绕Z轴旋转) 对应 Gz (Z轴角速度)
//     gyro_roll += gy * dt;
//     gyro_pitch += gx * dt;  // 注意这里！Gx对应pitch
//     gyro_yaw += gz * dt;
    
//     // ============ 加速度计计算角度 ============
//     float acc_roll, acc_pitch;
    
//     // Roll: atan2(Ay, Az)
//     acc_roll = atan2f(ay, az) * 57.29578f;
    
//     // Pitch: atan2(-Ax, sqrt(Ay² + Az²))
//     float denom = sqrtf(ay*ay + az*az);
//     if (denom > 0.01f) {
//         acc_pitch = atan2f(-ax, denom) * 57.29578f;
//     } else {
//         acc_pitch = 0;  // 避免除零
//     }
    
//     // ============ 自适应权重 ============
//     float weight;
//     if (acc_mag > 1.2f || acc_mag < 0.8f) {
//         // 剧烈运动：信任陀螺仪
//         weight = 0.2f;  // 更小的权重给加速度计
//     } else {
//         // 正常状态：信任加速度计
//         weight = 0.98f;
//     }
    
//     // ============ 互补滤波融合 ============
//     eulerAngle->roll = weight * acc_roll + (1.0f - weight) * gyro_roll;
//     eulerAngle->pitch = weight * acc_pitch + (1.0f - weight) * gyro_pitch;
    
//     // ============ 偏航角处理 ============
//     // 简单积分，但添加死区减少噪声影响
//     if (fabsf(gz) > 0.5f) {  // 0.5°/s死区
//         eulerAngle->yaw = gyro_yaw;
//     }
//     // 限制角度范围
//     if (eulerAngle->roll > 180.0f) eulerAngle->roll -= 360.0f;
//     if (eulerAngle->roll < -180.0f) eulerAngle->roll += 360.0f;
//     if (eulerAngle->pitch > 180.0f) eulerAngle->pitch -= 360.0f;
//     if (eulerAngle->pitch < -180.0f) eulerAngle->pitch += 360.0f;
//     if (eulerAngle->yaw > 180.0f) eulerAngle->yaw -= 360.0f;
//     if (eulerAngle->yaw < -180.0f) eulerAngle->yaw += 360.0f;
// }

/* 较好的数据解算 */
// void IMU_GetEulerAngle(Gyro_Acc_Struct *gyroAccel,
//                                 EulerAngle_Struct *eulerAngle,
//                                 float dt)
// {
//     // BMI088灵敏度
//     #define BMI088_GYRO_SENSITIVITY 16.384f
//     #define BMI088_ACC_SENSITIVITY 5461.0f
//     #define DEG_TO_RAD 0.01745329251994329576f
    
//     static Quaternion_Struct q = {1, 0, 0, 0};
//     static float integralX = 0, integralY = 0, integralZ = 0;
//     static uint16_t times = 0;
    
//     // 转换为物理值
//     float ax = (float)gyroAccel->acc.x / BMI088_ACC_SENSITIVITY;
//     float ay = (float)gyroAccel->acc.y / BMI088_ACC_SENSITIVITY;
//     float az = (float)gyroAccel->acc.z / BMI088_ACC_SENSITIVITY;
    
//     float gx = (float)gyroAccel->gyro.x / BMI088_GYRO_SENSITIVITY * DEG_TO_RAD;
//     float gy = (float)gyroAccel->gyro.y / BMI088_GYRO_SENSITIVITY * DEG_TO_RAD;
//     float gz = (float)gyroAccel->gyro.z / BMI088_GYRO_SENSITIVITY * DEG_TO_RAD;
    
//     // ============ 自适应参数 ============
//     float Kp, Ki;
//     float accMag = ax*ax + ay*ay + az*az;
    
//     // 初始化阶段（前2.4秒）
//     if (times < 400) {
//         times++;
//         Kp = 8.0f;      // 较大的Kp快速收敛
//         Ki = 0.002f;    // 较大的Ki快速消除初始误差
//     } else {
//         // 动态调整：根据加速度幅值
//         // 1.44 = 1.2^2, 0.64 = 0.8^2
//         if (accMag > 1.44f || accMag < 0.64f) {
//             // 剧烈运动：更信任陀螺仪
//             Kp = 3.6f;
//             Ki = 0.001f;
//         } else {
//             // 静止或缓慢运动：信任加速度计
//             Kp = 4.8f;
//             Ki = 0.0015f;
//         }
//     }
    
//     // ============ Mahony算法核心 ============
//     if (accMag > 0.01f) {  // 加速度数据有效
//         float recipNorm = Q_rsqrt(accMag);
//         ax *= recipNorm;
//         ay *= recipNorm;
//         az *= recipNorm;
        
//         // 计算重力方向误差
//         float vx = 2.0f * (q.q1 * q.q3 - q.q0 * q.q2);
//         float vy = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
//         float vz = q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3;
        
//         float ex = ay * vz - az * vy;
//         float ey = az * vx - ax * vz;
//         float ez = ax * vy - ay * vx;
        
//         // 积分项
//         if (Ki > 0.0f) {
//             integralX += ex * dt;
//             integralY += ey * dt;
//             integralZ += ez * dt;
            
//             gx += Ki * integralX;
//             gy += Ki * integralY;
//             gz += Ki * integralZ;
//         }
        
//         // 比例项
//         gx += Kp * ex;
//         gy += Kp * ey;
//         gz += Kp * ez;
//     }
    
//     // 四元数更新
//     float qDot0 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz);
//     float qDot1 = 0.5f * (q.q0 * gx + q.q2 * gz - q.q3 * gy);
//     float qDot2 = 0.5f * (q.q0 * gy - q.q1 * gz + q.q3 * gx);
//     float qDot3 = 0.5f * (q.q0 * gz + q.q1 * gy - q.q2 * gx);
    
//     q.q0 += qDot0 * dt;
//     q.q1 += qDot1 * dt;
//     q.q2 += qDot2 * dt;
//     q.q3 += qDot3 * dt;
    
//     // 归一化
//     float norm = Q_rsqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
//     q.q0 *= norm;
//     q.q1 *= norm;
//     q.q2 *= norm;
//     q.q3 *= norm;
    
//     // 计算欧拉角
//     float sinr_cosp = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
//     float cosr_cosp = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);
//     eulerAngle->roll = atan2f(sinr_cosp, cosr_cosp) * 57.29578f;
    
//     float sinp = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);
//     if (fabsf(sinp) >= 1.0f) {
//         eulerAngle->pitch = copysignf(3.14159265358979323846f / 2.0f, sinp) * 57.29578f;
//     } else {
//         eulerAngle->pitch = asinf(sinp) * 57.29578f;
//     }
    
//     // ============ 添加yaw解包裹 ============
//     static float unwrapped_yaw = 0;
//     static uint8_t first_run = 1;
    
//     float current_yaw = atan2f(2.0f * (q.q0 * q.q3 + q.q1 * q.q2), 
//                                1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3)) * 57.29578f;
    
//     if (first_run) {
//         unwrapped_yaw = current_yaw;
//         first_run = 0;
//     } else {
//         float diff = current_yaw - unwrapped_yaw;
//         if (diff > 180.0f) {
//             unwrapped_yaw += diff - 360.0f;
//         } else if (diff < -180.0f) {
//             unwrapped_yaw += diff + 360.0f;
//         } else {
//             unwrapped_yaw = current_yaw;
//         }
//     }
    
//     eulerAngle->yaw = unwrapped_yaw;
// }

/**
 * @brief 改进的Mahony算法，带自适应零偏补偿
 */
// void IMU_GetEulerAngle(Gyro_Acc_Struct  *gyroAccel,
//                                 EulerAngle_Struct *eulerAngle,
//                                 float dt)
// {
//     #define BMI088_GYRO_SENSITIVITY 16.384f
//     #define BMI088_ACC_SENSITIVITY  5461.0f
//     #define DEG_TO_RAD 0.01745329251994329576f
    
//     static Quaternion_Struct q = {1, 0, 0, 0};
//     static float integralFBx = 0, integralFBy = 0, integralFBz = 0;
//     static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
//     static uint32_t stationary_time = 0;
    
//     // 转换单位
//     float ax = (float)gyroAccel->acc.x / BMI088_ACC_SENSITIVITY;
//     float ay = (float)gyroAccel->acc.y / BMI088_ACC_SENSITIVITY;
//     float az = (float)gyroAccel->acc.z / BMI088_ACC_SENSITIVITY;
    
//     float gx_raw = (float)gyroAccel->gyro.x / BMI088_GYRO_SENSITIVITY;  // °/s
//     float gy_raw = (float)gyroAccel->gyro.y / BMI088_GYRO_SENSITIVITY;
//     float gz_raw = (float)gyroAccel->gyro.z / BMI088_GYRO_SENSITIVITY;
    
//     // ============ 自适应参数 ============
//     float Kp, Ki;
//     float acc_magnitude = sqrtf(ax*ax + ay*ay + az*az);
    
//     if (fabsf(acc_magnitude - 1.0f) < 0.2f) {
//         // 静止或缓慢运动：信任加速度计
//         Kp = 1.0f;     // 较大的比例增益
//         Ki = 0.005f;   // 较大的积分增益
        
//         // 更新零偏估计
//         stationary_time++;
//         if (stationary_time > 100) {  // 静止0.6秒以上
//             float alpha = 0.0005f;  // 很慢的零偏估计
//             gyro_bias_x = (1.0f - alpha) * gyro_bias_x + alpha * gx_raw;
//             gyro_bias_y = (1.0f - alpha) * gyro_bias_y + alpha * gy_raw;
//             gyro_bias_z = (1.0f - alpha) * gyro_bias_z + alpha * gz_raw;
//         }
//     } else {
//         // 剧烈运动：信任陀螺仪
//         Kp = 0.1f;     // 较小的比例增益
//         Ki = 0.0f;     // 关闭积分
//         stationary_time = 0;
//     }
    
//     // 应用零偏补偿
//     float gx = (gx_raw - gyro_bias_x) * DEG_TO_RAD;
//     float gy = (gy_raw - gyro_bias_y) * DEG_TO_RAD;
//     float gz = (gz_raw - gyro_bias_z) * DEG_TO_RAD;
    
//     // Mahony算法核心
//     float recipNorm;
//     float halfvx, halfvy, halfvz;
//     float halfex, halfey, halfez;
//     float qa, qb, qc;
    
//     // 计算误差
//     if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
//         recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
//         ax *= recipNorm;
//         ay *= recipNorm;
//         az *= recipNorm;
        
//         halfvx = q.q1 * q.q3 - q.q0 * q.q2;
//         halfvy = q.q0 * q.q1 + q.q2 * q.q3;
//         halfvz = q.q0 * q.q0 - 0.5f + q.q3 * q.q3;
        
//         halfex = (ay * halfvz - az * halfvy);
//         halfey = (az * halfvx - ax * halfvz);
//         halfez = (ax * halfvy - ay * halfvx);
        
//         // 积分误差
//         if(Ki > 0.0f) {
//             integralFBx += Ki * halfex * dt;
//             integralFBy += Ki * halfey * dt;
//             integralFBz += Ki * halfez * dt;
            
//             gx += integralFBx;
//             gy += integralFBy;
//             gz += integralFBz;
//         }
        
//         // 比例反馈
//         gx += Kp * halfex;
//         gy += Kp * halfey;
//         gz += Kp * halfez;
//     }
    
//     // 积分四元数
//     gx *= (0.5f * dt);
//     gy *= (0.5f * dt);
//     gz *= (0.5f * dt);
    
//     qa = q.q0;
//     qb = q.q1;
//     qc = q.q2;
    
//     q.q0 += (-qb * gx - qc * gy - q.q3 * gz);
//     q.q1 += (qa * gx + qc * gz - q.q3 * gy);
//     q.q2 += (qa * gy - qb * gz + q.q3 * gx);
//     q.q3 += (qa * gz + qb * gy - qc * gx);
    
//     // 归一化
//     recipNorm = 1.0f / sqrtf(q.q0 * q.q0 + q.q1 * q.q1 + 
//                             q.q2 * q.q2 + q.q3 * q.q3);
//     q.q0 *= recipNorm;
//     q.q1 *= recipNorm;
//     q.q2 *= recipNorm;
//     q.q3 *= recipNorm;
    
//     // 计算欧拉角
//     float sinr_cosp = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
//     float cosr_cosp = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);
//     eulerAngle->roll = atan2f(sinr_cosp, cosr_cosp) * 57.29578f;

//     float sinp = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);
//     if (fabsf(sinp) >= 1.0f) {
//         eulerAngle->pitch = copysignf(3.14159265358979323846f / 2.0f, sinp) * 57.29578f;
//     } else {
//         eulerAngle->pitch = asinf(sinp) * 57.29578f;
//     }

//     float siny_cosp = 2.0f * (q.q0 * q.q3 + q.q1 * q.q2);
//     float cosy_cosp = 1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3);
//     eulerAngle->yaw = atan2f(siny_cosp, cosy_cosp) * 57.29578f;
// }

 /**
 * @brief 修改后的欧拉角解算函数，适配BMI088
 */

// void IMU_GetEulerAngle(Gyro_Acc_Struct  *gyroAccel,
//                               EulerAngle_Struct *eulerAngle,
//                               float dt)
// {
//     // BMI088灵敏度定义
//     #define BMI088_GYRO_SENSITIVITY 16.384f  // LSB/(°/s)
//     #define BMI088_ACC_SENSITIVITY  5461.0f  // LSB/g
//     #define DEG_TO_RAD 0.01745329251994329576f
    
//     volatile struct V
//     {
//         float x;
//         float y;
//         float z;
//     } Gravity, Acc, Gyro, AccGravity;

//     static struct V          GyroIntegError = {0};
//     static float             KpDef          = 0.8f;
//     static float             KiDef          = 0.0003f;
//     static Quaternion_Struct NumQ           = {1, 0, 0, 0};
//     float                    q0_t, q1_t, q2_t, q3_t;
//     float NormQuat;
//     float HalfTime = dt * 0.5f;

//     // ============ 关键修改：单位转换 ============
//     // 1. 将加速度原始值转换为g单位
//     float acc_x_g = (float)gyroAccel->acc.x / BMI088_ACC_SENSITIVITY;
//     float acc_y_g = (float)gyroAccel->acc.y / BMI088_ACC_SENSITIVITY;
//     float acc_z_g = (float)gyroAccel->acc.z / BMI088_ACC_SENSITIVITY;
    
//     // 2. 将陀螺仪原始值转换为弧度/秒
//     float gyro_x_rad = (float)gyroAccel->gyro.x / BMI088_GYRO_SENSITIVITY * DEG_TO_RAD;
//     float gyro_y_rad = (float)gyroAccel->gyro.y / BMI088_GYRO_SENSITIVITY * DEG_TO_RAD;
//     float gyro_z_rad = (float)gyroAccel->gyro.z / BMI088_GYRO_SENSITIVITY * DEG_TO_RAD;
//     // ===========================================

//     // 提取等效旋转矩阵中的重力分量
//     Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
//     Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
//     Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    
//     // 加速度归一化（使用物理值g）
//     NormQuat = Q_rsqrt(acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_g * acc_z_g);

//     Acc.x = acc_x_g * NormQuat;
//     Acc.y = acc_y_g * NormQuat;
//     Acc.z = acc_z_g * NormQuat;
    
//     // 向量差乘得出的值
//     AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
//     AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
//     AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    
//     // 再做加速度积分补偿角速度的补偿值
//     GyroIntegError.x += AccGravity.x * KiDef;
//     GyroIntegError.y += AccGravity.y * KiDef;
//     GyroIntegError.z += AccGravity.z * KiDef;
    
//     // 角速度融合加速度积分补偿值（使用弧度/秒）
//     Gyro.x = gyro_x_rad + KpDef * AccGravity.x + GyroIntegError.x;
//     Gyro.y = gyro_y_rad + KpDef * AccGravity.y + GyroIntegError.y;
//     Gyro.z = gyro_z_rad + KpDef * AccGravity.z + GyroIntegError.z;

//     // 一阶龙格库塔法, 更新四元数
//     q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
//     q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
//     q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
//     q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

//     NumQ.q0 += q0_t;
//     NumQ.q1 += q1_t;
//     NumQ.q2 += q2_t;
//     NumQ.q3 += q3_t;

//     // 四元数归一化
//     NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
//     NumQ.q0 *= NormQuat;
//     NumQ.q1 *= NormQuat;
//     NumQ.q2 *= NormQuat;
//     NumQ.q3 *= NormQuat;

//     /*机体坐标系下的Z方向向量*/
//     float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;
//     float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;
//     float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;

//     // ============ 修正偏航角计算 ============
//     // 使用Mahony算法计算的偏航角，而不是单独积分
//     // 从四元数计算所有三个欧拉角
//     float sinr_cosp = 2.0f * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
//     float cosr_cosp = 1.0f - 2.0f * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
//     eulerAngle->roll = atan2f(sinr_cosp, cosr_cosp) * RtA;

//     float sinp = 2.0f * (NumQ.q0 * NumQ.q2 - NumQ.q3 * NumQ.q1);
//     if (fabsf(sinp) >= 1.0f) {
//         eulerAngle->pitch = copysignf(3.14159265358979323846f / 2.0f, sinp) * RtA;
//     } else {
//         eulerAngle->pitch = asinf(sinp) * RtA;
//     }

//     float siny_cosp = 2.0f * (NumQ.q0 * NumQ.q3 + NumQ.q1 * NumQ.q2);
//     float cosy_cosp = 1.0f - 2.0f * (NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3);
//     eulerAngle->yaw = atan2f(siny_cosp, cosy_cosp) * RtA;

//     // 计算Z轴加速度（可选）
//     static float normAccz;
//     normAccz = acc_x_g * vecxZ + acc_y_g * vecyZ + acc_z_g * veczZ;
// }

/**
 * @description: 根据mpu的6轴数据, 获取表征姿态的欧拉角
 * @param {GyroAccel_Struct} *gyroAccel mpu的6轴数据
 * @param {EulerAngle_Struct} *EulerAngle 计算后得到的欧拉角
 * @param {float} dt 采样周期 (单位s)
 * @return {*}
 */

// void IMU_GetEulerAngle(Gyro_Acc_Struct  *gyroAccel,
//                               EulerAngle_Struct *eulerAngle,
//                               float              dt)
// {
//     volatile struct V
//     {
//         float x;
//         float y;
//         float z;
//     } Gravity, Acc, Gyro, AccGravity;

//     static struct V          GyroIntegError = {0};
//     static float             KpDef          = 0.8f;
//     static float             KiDef          = 0.0003f;
//     static Quaternion_Struct NumQ           = {1, 0, 0, 0};
//     float                    q0_t, q1_t, q2_t, q3_t;
//     // float NormAcc;
//     float NormQuat;
//     float HalfTime = dt * 0.5f;

//     // 提取等效旋转矩阵中的重力分量
//     Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
//     Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
//     Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
//     // 加速度归一化
//     NormQuat = Q_rsqrt(squa(gyroAccel->acc.x) +
//                        squa(gyroAccel->acc.y) +
//                        squa(gyroAccel->acc.z));

//     Acc.x = gyroAccel->acc.x * NormQuat;
//     Acc.y = gyroAccel->acc.y * NormQuat;
//     Acc.z = gyroAccel->acc.z * NormQuat;
//     // 向量差乘得出的值
//     AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
//     AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
//     AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
//     // 再做加速度积分补偿角速度的补偿值
//     GyroIntegError.x += AccGravity.x * KiDef;
//     GyroIntegError.y += AccGravity.y * KiDef;
//     GyroIntegError.z += AccGravity.z * KiDef;
//     // 角速度融合加速度积分补偿值
//     Gyro.x = gyroAccel->gyro.x * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;   // 弧度制
//     Gyro.y = gyroAccel->gyro.y * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
//     Gyro.z = gyroAccel->gyro.z * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

//     // 一阶龙格库塔法, 更新四元数
//     q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
//     q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
//     q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
//     q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

//     NumQ.q0 += q0_t;
//     NumQ.q1 += q1_t;
//     NumQ.q2 += q2_t;
//     NumQ.q3 += q3_t;

//     // 四元数归一化
//     NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
//     NumQ.q0 *= NormQuat;
//     NumQ.q1 *= NormQuat;
//     NumQ.q2 *= NormQuat;
//     NumQ.q3 *= NormQuat;

//     /*机体坐标系下的Z方向向量*/
//     float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;     /*矩阵(3,1)项*/
//     float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;     /*矩阵(3,2)项*/
//     float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; /*矩阵(3,3)项*/

//     float yaw_G = gyroAccel->gyro.z * Gyro_G;   // 将Z轴角速度陀螺仪值 转换为Z角度/秒      Gyro_G陀螺仪初始化量程+-2000度每秒于1 / (65536 / 4000) = 0.03051756*2
//     if((yaw_G > 0.5f) || (yaw_G < -0.5))           // 数据太小可以认为是干扰，不是偏航动作
//     {
//         eulerAngle->yaw += yaw_G * dt;   // 角速度积分成偏航角
//     }

//     eulerAngle->pitch = asin(vecxZ) * RtA;   // 俯仰角

//     eulerAngle->roll = atan2f(vecyZ, veczZ) * RtA;   // 横滚角

//     normAccz = gyroAccel->acc.x * vecxZ + gyroAccel->acc.y * vecyZ + gyroAccel->acc.z * veczZ; /*Z轴垂直方向上的加速度，此值涵盖了倾斜时在Z轴角速度的向量和，不是单纯重力感应得出的值*/
// }

/**
 * @description: 获取Z轴上的加速度 (如果已经倾斜,会考虑z轴上加速度的合成)
 * @return {*}
 */
float IMU_GetNormAccZ(void)
{
    return normAccz;
}
/* ======================欧拉角计算================================== */
/* ========================结束===================================== */
