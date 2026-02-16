#include "control.h"
#include "headfile.h"
#include "remote.h"

/* 周期：控制4，通讯6 灯控50 */
/* 优先级：控制8，通讯8 灯控2*/

/* 周期：电源10000 控制6，通讯10 灯控100 */
/* 优先级：控制4，通讯3 灯控1 电源4 */

// 任务执行周期(ms)
#define CTRL_PERIOD 4
#define COMM_PERIOD 6
#define OTHER_PERIOD 50

void StartControlTask(void const * argument)
{
    for(;;)
    {   
            // 获取加速度和角速度
        IMU_Get_GyroAcc(&gyro_acc);
        // 获取欧拉角
        IMU_Get_EulerAngle(&gyro_acc, &euler_angle, CTRL_PERIOD/1000.0f);
        // 计算姿态环
        Flight_Calculate_PID(&gyro_acc, &euler_angle, &flight_rc_data, CTRL_PERIOD/1000.0f);
        // 电机控制
        FlyControl();
        osDelay(CTRL_PERIOD);
    }
}

// void StartIMUTask(void const * argument)
// {
//     for(;;)
//     {   


//         osDelay(IMU_PERIOD);
//     }
// }

void StartCommTask(void const * argument)
{
    for(;;)
    {
        Remote_ReceiveData();
        osDelay(COMM_PERIOD);
    }
}

void StartOtherTask(void const * argument)
{
    for(;;)
    {
        LedScan();
        osDelay(OTHER_PERIOD); 
    }
}
