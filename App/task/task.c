#include "control.h"
#include "headfile.h"

// 任务执行周期(ms)
#define IMU_PERIOD 6
#define CTRL_PERIOD 6
#define COMM_PERIOD 6
#define OTHER_PERIOD 50

void StartIMUTask(void const * argument)
{
    for(;;)
    {   
        IMU_Get_GyroAcc(&gyro_acc);
        IMU_Get_EulerAngle(&gyro_acc, &euler_angle, IMU_PERIOD / 1000.0f);
        osDelay(IMU_PERIOD);
    }
}


void StartControlTask(void const * argument)
{
    for(;;)
    {   
        FlyControl();
        osDelay(CTRL_PERIOD);
    }
}

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
