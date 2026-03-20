#include "remote.h"
#include "nrf24l01.h"
#include "power.h"
#include "control.h"

RX_Data_Struct flight_rc_data = {0}; 
TX_Data_Struct flight_status_data = {0}; // 飞控状态数据
TX_Frame_Struct status_frame;            // 回传数据帧

/**
 * @brief 计算数据帧的校验和
 * @param frame 数据帧指针
 * @return 计算得到的校验和
 */
static uint8_t Calculate_Checksum(RX_Frame_Struct *frame)
{
    uint8_t checksum = 0;
    uint8_t *data = (uint8_t *)frame;
    // 遍历长度为 14 - 1 = 13
    for (int i = 0; i < sizeof(RX_Frame_Struct) - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief 计算回传帧的校验和
 */
static uint8_t Calculate_Status_Checksum(TX_Frame_Struct *frame)
{
    uint8_t checksum = 0;
    uint8_t *data = (uint8_t *)frame;
    for (int i = 0; i < sizeof(TX_Frame_Struct) - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief 打包飞控状态数据
 */
void Remote_PackStatusData(void)
{
    status_frame.header[0] = FRAME0;
    status_frame.header[1] = FRAME1;
    status_frame.header[2] = FRAME2;
    
    // 填充实时姿态与PID (这些变量由飞控算法更新)
    status_frame.pitch = euler_angle.pitch;
    status_frame.roll  = euler_angle.roll;
    status_frame.yaw   = euler_angle.yaw;
    status_frame.pid_kp = pid_gyro_y.kp;
    status_frame.pid_ki = pid_gyro_y.ki;
    status_frame.pid_kd = pid_gyro_y.kd;
    status_frame.voltage = Voltage_Check(); // 转为mV
    
    status_frame.checksum = Calculate_Status_Checksum(&status_frame);
}

static uint8_t Remote_ParseData(uint8_t *rx_buf, RX_Data_Struct *rx_data)
{
    RX_Frame_Struct *frame_ptr = (RX_Frame_Struct *)rx_buf;

    if (frame_ptr->header[0] != FRAME0 ||
        frame_ptr->header[1] != FRAME1 ||
        frame_ptr->header[2] != FRAME2)
        return 0;

    if (Calculate_Checksum(frame_ptr) != frame_ptr->checksum)
        return 0;

    rx_data->THR = frame_ptr->THR;
    rx_data->YAW = frame_ptr->YAW;
    rx_data->PIT = frame_ptr->PIT;
    rx_data->ROL = frame_ptr->ROL;
    rx_data->FIX_HEIGHT = frame_ptr->FIX_HEIGHT;
    rx_data->LOCK_KEY = frame_ptr->LOCK_KEY;
    return 1;
}


/**
 * @brief 飞控端遥控数据解析任务
 * @return 
 */
uint8_t rx_buf[32]; // NRF 接收缓冲区
static uint8_t success_cnt = 0;
static uint8_t fail_cnt = 0;
void Remote_ReceiveData(void)
{
    if (NRF24L01_RxPacket(rx_buf) == 0) 
    {
        // 解析数据
        if(Remote_ParseData(rx_buf, &flight_rc_data))
        {
            if(success_cnt < 2) success_cnt++;
            fail_cnt = 0;
            // --- 新增：准备并装载下一次回传的数据 ---
            // 注意：ACK Payload 是“预装载”机制，当前装载的数据会在遥控器下一次发包时带回   
            Voltage_Check();
            Remote_PackStatusData();
            NRF_TxPacket_AP((uint8_t*)&status_frame, sizeof(TX_Frame_Struct));
        }
    }  
    else 
    {
        if(fail_cnt < 10) fail_cnt++;
        success_cnt = 0;
    }
    // 判断是否连接
    if(success_cnt >= 2 && flight_rc_data.CONNECT == 0)
    {
        flight_rc_data.CONNECT = 1;
        flight_rc_data.NRF_ERR = 0;
    }
    else if(fail_cnt >= 10 && flight_rc_data.CONNECT == 1)
    {
        flight_rc_data.CONNECT = 0;
        // 安全保护
        flight_rc_data.THR = 1000;   // 最小油门
        flight_rc_data.YAW = 1500;
        flight_rc_data.PIT = 1500;
        flight_rc_data.ROL = 1500;

        if(flight_rc_data.NRF_ERR < 50)
            flight_rc_data.NRF_ERR++;
        else
        {
            // 重新连接
            NRF_Init(MODEL_RX2, CONNECT_CHANNAL);
            flight_rc_data.NRF_ERR = 0;
        }
    }
}