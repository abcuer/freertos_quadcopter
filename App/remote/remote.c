#include "remote.h"
#include "nrf24l01.h"

Remote_Data_Struct flight_rc_data = {0}; 
// 默认 姿态模式
// 否则 定高模式
/**
 * @brief 计算数据帧的校验和
 * @param frame 数据帧指针
 * @return 计算得到的校验和
 */
static uint8_t Calculate_Checksum(RC_Frame_Struct *frame)
{
    uint8_t checksum = 0;
    uint8_t *data = (uint8_t *)frame;
    // 遍历长度为 14 - 1 = 13
    for (int i = 0; i < sizeof(RC_Frame_Struct) - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

static uint8_t Remote_ParseData(uint8_t *rx_buf, Remote_Data_Struct *rc_data)
{
    RC_Frame_Struct *frame_ptr = (RC_Frame_Struct *)rx_buf;

    if (frame_ptr->header[0] != FRAME0 ||
        frame_ptr->header[1] != FRAME1 ||
        frame_ptr->header[2] != FRAME2)
        return 0;

    if (Calculate_Checksum(frame_ptr) != frame_ptr->checksum)
        return 0;

    rc_data->THR = frame_ptr->THR;
    rc_data->YAW = frame_ptr->YAW;
    rc_data->PIT = frame_ptr->PIT;
    rc_data->ROL = frame_ptr->ROL;
    rc_data->FIX_HEIGHT = frame_ptr->FIX_HEIGHT;
    rc_data->LOCK_KEY = frame_ptr->LOCK_KEY;
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
            NRF24L01_RX_Mode();
            flight_rc_data.NRF_ERR = 0;
        }
    }
}