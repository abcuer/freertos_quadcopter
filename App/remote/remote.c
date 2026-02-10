#include "remote.h"
#include "led.h"
#include "motor.h"
#include "nrf24l01.h"
#include "bsp_delay.h"
#include "stm32f1xx_hal.h"

RC_Data_Struct flight_rc_data = {0}; 

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

/**
 * @brief 飞控端遥控数据解析任务
 * @return 0: 成功解析一帧有效数据; 1: 无数据或数据非法
 */
uint8_t Remote_Parse_Task(void)
{
    uint8_t rx_buf[32]; // NRF 接收缓冲区
    RC_Frame_Struct *frame_ptr;
    uint8_t checksum = 0;

    if (NRF24L01_RxPacket(rx_buf) == 0) 
    {
        // 将接收缓冲区强制转换为结构体指针
        frame_ptr = (RC_Frame_Struct *)rx_buf;
        if (frame_ptr->header[0] == FRAME0 && 
            frame_ptr->header[1] == FRAME1 && 
            frame_ptr->header[2] == FRAME2)
        {
            // 计算校验和 (异或校验)
            checksum = Calculate_Checksum(frame_ptr);
            // 4. 比较校验和
            if (checksum == frame_ptr->checksum)
            {
                // 校验通过，提取数据到飞控逻辑结构体
                flight_rc_data.THR = frame_ptr->THR;
                flight_rc_data.YAW = frame_ptr->YAW;
                flight_rc_data.PIT = frame_ptr->PIT;
                flight_rc_data.ROL = frame_ptr->ROL;
                flight_rc_data.FIX_HEIGHT = frame_ptr->FIX_HEIGHT;
                flight_rc_data.LOCK_KEY = frame_ptr->LOCK_KEY;

                flight_rc_data.CONNECT = 1; // 标记连接正常
                flight_rc_data.NRF_ERR = 0; // 清除错误计数
                
                // if(flight_rc_data.CONNECT)

                return 0; // 成功解析
            }
        }
    }
    // 如果走到这里，说明本周期没有收到有效包
    return 1; 
}

// /**
//  * @brief 确认是否连接成功
//  */
// static uint8_t Nrf_Connect(void)
// {
//     static uint32_t last_receive_time = 0;
//     static uint8_t success_count = 0;
    
//     // 检查是否有新的有效数据
//     if (rc_data.CONNECT == 1)
//     {
//         // 数据有效，更新接收时间戳
//         last_receive_time = GetSysTime_us();
//         success_count++;

//         return 1; // 连接成功
//     }
//     else
//     {
//         // 接收超时判断（2秒无数据认为断开）
//         uint32_t current_time = GetSysTime_us();
//         if (current_time - last_receive_time > 1000)
//         {
//             rc_data.CONNECT = 0;
//             success_count = 0;
            
//             // 连接失败状态指示（红灯闪烁）
//             static uint32_t blink_time = 0;
//             if (current_time - blink_time > 200)
//             {
//                 SetLedMode(rLEDL, LED_TOGGLE);
//                 blink_time = current_time;
//             }
//             SetLedMode(rLEDL, LED_OFF);
            
//             return 0; // 连接失败
//         }
        
//         // 未超时但未连接，保持原状态
//         return rc_data.CONNECT;
//     }
// }

// /**
//  * @brief 
//  */
// static void NRF_Check_Event(void)
// {
//     uint8_t sta = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);
    
//     if(sta & (1<<6)) // RX_DR: 接收到数据
//     {
//         uint8_t rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);
//         if(rx_len == sizeof(RC_Frame_Struct)) // 检查数据长度
//         {
//             // 读取数据
//             NRF24L01_Read_Buf(RD_RX_PLOAD, NRF24L01_RXDATA, rx_len);
            
//             // 解析数据，自动更新rc_data.CONNECT状态
//             Parse_Remote_Data(NRF24L01_RXDATA, rx_len);
//         }
//         else 
//         {
//             NRF24L01_Write_Reg(FLUSH_RX, 0xff); // 清空缓存
//         }
//     }
//     // 清除状态标志
//     NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, sta);
// }

// void Remote_Receive_Task()
// {
//     // 数据解析
//     NRF_Check_Event();
//     if(Nrf_Connect())
//     {   
//         // 连接成功状态指示（蓝灯常亮）
//         SetLedMode(bLEDL, LED_TOGGLE);
//         SetLedMode(bLEDR, LED_TOGGLE);
//         SetLedMode(rLEDL, LED_OFF);
//         SetLedMode(rLEDR, LED_OFF);
//         HAL_Delay(300);
//     }
//     else 
//     {
//         SetLedMode(bLEDL, LED_OFF);
//         SetLedMode(bLEDR, LED_OFF);
//         SetLedMode(rLEDL, LED_TOGGLE);
//         SetLedMode(rLEDR, LED_TOGGLE);
//         HAL_Delay(300);
//     }

// }