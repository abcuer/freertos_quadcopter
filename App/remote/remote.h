#ifndef __REMOTE_H
#define __REMOTE_H
#include "stdint.h"

// 校验帧
#define FRAME0 'M'  // 帧头1
#define FRAME1 'G'  // 帧头2
#define FRAME2 'S'  // 帧尾

// 遥控数据结构
typedef struct {
    uint16_t THR;        // 油门 (1000-2000)
    uint16_t YAW;        // 偏航 (1000-2000)
    uint16_t PIT;        // 俯仰 (1000-2000)
    uint16_t ROL;        // 横滚 (1000-2000)
    uint8_t LOCK_KEY;    // 锁定标志 (0/1)
    uint8_t FIX_HEIGHT;  // 定高标志 (0/1)
    uint8_t CONNECT;     // 连接成功标志位
    uint8_t NRF_ERR;     // 错误计数
} Remote_Data_Struct;

// 数据帧结构（用于通信）
typedef struct __attribute__((packed)){ // 强制编译器取消对齐补齐
    uint8_t header[3];   // 帧头: 'M', 'G', 'S'
    uint16_t THR;        // 油门
    uint16_t YAW;        // 偏航
    uint16_t PIT;        // 俯仰
    uint16_t ROL;        // 横滚
    uint8_t FIX_HEIGHT;  // 定高标志
    uint8_t LOCK_KEY;    // 锁定标志
    uint8_t checksum;    // 校验和
} RC_Frame_Struct;

void Remote_ReceiveData(void);
extern Remote_Data_Struct flight_rc_data; 

#endif