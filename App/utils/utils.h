#ifndef __UTILS_H
#define __UTILS_H

#include "stdint.h"

void System_Init(void);
void LedScan(void);
uint8_t NRF_RX_Wait_Connect(void);

#endif
