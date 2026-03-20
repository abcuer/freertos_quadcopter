#ifndef __POWER_H
#define __POWER_H
#include "stdint.h"

#define POWER0 3700
#define POWER1 3750

void Power_Init(void);
int16_t Voltage_Check(void);

#endif