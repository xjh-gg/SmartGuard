#ifndef __MQ2_H
#define __MQ2_H

#include "stm32f4xx.h"

void MQ2_Init(void);
uint16_t MQ2_Read(void);
uint16_t MQ2_Get_Baseline(void);

#endif
