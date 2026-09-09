#ifndef __KEY_H__
#define __KEY_H__
#include "stm32f4xx.h"

enum{
	S1=1,
	S2,
	S3,
	S4
};

void KEY_Init(void);
uint8_t KEY_Scan(void);

#endif
