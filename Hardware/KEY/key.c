#include "key.h"

void KEY_Init(void)
{
	// 配置GPIO的引脚
	GPIO_InitTypeDef myGPIO; // GPIO标准结构体定义   定义变量放在全局或函数的开头部分
	
	// 开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);
	
	myGPIO.GPIO_Pin = GPIO_Pin_0;
	myGPIO.GPIO_Mode = GPIO_Mode_IN;
	myGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &myGPIO);
	
	myGPIO.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_Init(GPIOE, &myGPIO);
}

uint8_t KEY_Scan(void)
{
	if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		return S1;
	else if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))
		return S2;
	else if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
		return S3;
	else if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
		return S4;
	else
		return !S1;
}


