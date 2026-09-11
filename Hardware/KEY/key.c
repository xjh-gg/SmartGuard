#include "key.h"

void KEY_Init(void)
{
	// 配置GPIO的引脚
	GPIO_InitTypeDef myGPIO; // GPIO标准结构体定义   定义变量放在全局或函数的开头部分
	
	// 开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);
	
	myGPIO.GPIO_Pin = GPIO_Pin_0;
	myGPIO.GPIO_Mode = GPIO_Mode_IN;
	/* 四个按键（PA0/PE2/PE3/PE4）都是低电平有效，必须有个上拉把空闲态钉在 3.3V。
	   原来配 NOPULL 完全依赖板外部上拉，缺一个或阻值偏高就会浮空，被主循环 100ms
	   采样读成随机低电平 → 制造伪边沿 → 一次按压多次触发。内部上拉约 40kΩ，
	   与外部上拉并联无害。 */
	myGPIO.GPIO_PuPd = GPIO_PuPd_UP;
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


