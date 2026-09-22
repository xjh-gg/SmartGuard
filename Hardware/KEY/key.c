#include "key.h"

void KEY_Init(void)
{
	// 配置GPIO的引脚
	GPIO_InitTypeDef myGPIO; // GPIO标准结构体定义   定义变量放在全局或函数的开头部分
	
	// 开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);
	
	/* 实测结论（KEYDBG 原始电平打印验证）：本板四个按键全部为低电平有效，
	   空闲 PA0/PE2/PE3/PE4 均读 1，按下读 0。
	   注意 PA0 虽名为 WK_UP，但 GEC-M4 这块板上实际带外部上拉（曾配内部
	   下拉，空闲仍读 1，证明外部上拉占优），按下接 GND——与老师模板注释的
	   "常见板 WK_UP 按下为高"不符，以实测为准。
	   全部配内部上拉加固空闲态，防浮空伪边沿。 */
	myGPIO.GPIO_Pin = GPIO_Pin_0;
	myGPIO.GPIO_Mode = GPIO_Mode_IN;
	myGPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &myGPIO);

	myGPIO.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_Init(GPIOE, &myGPIO);
}

uint8_t KEY_Scan(void)
{
	if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))   /* S1=PA0 实测低电平有效 */
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


