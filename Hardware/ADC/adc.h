/**
  ******************************************************************************
  * @file    adc.h
  * @brief   ADC 多通道采样驱动头文件
  *
  * 硬件连接（STM32F407ZE）：
  *   电位器      -> PA5 / ADC1_IN5
  *   光敏电阻    -> PF7 / ADC3_IN5
  *
  * 说明：
  *   ADC1 与 ADC3 共享 ADC_CommonInit 配置，因此统一初始化。
  ******************************************************************************
  */
#ifndef __ADC_H__
#define __ADC_H__

#include "stm32f4xx.h"

/* 电位器：ADC1 通道 5 -> PA5 */
#define POT_ADC                 ADC1
#define POT_ADC_CLK             RCC_APB2Periph_ADC1
#define POT_ADC_CHANNEL         ADC_Channel_5
#define POT_GPIO_PORT           GPIOA
#define POT_GPIO_PIN            GPIO_Pin_5
#define POT_GPIO_CLK            RCC_AHB1Periph_GPIOA

/* 光敏电阻：ADC3 通道 5 -> PF7 */
#define LIGHT_ADC               ADC3
#define LIGHT_ADC_CLK           RCC_APB2Periph_ADC3
#define LIGHT_ADC_CHANNEL       ADC_Channel_5
#define LIGHT_GPIO_PORT         GPIOF
#define LIGHT_GPIO_PIN          GPIO_Pin_7
#define LIGHT_GPIO_CLK          RCC_AHB1Periph_GPIOF

/* MQ-2 烟雾传感器：ADC1 通道 4 -> PA4 */
#define MQ2_ADC                 ADC1
#define MQ2_ADC_CLK             RCC_APB2Periph_ADC1
#define MQ2_ADC_CHANNEL         ADC_Channel_4
#define MQ2_GPIO_PORT           GPIOA
#define MQ2_GPIO_PIN            GPIO_Pin_4
#define MQ2_GPIO_CLK            RCC_AHB1Periph_GPIOA

/* MQ-2 硬件开关：0=未接线（ADC_ReadMQ2 恒返回 0，火灾分支自动不触发）
   1=已接 MQ-2（PA4 模拟输入，5V 供电；上电需预热数分钟再做基线学习） */
#ifndef USE_MQ2
#define USE_MQ2                 1
#endif

void ADC_Hardware_Init(void);
uint16_t ADC_ReadPotentiometer(void);
uint16_t ADC_ReadPhotoresistor(void);
uint16_t ADC_ReadMQ2(void);

#endif /* __ADC_H__ */
