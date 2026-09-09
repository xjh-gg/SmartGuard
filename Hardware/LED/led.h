/**
  ******************************************************************************
  * @file    led.h
  * @brief   LED 驱动头文件（第07课 启动文件分析 工程共用）
  *   LED0 -> PF9   LED1 -> PF10   LED2 -> PE13   LED3 -> PE14（低电平点亮）
  ******************************************************************************
  */
#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

#define LED0_PIN  GPIO_Pin_9
#define LED0_PORT GPIOF
#define LED1_PIN  GPIO_Pin_10
#define LED1_PORT GPIOF
#define LED2_PIN  GPIO_Pin_13
#define LED2_PORT GPIOE
#define LED3_PIN  GPIO_Pin_14
#define LED3_PORT GPIOE

void LED_Init(void);
void LED_ON(uint8_t x);
void LED_OFF(uint8_t x);
void LED_Toggle(uint8_t x);
void LED_AllOn(void);
void LED_AllOff(void);

#endif /* __LED_H__ */
