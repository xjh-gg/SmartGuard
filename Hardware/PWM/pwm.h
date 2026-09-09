/**
  ******************************************************************************
  * @file    pwm.h
  * @brief   TIM 双通道 PWM 驱动头文件
  *
  * 硬件连接（STM32F407ZE）：
  *   手机模式灯 -> LED1(PF9)  / TIM14_CH1
  *   家居模式灯 -> LED4(PE14) / TIM1_CH4
  *
  * 默认 PWM 频率：1 kHz（周期 1000 个计数）
  ******************************************************************************
  */
#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f4xx.h"

/* 手机模式 PWM：TIM14_CH1 -> PF9 */
#define PWM_PHONE_TIM           TIM14
#define PWM_PHONE_TIM_CLK       RCC_APB1Periph_TIM14
#define PWM_PHONE_GPIO_PORT     GPIOF
#define PWM_PHONE_GPIO_PIN      GPIO_Pin_9
#define PWM_PHONE_GPIO_CLK      RCC_AHB1Periph_GPIOF
#define PWM_PHONE_GPIO_AF       GPIO_AF_TIM14
#define PWM_PHONE_GPIO_SRC      GPIO_PinSource9

/* 家居模式 PWM：TIM1_CH4 -> PE14 */
#define PWM_HOME_TIM            TIM1
#define PWM_HOME_TIM_CLK        RCC_APB2Periph_TIM1
#define PWM_HOME_GPIO_PORT      GPIOE
#define PWM_HOME_GPIO_PIN       GPIO_Pin_14
#define PWM_HOME_GPIO_CLK       RCC_AHB1Periph_GPIOE
#define PWM_HOME_GPIO_AF        GPIO_AF_TIM1
#define PWM_HOME_GPIO_SRC       GPIO_PinSource14

#define PWM_PERIOD              1000

void PWM_Init(uint16_t period);
void PWM_Phone_SetDuty(uint16_t duty);
void PWM_Home_SetDuty(uint16_t duty);
void PWM_Phone_Off(void);
void PWM_Home_Off(void);

#endif /* __PWM_H__ */
