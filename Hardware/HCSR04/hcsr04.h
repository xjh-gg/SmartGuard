/**
  ******************************************************************************
  * @file    hcsr04.h
  * @brief   HC-SR04 超声波测距驱动头文件
  *
  * 实现方式：
  *   - Echo 引脚 PC6 → TIM8_CH1 输入捕获（1MHz 计数器，1 tick = 1us）
  *   - Trig 引脚 PC8 → GPIO 推挽 + delay_us 产生 15us 触发脉冲
  *   - 利用 TIM8_CCR1(上升沿) 和 TIM8_CCR2(下降沿) 差值计算 Echo 高电平时间
  *   - 超时保护：30ms（约 5m 量程）
  *
  * 硬件连接（STM32F407ZE）：
  *   HC-SR04 VCC -> 5V
  *   HC-SR04 GND -> GND
  *   HC-SR04 Trig -> PC8  (GPIO 推挽输出)
  *   HC-SR04 Echo -> PC6  (TIM8_CH1 AF 模式)
  *
  * 测距原理：
  *   d(cm) = t(us) / 58
  *   d(mm) = t(us) * 17 / 100
  ******************************************************************************
  */
#ifndef __HCSR04_H__
#define __HCSR04_H__

#include "stm32f4xx.h"

/* ========== 硬件引脚配置 ========== */
#define HCSR04_TRIG_PORT       GPIOC
#define HCSR04_TRIG_PIN        GPIO_Pin_8
#define HCSR04_ECHO_PORT       GPIOC
#define HCSR04_ECHO_PIN        GPIO_Pin_6

/* ========== 测距参数 ========== */
#define HCSR04_TRIG_US         15      /* Trig 触发脉冲宽度（us） */
#define HCSR04_TIMEOUT_US      30000   /* Echo 超时（30ms，对应约 5m） */

/* 错误码：g_hcsr04_err 上一次测距失败原因 */
extern volatile uint8_t g_hcsr04_err;
#define HCSR04_ERR_OK          0   /* 测距成功 */
#define HCSR04_ERR_NO_RISING   1   /* Echo 上升沿未出现（模块未响应） */
#define HCSR04_ERR_NO_FALLING  2   /* Echo 上升沿后出现但未下降（超时） */

void     HCSR04_Init(void);                        /* 初始化 GPIO + TIM8 输入捕获 */
float    HCSR04_GetDistance(void);                 /* 获取距离（厘米），超时返回 0 */
uint32_t HCSR04_GetDistanceMM(void);               /* 获取距离（毫米），超时返回 0 */
void     HCSR04_GetDistanceBoth(uint32_t *cm, uint32_t *mm); /* 一次触发同时返回 cm/mm */

#endif /* __HCSR04_H__ */
