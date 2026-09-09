/**
  ******************************************************************************
  * @file    dht11.h
  * @brief   DHT11 温湿度传感器驱动头文件
  *
  * 硬件连接（STM32F407ZE）：
  *   DHT11_DATA -> PG9
  *   VCC        -> 3.3V
  *   GND        -> GND
  *
  * 数据格式：
  *   一次完整读取返回 5 字节：
  *     data[0] = 湿度整数部分
  *     data[1] = 湿度小数部分（DHT11 通常为 0）
  *     data[2] = 温度整数部分
  *     data[3] = 温度小数部分（DHT11 通常为 0）
  *     data[4] = 校验和 = data[0] + data[1] + data[2] + data[3]
  ******************************************************************************
  */
#ifndef __DHT11_H__
#define __DHT11_H__

#include "stm32f4xx.h"

/* DHT11 单总线引脚定义 */
#define DHT11_GPIO_PORT     GPIOG
#define DHT11_GPIO_PIN      GPIO_Pin_9
#define DHT11_GPIO_CLK      RCC_AHB1Periph_GPIOG

void DHT11_Init(void);
int32_t DHT11_Read(uint8_t *data);
uint8_t DHT11_Read_Data(float *temperature, float *humidity);

#endif /* __DHT11_H__ */
