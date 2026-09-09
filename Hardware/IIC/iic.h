/**
  ******************************************************************************
  * @file    iic.h
  * @brief   软件 I2C（模拟 IIC）总线驱动头文件
  *
  * 硬件连接（STM32F407ZE 板载 MPU6050）：
  *   SCL -> PB8
  *   SDA -> PB9
  *
  * 软件 IIC 核心六个底层函数：
  *   IIC_Init        初始化 SCL/SDA 引脚，空闲拉高
  *   IIC_Start       启动信号
  *   IIC_Stop        停止信号
  *   IIC_SendByte     发送一字节（MSB 在前）
  *   IIC_ReceiveByte  接收一字节
  *   IIC_ReceiveAck / IIC_SendAck    主机收发应答
  ******************************************************************************
  */
#ifndef __IIC_H__
#define __IIC_H__

#include "stm32f4xx.h"

/* ---- IIC 引脚定义（板载 MPU6050: PB8=SCL, PB9=SDA）---- */
#define IIC_SCL_PORT       GPIOB
#define IIC_SCL_PIN        GPIO_Pin_8
#define IIC_SDA_PORT       GPIOB
#define IIC_SDA_PIN        GPIO_Pin_9
#define IIC_GPIO_CLK       RCC_AHB1Periph_GPIOB

/* ---- 电平控制宏 ---- */
#define IIC_SCL_H()        GPIO_SetBits(IIC_SCL_PORT, IIC_SCL_PIN)
#define IIC_SCL_L()        GPIO_ResetBits(IIC_SCL_PORT, IIC_SCL_PIN)
#define IIC_SDA_H()        GPIO_SetBits(IIC_SDA_PORT, IIC_SDA_PIN)
#define IIC_SDA_L()        GPIO_ResetBits(IIC_SDA_PORT, IIC_SDA_PIN)
#define IIC_SDA_READ()     GPIO_ReadInputDataBit(IIC_SDA_PORT, IIC_SDA_PIN)

/* ---- 应答值 ---- */
#define IIC_ACK            0
#define IIC_NACK           1

/* ---- 接口函数 ---- */
void     IIC_Init(void);
void     IIC_Start(void);
void     IIC_Stop(void);
void     IIC_SendByte(uint8_t data);
uint8_t  IIC_ReceiveByte(void);
uint8_t  IIC_ReceiveAck(void);
void     IIC_SendAck(uint8_t ack);

#endif /* __IIC_H__ */
