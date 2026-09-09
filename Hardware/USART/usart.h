/**
  ******************************************************************************
  * @file    usart.h
  * @brief   串口驱动头文件（USART1，中断接收 + 环形缓冲区 + printf 重定向）
  *
  * 硬件连接（STM32F407ZE）：
  *   USART1_TX -> PA9  （复用 AF7，推挽输出）
  *   USART1_RX -> PA10 （复用 AF7，浮空/上拉输入）
  *   板载 USB-TTL 芯片（CH340/CP2102 等）引出 PA9/PA10 供上位机通信
  *
  *   USART2_TX -> PA2  （复用 AF7，推挽输出）
  *   USART2_RX -> PA3  （复用 AF7，浮空/上拉输入）
  *   接蓝牙模块等外设，避免与 USB-TTL 共用 USART1 冲突
  *
  * 使用方法：
  *   USART1_Init(115200);                  // 初始化（含 NVIC 中断接收）
  *   USART1_SendString("hello\r\n");       // 发送字符串
  *   USART1_Printf("val=%d\r\n", 123);     // 格式化发送（支持 printf 格式）
  *   uint8_t ch;
  *   if (USART1_GetByte(&ch)) { ... }      // 非阻塞读取收到的字节
  ******************************************************************************
  */
#ifndef __USART_H__
#define __USART_H__

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>

/* ========== 环形缓冲区配置 ========== */
#define RX_BUF_SIZE   128     /* 接收缓冲区大小（2 的幂方便取模） */

/* ========== USART1 硬件引脚配置 ========== */
#define USART1_TX_PIN     GPIO_Pin_9
#define USART1_TX_PORT    GPIOA
#define USART1_TX_SRC     GPIO_PinSource9
#define USART1_RX_PIN     GPIO_Pin_10
#define USART1_RX_PORT    GPIOA
#define USART1_RX_SRC     GPIO_PinSource10
#define USART1_AF         GPIO_AF_USART1

/* ========== USART2 硬件引脚配置 ========== */
#define USART2_TX_PIN     GPIO_Pin_2
#define USART2_TX_PORT    GPIOA
#define USART2_TX_SRC     GPIO_PinSource2
#define USART2_RX_PIN     GPIO_Pin_3
#define USART2_RX_PORT    GPIOA
#define USART2_RX_SRC     GPIO_PinSource3
#define USART2_AF         GPIO_AF_USART2

/* ========== USART3 硬件引脚配置（接 ESP8266 WiFi 模块） ========== */
#define USART3_TX_PIN     GPIO_Pin_10
#define USART3_TX_PORT    GPIOB
#define USART3_TX_SRC     GPIO_PinSource10
#define USART3_RX_PIN     GPIO_Pin_11
#define USART3_RX_PORT    GPIOB
#define USART3_RX_SRC     GPIO_PinSource11
#define USART3_AF         GPIO_AF_USART3

/* ========== 串口速率预设 ========== */
#define BAUD_9600       9600
#define BAUD_115200     115200
#define BAUD_460800     460800

void USART1_Init(uint32_t baud);               /* 初始化 USART1（GPIO+USART+NVIC） */
void USART1_SendByte(uint8_t data);              /* 发送一个字节（阻塞等待 TXE） */
void USART1_SendString(const char *str);         /* 发送字符串（遇 '\0' 结束） */
void USART1_Printf(const char *fmt, ...);        /* 格式化发送（vsprintf） */
uint8_t USART1_GetByte(uint8_t *data);           /* 非阻塞读取一字节，返回 1=有数据 */
uint16_t USART1_GetRxCount(void);                /* 获取缓冲区中未读字节数 */
void USART1_FlushRx(void);                       /* 清空接收缓冲区 */
void USART1_RxHandler(void);                     /* 接收中断处理（由 USART1_IRQHandler 调用） */

void USART2_Init(uint32_t baud);               /* 初始化 USART2（GPIO+USART+NVIC） */
void USART2_SendByte(uint8_t data);              /* 发送一个字节（阻塞等待 TXE） */
void USART2_SendString(const char *str);         /* 发送字符串（遇 '\0' 结束） */
void USART2_Printf(const char *fmt, ...);        /* 格式化发送（vsprintf） */
uint8_t USART2_GetByte(uint8_t *data);           /* 非阻塞读取一字节，返回 1=有数据 */
uint16_t USART2_GetRxCount(void);                /* 获取缓冲区中未读字节数 */
void USART2_FlushRx(void);                       /* 清空接收缓冲区 */
void USART2_RxHandler(void);                     /* 接收中断处理（由 USART2_IRQHandler 调用） */

void USART3_Init(uint32_t baud);               /* 初始化 USART3（GPIO+USART+NVIC） */
void USART3_SendByte(uint8_t data);              /* 发送一个字节（阻塞等待 TXE） */
void USART3_SendString(const char *str);         /* 发送字符串（遇 '\0' 结束） */
uint8_t USART3_GetByte(uint8_t *data);           /* 非阻塞读取一字节，返回 1=有数据 */
uint16_t USART3_GetRxCount(void);                /* 获取缓冲区中未读字节数 */
void USART3_FlushRx(void);                       /* 清空接收缓冲区 */
void USART3_RxHandler(void);                     /* 接收中断处理（由 USART3_IRQHandler 调用） */

#endif /* __USART_H__ */
