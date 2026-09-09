/**
  ******************************************************************************
  * @file    oled.h
  * @brief   0.96寸 OLED SSD1306 SPI 驱动（软件模拟 SPI）
  *
  * 接线表（TFTLCD 触摸 SPI 区，来自 GEC-M4 原理图）：
  *   OLED 引脚  ->  TFTLCD 排针丝印 -> STM32F407ZE 引脚
  *   ---------      --------------     -----------------
  *   GND        ->  GND               -> GND
  *   VCC        ->  5V                -> 5V
  *   CLK        ->  T_SCK             -> PB0   (GPIO, 非 SPI 复用)
  *   MOSI       ->  T_MOSI            -> PF11  (GPIO, 非 SPI 复用)
  *   DC         ->  T_PEN             -> PB1   (触摸中断, 空闲 GPIO)
  *   CS1        ->  T_CS              -> PC13  (RTC, 需降速 2MHz)
  *   CS2        ->  VCC (3.3V)        -> 🚨 务必接 VCC 禁用 GT20L1616 字库芯片！
  *   FS0        ->  悬空              -> OLED 自己选 SPI 模式
  *
  * 致命坑 1：OLED 模块上 GT20L1616 字库芯片共享 SPI 总线！
 *   CS2 悬空时字库芯片会抢 SPI 导致 SSD1306 收不到指令 = 黑屏！
 *   把 CS2 接 VCC(3.3V/5V) 即可彻底禁用字库芯片。
 * 致命坑 2：PC13 同时是 RTC Tamper 引脚，必须在 rtc.c 中禁用 Tamper，
 *   否则 RTC 初始化后会静默复用 PC13，OLED_CS 失效。
 * 致命坑 3：STM32F407 主频 168MHz，软件 SPI 若用 __nop() 延时会太快，
 *   SSD1306 采样不到数据 = 黑屏。必须用 delay_us(1) 放慢时序。
 ******************************************************************************/
#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ============ SPI 引脚定义 (TFTLCD 触摸 SPI) ============ */
#define OLED_CS_Clr()    GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define OLED_CS_Set()    GPIO_SetBits(GPIOC,   GPIO_Pin_13)

#define OLED_DC_Clr()    GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define OLED_DC_Set()    GPIO_SetBits(GPIOB,   GPIO_Pin_1)

#define OLED_SCLK_Clr()  GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define OLED_SCLK_Set()  GPIO_SetBits(GPIOB,   GPIO_Pin_0)

#define OLED_SDIN_Clr()  GPIO_ResetBits(GPIOF, GPIO_Pin_11)
#define OLED_SDIN_Set()  GPIO_SetBits(GPIOF,   GPIO_Pin_11)

#define OLED_CMD  0
#define OLED_DATA 1

/* ============ 分辨率 ============ */
#define X_WIDTH   128
#define Y_WIDTH   64
#define Max_Column 128
#define Max_Row    64
#define SIZE       16

/* ============ SmartGuard 接口 ============ */
#include "sensor_hub.h"
#include "decision_engine.h"

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearArea(uint8_t x, uint8_t page, uint8_t width);
void OLED_DrawHLine(uint8_t x, uint8_t page, uint8_t len);
void OLED_DrawVLine(uint8_t x, uint8_t page_start, uint8_t page_end);
void OLED_ShowChar_raw(uint8_t x, uint8_t y, uint8_t chr);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowFloat(uint8_t x, uint8_t y, float val, uint8_t int_len, uint8_t dec_len);
void OLED_ShowIcon8x8(uint8_t x, uint8_t page, const uint8_t *icon);
void OLED_ShowHomeScreen(Sensor_Data_t *data, Guard_Mode_t mode, Alert_Type_t alert);

#endif
