/**
  ******************************************************************************
  * @file    w25qxx.h
  * @brief   W25Qxx SPI NOR FLASH 驱动（板载，软件 SPI 模式 0）
  *
  * 硬件连接（GEC-M4 板载，课程示例 19spi 同一组引脚）：
  *   CS   -> PB14
  *   SCLK -> PB3
  *   MOSI -> PB5
  *   MISO -> PB4
  *   注意：PB3/PB4 是 JTDO/JTRST 复用脚，需调试器工作在 SWD 模式
  *        （J-Link 默认 SWD，不占这两脚，课程示例已验证可用）。
  *
  * 常用指令：0x90 读 ID / 0x05 读状态 / 0x06 写使能 /
  *          0x20 4KB 扇区擦除 / 0x02 页编程(≤256B) / 0x03 读数据
  *
  * 用途：SmartGuard 告警事件历史日志（event_log.c），MB 级容量，
  *      补足"设备端无历史数据库"的缺口。
  ******************************************************************************
  */
#ifndef __W25QXX_H__
#define __W25QXX_H__

#include "stm32f4xx.h"

void     W25QXX_Init(void);                 /* 初始化 SPI 引脚 */
uint16_t W25QXX_ReadID(void);               /* 0x90：高字节=厂商(0xEF)，低字节=容量 ID */
uint8_t  W25QXX_ReadSR(void);               /* 读状态寄存器 1（bit0=BUSY） */
uint8_t  W25QXX_ReadSR2(void);              /* 0x35：读状态寄存器 2（CMP/QE/SRP1） */
uint8_t  W25QXX_ReadSR3(void);              /* 0x15：读状态寄存器 3（bit2=WPS） */
void     W25QXX_WriteSR1(uint8_t v);        /* 0x01：写 SR1（清 BP 块保护用） */
void     W25QXX_WriteSR2(uint8_t v);        /* 0x31：写 SR2（清 CMP 互补保护用） */
void     W25QXX_WriteSR3(uint8_t v);        /* 0x11：写 SR3（清 WPS 独立块锁模式用） */
void     W25QXX_GlobalUnlock(void);         /* 0x98：全局解除独立块锁（WPS=1 时有效） */
void     W25QXX_WaitBusy(void);             /* 等待擦写完成（有界 ~3s，含喂狗） */
void     W25QXX_WriteEnable(void);          /* 写使能（每次擦/写前必须调用） */
void     W25QXX_SectorErase(uint32_t addr); /* 擦除 addr 所在 4KB 扇区 */
/* 页编程：len ≤ 256 且不得跨 256 字节页边界（调用方保证，event_log 16B 对齐天然满足） */
void     W25QXX_PageWrite(uint32_t addr, const uint8_t *buf, uint16_t len);
void     W25QXX_Read(uint32_t addr, uint8_t *buf, uint32_t len);

#endif /* __W25QXX_H__ */
