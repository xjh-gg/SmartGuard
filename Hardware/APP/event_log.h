/**
  ******************************************************************************
  * @file    event_log.h
  * @brief   告警事件历史日志（W25Qxx SPI FLASH 环形存储）
  *
  * 设计（环形日志 / 黑匣子）：
  *   - 区域：FLASH 起始 EVENT_LOG_SECTOR_COUNT 个 4KB 扇区（默认 4 个 = 16KB）
  *   - 记录：16 字节定长，每扇区 256 条，总容量 1024 条
  *   - 追加写：NOR FLASH 擦除态(0xFF)可直接编程，无需每条都擦
  *   - 回卷：写满一个扇区进下一个；回到含旧数据的扇区时先整扇区擦除
  *          （4KB 擦除典型 45ms，远好于内部 FLASH 128KB 的 ~1s 停顿）
  *   - 掉电恢复：上电扫描各扇区，定位"首个擦除态记录槽"恢复写指针
  *
  * 记录布局（16B）：
  *   [0]    0xEB 记录魔数（区分擦除态 0xFF 与撕裂写）
  *   [1]    告警类型 Alert_Type_t
  *   [2..7] RTC：年(0-99) 月 日 时 分 秒
  *   [8..9] 光照 ADC（大端）
  *   [10..11] MQ2 烟雾 ADC（大端）
  *   [12..13] 距离 cm（大端）
  *   [14]   布防模式
  *   [15]   校验和 = bytes[0..14] 之和 & 0xFF
  ******************************************************************************
  */
#ifndef __EVENT_LOG_H
#define __EVENT_LOG_H

#include "stm32f4xx.h"
#include "sensor_hub.h"

/* 初始化：探测 W25Qxx + 扫描恢复写指针。1=可用，0=芯片不在（后续 API 全部空转） */
uint8_t  EventLog_Init(void);
/* 追加一条告警事件（type 取 Alert_Type_t；芯片不在时静默忽略） */
void     EventLog_Add(uint8_t alert_type, const Sensor_Data_t *s, uint8_t mode);
/* 当前存储的事件条数 */
uint16_t EventLog_Count(void);
/* 串口回显最近 last_n 条（新的在前） */
void     EventLog_Dump(uint16_t last_n);
/* 清空全部事件 */
void     EventLog_Clear(void);

#endif /* __EVENT_LOG_H */
