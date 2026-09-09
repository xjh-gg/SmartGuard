/**
  ******************************************************************************
  * @file    config_store.h
  * @brief   SmartGuard 配置掉电存储（STM32F407 内部 FLASH）
  *
  * 存储内容：基线 + 模式 + CRC32 + Magic Number
  * 存储位置：STM32F407ZE Sector 7（0x080E0000，最后一个扇区，128KB）
  * CRC 校验：STM32F4 硬件 CRC 单元（多项式 0x04C11DB7）
  *
  * 典型流程：
  *   上电 → Config_Load() → CRC 有效？ → 恢复基线/模式，跳过 20s 学习
  *                          → CRC 无效？→ 正常走基线学习
  *   学习完/模式切换 → Config_Save() → 擦除 Sector 7 → 写入新数据
  *
  * 注意：STM32F4 内部 FLASH 擦写寿命 ≈ 10000 次，
  *       基线只在学习完存一次、模式切换也不频繁，寿命没问题。
  ******************************************************************************
  */
#ifndef __CONFIG_STORE_H
#define __CONFIG_STORE_H

#include "stm32f4xx.h"
#include "decision_engine.h"   /* Guard_Mode_t */
#include "adaptive_threshold.h" /* AT_Baseline_t */

/* FLASH 存储地址：Sector 7，最后 128KB */
#define CONFIG_FLASH_SECTOR     FLASH_Sector_7
#define CONFIG_FLASH_ADDR       0x080E0000UL
#define CONFIG_FLASH_SECTOR_SIZE (128 * 1024)

/* 魔数："SGDG" = SmartGuard */
#define CONFIG_MAGIC            0x53474447UL

/* ==== 配置结构体（56 字节，擦除时整页擦 + 写入） ==== */
typedef struct {
    uint32_t        magic;              /* 魔数，验证是我们自己存的 */
    uint16_t        light_baseline;     /* 光敏基线 */
    uint16_t        light_sigma;        /* 光敏标准差 */
    float           temp_baseline;      /* 温度基线 (°C) */
    float           humi_baseline;      /* 湿度基线 (%) */
    Guard_Mode_t    mode;               /* 当前模式 */
    uint8_t         is_learned;         /* 1=基线已学习，0=未学习 */
    uint8_t         reserved;           /* 对齐 */
    uint16_t        mq2_sigma;          /* 烟雾标准差 */
    float           light_ewma;         /* EWMA 动态光敏基线 */
    float           temp_ewma;          /* EWMA 动态温度基线 */
    float           humi_ewma;          /* EWMA 动态湿度基线 */
    float           temp_sigma;         /* 温度标准差 */
    float           humi_sigma;         /* 湿度标准差 */
    float           mq2_ewma;           /* EWMA 动态烟雾基线 */
    uint16_t        mq2_fire_threshold; /* 烟雾告警动态阈值 */
    uint16_t        reserved2;          /* 对齐到 4 字节 */
    uint32_t        crc32;              /* CRC32（校验范围 = magic 到 reserved2，不含 crc32 自己） */
} ConfigStore_t;

/* 对外接口 */
uint8_t  Config_Load(AT_Baseline_t *bl, Guard_Mode_t *mode);
uint8_t  Config_Save(const AT_Baseline_t *bl, Guard_Mode_t mode);

#endif /* __CONFIG_STORE_H */
