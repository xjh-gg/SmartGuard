/**
  ******************************************************************************
  * @file    config_store.h
  * @brief   SmartGuard 配置掉电存储
  *
  * 存储后端（CONFIG_USE_EEPROM 一键切换）：
  *   1 = AT24C02 EEPROM（板载，软件 IIC PB8/PB9，地址 0x00 起 56 字节）
  *       按字节擦写，无扇区擦除 CPU 停顿，寿命 ~100 万次 —— 默认方案
  *   0 = 内部 FLASH Sector 11（0x080E0000，128KB）
  *       历史方案，留作 EEPROM 未焊接时的回退
  *
  * 数据布局（两种后端相同）：ConfigStore_t 56 字节 = magic + 基线 + 模式 + CRC32
  *
  * 典型流程：
  *   上电 → Config_Load() → magic+CRC 有效？ → 恢复基线/模式，跳过 20s 学习
  *                          → 无效？→ 正常走基线学习
  *   学习完/模式切换 → Config_Save()
  ******************************************************************************
  */
#ifndef __CONFIG_STORE_H
#define __CONFIG_STORE_H

#include "stm32f4xx.h"
#include "decision_engine.h"   /* Guard_Mode_t */
#include "adaptive_threshold.h" /* AT_Baseline_t */

/* 存储后端选择：1=AT24C02 EEPROM（默认）  0=内部 FLASH Sector 11 */
#define CONFIG_USE_EEPROM       1

/* ---- 内部 FLASH 后端参数（CONFIG_USE_EEPROM=0 时使用）----
   注意：SECTOR 与 ADDR 必须指向同一扇区！历史上曾写成
   FLASH_Sector_7（0x08060000）配 0x080E0000（Sector 11），
   导致每次保存"擦 A 写 B"，掉电恢复从未真正工作过（实测定位修复） */
#define CONFIG_FLASH_SECTOR     FLASH_Sector_11
#define CONFIG_FLASH_ADDR       0x080E0000UL
#define CONFIG_FLASH_SECTOR_SIZE (128 * 1024)

/* ---- EEPROM 后端参数（CONFIG_USE_EEPROM=1 时使用）---- */
#define CONFIG_EEPROM_ADDR      0x00    /* 配置在 AT24C02 中的起始地址 */

/* 魔数："SGDG" = SmartGuard */
#define CONFIG_MAGIC            0x53474447UL

/* ==== 配置结构体（56 字节，两种后端通用） ==== */
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

/* 对外接口（签名与后端无关，上层零改动） */
uint8_t  Config_Load(AT_Baseline_t *bl, Guard_Mode_t *mode);
uint8_t  Config_Save(const AT_Baseline_t *bl, Guard_Mode_t mode);

#endif /* __CONFIG_STORE_H */
