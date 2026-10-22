/**
  ******************************************************************************
  * @file    config_store.c
  * @brief   SmartGuard 配置掉电存储实现（双后端，CONFIG_USE_EEPROM 切换）
  *
  * 通用流程：
  *   Save: 填充 ConfigStore_t → 算 CRC32 → 写入后端介质
  *   Load: 读出 56 字节 → magic 校验 → CRC 校验 → 有效才复制输出
  *
  * CRC 设计意图：写入是非原子的（EEPROM 分 4 页 / FLASH 逐字），写一半
  *   掉电会留下撕裂数据。CRC 保证"要么读到完整旧数据，要么明确知道无效"，
  *   宁可重新学习 20s 也不用撕裂基线跑决策引擎。
  ******************************************************************************
  */
#include "config_store.h"
#include "usart.h"
#include <stddef.h>   /* offsetof 宏 */

#if CONFIG_USE_EEPROM
#include "at24c02.h"
#else
#include "iwdg.h"      /* 擦除前喂狗（128KB 擦除期间 CPU 取指停顿，无法喂狗） */
#endif

/* ====== CRC 计算范围偏移：magic 到 reserved2，不含 crc32（由 offsetof 自动计算） ====== */
#define CRC_RANGE_OFFSET  (offsetof(ConfigStore_t, magic))
#define CRC_RANGE_LEN     (offsetof(ConfigStore_t, crc32) - offsetof(ConfigStore_t, magic))

/**
  * @brief  计算配置数据 CRC32（使用 STM32F4 硬件 CRC 单元）
  */
static uint32_t Config_CalcCRC(const ConfigStore_t *cfg)
{
    /* 开 CRC 时钟（如果还没开） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
    /* 硬件 CRC 是链式移位寄存器：不复位会链到上一次计算的结果上，
       导致保存时算的 CRC 与加载时重算的永远不一致（断电实测复现）。
       同时注意 CalcBlockCRC 的长度参数单位是"字"而非"字节"，
       offsetof 差值是字节数，必须 /4，否则会多读 3 倍数据
       （保存时读到栈垃圾、加载时读到 FLASH 尾部，同样永远不匹配）。 */
    CRC_ResetDR();
    return CRC_CalcBlockCRC((uint32_t *)((uint8_t *)cfg + CRC_RANGE_OFFSET),
                            CRC_RANGE_LEN / 4);
}

/* 把基线/模式填进存储结构体并算好 CRC（Save 路径共用） */
static void Config_Fill(ConfigStore_t *cfg, const AT_Baseline_t *bl, Guard_Mode_t mode)
{
    cfg->magic              = CONFIG_MAGIC;
    cfg->light_baseline     = bl->light_baseline;
    cfg->temp_baseline      = bl->temp_baseline;
    cfg->humi_baseline      = bl->humi_baseline;
    cfg->mode               = mode;
    cfg->is_learned         = bl->is_learned;
    cfg->reserved           = 0;
    cfg->light_sigma        = bl->light_sigma;
    cfg->mq2_sigma          = bl->mq2_sigma;
    cfg->temp_sigma         = bl->temp_sigma;
    cfg->humi_sigma         = bl->humi_sigma;
    cfg->mq2_fire_threshold = bl->mq2_fire_threshold;
    cfg->reserved2          = 0;
    cfg->light_ewma         = bl->light_ewma;
    cfg->temp_ewma          = bl->temp_ewma;
    cfg->humi_ewma          = bl->humi_ewma;
    cfg->mq2_ewma           = bl->mq2_ewma;
    cfg->crc32              = Config_CalcCRC(cfg);   /* CRC 不包含自己，所以放最后计算 */
}

/* 校验通过后将存储结构体展开到输出参数（Load 路径共用） */
static void Config_Unpack(const ConfigStore_t *cfg, AT_Baseline_t *bl, Guard_Mode_t *mode)
{
    bl->light_baseline     = cfg->light_baseline;
    bl->temp_baseline      = cfg->temp_baseline;
    bl->humi_baseline      = cfg->humi_baseline;
    bl->is_learned         = cfg->is_learned;
    bl->light_sigma        = cfg->light_sigma;
    bl->mq2_sigma          = cfg->mq2_sigma;
    bl->temp_sigma         = cfg->temp_sigma;
    bl->humi_sigma         = cfg->humi_sigma;
    bl->mq2_fire_threshold = cfg->mq2_fire_threshold;
    bl->light_ewma         = cfg->light_ewma;
    bl->temp_ewma          = cfg->temp_ewma;
    bl->humi_ewma          = cfg->humi_ewma;
    bl->mq2_ewma           = cfg->mq2_ewma;
    *mode = cfg->mode;
}

#if CONFIG_USE_EEPROM
/* ================= AT24C02 EEPROM 后端 ================= */

/**
  * @brief  加载配置（上电时调用）
  * @retval 1 = 成功（magic+CRC 有效），0 = 失败/器件不在
  */
uint8_t Config_Load(AT_Baseline_t *bl, Guard_Mode_t *mode)
{
    ConfigStore_t cfg;

    /* 56 字节从 EEPROM 顺序读出（器件 NACK 直接判失败） */
    if (AT24C02_Read(CONFIG_EEPROM_ADDR, (uint8_t *)&cfg, sizeof(cfg)) != 0)
    {
        USART1_SendString("[CONFIG] EEPROM read fail (device missing?)\r\n");
        return 0;
    }

    if (cfg.magic != CONFIG_MAGIC)
    {
        USART1_SendString("[CONFIG] magic mismatch (first learn or data erased)\r\n");
        return 0;
    }
    if (cfg.crc32 != Config_CalcCRC(&cfg))
    {
        USART1_SendString("[CONFIG] CRC mismatch (EEPROM data corrupted)\r\n");
        return 0;
    }

    Config_Unpack(&cfg, bl, mode);
    return 1;
}

/**
  * @brief  保存配置
  * @note   EEPROM 按字节擦写：无扇区擦除、无 CPU 停顿、无需喂狗保护，
  *         56 字节分 4 页写入，每页写周期 ≤5ms（ACK 轮询），全程 ~20ms。
  * @retval 1 = 成功，0 = 失败
  */
uint8_t Config_Save(const AT_Baseline_t *bl, Guard_Mode_t mode)
{
    ConfigStore_t cfg;

    Config_Fill(&cfg, bl, mode);

    if (AT24C02_Write(CONFIG_EEPROM_ADDR, (const uint8_t *)&cfg, sizeof(cfg)) != 0)
    {
        USART1_SendString("[CONFIG] EEPROM write fail\r\n");
        return 0;
    }
    return 1;
}

#else
/* ================= 内部 FLASH Sector 11 后端（历史方案，回退用） ================= */

uint8_t Config_Load(AT_Baseline_t *bl, Guard_Mode_t *mode)
{
    ConfigStore_t *p = (ConfigStore_t *)CONFIG_FLASH_ADDR;

    if (p->magic != CONFIG_MAGIC) return 0;
    if (p->crc32 != Config_CalcCRC(p)) return 0;

    Config_Unpack(p, bl, mode);
    return 1;
}

uint8_t Config_Save(const AT_Baseline_t *bl, Guard_Mode_t mode)
{
    ConfigStore_t cfg;
    uint32_t status;
    uint32_t i;
    uint32_t *p_src;
    uint32_t *p_dst;

    Config_Fill(&cfg, bl, mode);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    /* 擦除 Sector 11（128KB）。单 Bank：擦除期间 CPU 取指停顿（典型 ~1s，
       最大 ~2s），无法喂狗，擦前先喂一次把 IWDG(2s) 窗口拉满 */
    IWDG_Feed();
    status = FLASH_EraseSector(CONFIG_FLASH_SECTOR, VoltageRange_3);
    if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        return 0;
    }

    p_src = (uint32_t *)&cfg;
    p_dst = (uint32_t *)CONFIG_FLASH_ADDR;
    for (i = 0; i < sizeof(cfg) / 4; i++)
    {
        status = FLASH_ProgramWord((uint32_t)(p_dst + i), p_src[i]);
        if (status != FLASH_COMPLETE) {
            FLASH_Lock();
            return 0;
        }
    }

    FLASH_Lock();
    return 1;
}

#endif /* CONFIG_USE_EEPROM */
