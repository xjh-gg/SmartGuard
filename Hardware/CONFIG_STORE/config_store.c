/**
  ******************************************************************************
  * @file    config_store.c
  * @brief   SmartGuard 配置掉电存储实现
  *
  * 核心流程：
  *   Save: 解锁 FLASH → 擦除 Sector 7 → 逐字写入 ConfigStore_t → 锁 FLASH
  *   Load: 直接指针转换读取 → CRC 校验 → 有效返回 1/无效返回 0
  ******************************************************************************
  */
#include "config_store.h"
#include <stddef.h>   /* offsetof 宏 */

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
    /* 硬件 CRC 单元自动复位后再计算 */
    return CRC_CalcBlockCRC((uint32_t *)((uint8_t *)cfg + CRC_RANGE_OFFSET),
                            CRC_RANGE_LEN);
}

/**
  * @brief  加载配置（上电时调用）
  * @param  bl:   输出基线
  * @param  mode: 输出模式
  * @retval 1 = 成功（CRC 有效），0 = 失败（CRC 无效/未初始化）
  */
uint8_t Config_Load(AT_Baseline_t *bl, Guard_Mode_t *mode)
{
    ConfigStore_t *p = (ConfigStore_t *)CONFIG_FLASH_ADDR;

    /* 1. 检查魔数 */
    if (p->magic != CONFIG_MAGIC) return 0;

    /* 2. CRC 校验 */
    if (p->crc32 != Config_CalcCRC(p)) return 0;

    /* 3. 数据有效，复制出来 */
    bl->light_baseline    = p->light_baseline;
    bl->temp_baseline     = p->temp_baseline;
    bl->humi_baseline     = p->humi_baseline;
    bl->is_learned        = p->is_learned;
    bl->light_sigma       = p->light_sigma;
    bl->mq2_sigma         = p->mq2_sigma;
    bl->temp_sigma        = p->temp_sigma;
    bl->humi_sigma        = p->humi_sigma;
    bl->mq2_fire_threshold= p->mq2_fire_threshold;
    bl->light_ewma        = p->light_ewma;
    bl->temp_ewma         = p->temp_ewma;
    bl->humi_ewma         = p->humi_ewma;
    bl->mq2_ewma          = p->mq2_ewma;
    *mode = p->mode;
    return 1;
}

/**
  * @brief  保存配置（基线学习完或模式切换时调用）
  * @param  bl:   输入基线
  * @param  mode: 输入模式
  * @retval 1 = 成功，0 = 失败（FLASH 擦写错误）
  */
uint8_t Config_Save(const AT_Baseline_t *bl, Guard_Mode_t mode)
{
    ConfigStore_t cfg;
    uint32_t status;
    uint32_t i;
    uint32_t *p_src;
    uint32_t *p_dst;

    /* 1. 填充结构体 */
    cfg.magic             = CONFIG_MAGIC;
    cfg.light_baseline    = bl->light_baseline;
    cfg.temp_baseline     = bl->temp_baseline;
    cfg.humi_baseline     = bl->humi_baseline;
    cfg.mode              = mode;
    cfg.is_learned        = bl->is_learned;
    cfg.reserved          = 0;
    cfg.light_sigma       = bl->light_sigma;
    cfg.mq2_sigma         = bl->mq2_sigma;
    cfg.temp_sigma        = bl->temp_sigma;
    cfg.humi_sigma        = bl->humi_sigma;
    cfg.mq2_fire_threshold= bl->mq2_fire_threshold;
    cfg.reserved2         = 0;
    cfg.light_ewma        = bl->light_ewma;
    cfg.temp_ewma         = bl->temp_ewma;
    cfg.humi_ewma         = bl->humi_ewma;
    cfg.mq2_ewma          = bl->mq2_ewma;
    cfg.crc32             = Config_CalcCRC(&cfg);   /* CRC 不包含自己，所以放最后计算 */

    /* 2. 解锁 FLASH */
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    /* 3. 擦除 Sector 7（最后一个 128KB 扇区） */
    status = FLASH_EraseSector(CONFIG_FLASH_SECTOR, VoltageRange_3);
    if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        return 0;
    }

    /* 4. 逐字（4 字节）写入 ConfigStore_t 到 FLASH */
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

    /* 5. 锁 FLASH */
    FLASH_Lock();
    return 1;
}
