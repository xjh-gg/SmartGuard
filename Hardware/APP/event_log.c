/**
  ******************************************************************************
  * @file    event_log.c
  * @brief   告警事件历史日志实现（W25Qxx 环形追加写）
  ******************************************************************************
  */
#include "event_log.h"
#include "w25qxx.h"
#include "usart.h"
#include "rtc.h"
#include "decision_engine.h"   /* USE_RTC、DecisionEngine_GetAlertString */
#include <string.h>

/* ---- 日志区域参数 ---- */
#define EVENT_LOG_SECTOR_COUNT   4                          /* 4 x 4KB = 16KB */
#define EVENT_SEC_SIZE           4096UL
#define EVENT_REC_SIZE           16
#define RECS_PER_SEC             (EVENT_SEC_SIZE / EVENT_REC_SIZE)   /* 256 */
#define EVENT_LOG_CAPACITY       (EVENT_LOG_SECTOR_COUNT * RECS_PER_SEC)
#define EVENT_REC_MAGIC          0xEB

static uint8_t  s_present = 0;         /* W25Qxx 是否在线 */
static uint32_t s_write_addr = 0;      /* 下一条记录的字节地址（16 对齐） */
static uint16_t s_count = 0;           /* 已存有效记录数（≤ EVENT_LOG_CAPACITY） */

/* 上电扫描用的扇区缓冲（4KB 静态，栈只有 1KB 放不下） */
static uint8_t s_sec_buf[EVENT_SEC_SIZE];

/* ---- 记录校验 ---- */
static uint8_t rec_valid(const uint8_t *r)
{
    uint8_t sum = 0;
    uint8_t i;

    if (r[0] != EVENT_REC_MAGIC) return 0;
    for (i = 0; i < EVENT_REC_SIZE - 1; i++) sum += r[i];
    return (sum == r[EVENT_REC_SIZE - 1]) ? 1 : 0;
}

static uint8_t rec_erased(const uint8_t *r)
{
    uint8_t i;

    for (i = 0; i < EVENT_REC_SIZE; i++)
    {
        if (r[i] != 0xFF) return 0;
    }
    return 1;
}

/**
  * @brief  初始化：探测芯片 + 扫描日志区恢复写指针
  * @note   扫描原理：本模块保证"扇区内记录从 0 开始连续有效、之后全擦除态"
  *         的不变量，因此每扇区从 0 数连续有效记录数 n，n 即写入偏移。
  *         活动扇区 = 有空位且前驱扇区已满的扇区（环形日志的接缝）；
  *         全空（新片/刚清空）取 0 号扇区；全满则擦 0 号扇区重新回卷。
  */
uint8_t EventLog_Init(void)
{
    uint16_t wid;
    uint8_t  s;
    uint16_t off[EVENT_LOG_SECTOR_COUNT];
    int8_t   active = -1;

    wid = W25QXX_ReadID();
    if ((wid >> 8) != 0xEF)
    {
        s_present = 0;
        USART1_Printf("[EVENTLOG] W25Qxx not found (ID=0x%04X), log disabled\r\n", wid);
        return 0;
    }
    s_present = 1;
    s_count   = 0;

    for (s = 0; s < EVENT_LOG_SECTOR_COUNT; s++)
    {
        uint16_t n = 0;

        W25QXX_Read((uint32_t)s * EVENT_SEC_SIZE, s_sec_buf, EVENT_SEC_SIZE);
        while (n < RECS_PER_SEC && rec_valid(s_sec_buf + n * EVENT_REC_SIZE)) n++;

        /* 撕裂写保护：连续有效段之后的槽位若不是擦除态（写一半掉电），
           该槽无法再编程（NOR 只能 1->0），整个扇区按"已满"处理，
           写指针前滚到下一扇区，旧数据在回卷时随整扇区擦除自然消亡 */
        if (n < RECS_PER_SEC && !rec_erased(s_sec_buf + n * EVENT_REC_SIZE))
        {
            n = RECS_PER_SEC;
        }
        off[s] = n;
        s_count += n;
    }

    /* 找活动扇区：优先"有空位且前驱已满"的接缝扇区 */
    for (s = 0; s < EVENT_LOG_SECTOR_COUNT; s++)
    {
        uint8_t prev = (s == 0) ? (EVENT_LOG_SECTOR_COUNT - 1) : (s - 1);
        if (off[s] < RECS_PER_SEC && off[prev] >= RECS_PER_SEC)
        {
            active = (int8_t)s;
            break;
        }
    }
    if (active < 0)
    {
        /* 无接缝：全部有空位（新片）→ 从 0 号开始；全满 → 擦 0 号回卷 */
        if (off[0] < RECS_PER_SEC)
        {
            active = 0;
        }
        else
        {
            W25QXX_SectorErase(0);
            off[0] = 0;
            active = 0;
        }
    }

    s_write_addr = (uint32_t)active * EVENT_SEC_SIZE + (uint32_t)off[active] * EVENT_REC_SIZE;
    return 1;
}

/**
  * @brief  追加一条告警事件
  */
void EventLog_Add(uint8_t alert_type, const Sensor_Data_t *s, uint8_t mode)
{
    uint8_t rec[EVENT_REC_SIZE];
    uint8_t sum = 0;
    uint8_t i;
    uint32_t dist_cm;

    if (!s_present || s == (const Sensor_Data_t *)0) return;

    rec[0] = EVENT_REC_MAGIC;
    rec[1] = alert_type;

#if USE_RTC
    {
        RTC_DateTime_t dt;
        SG_RTC_GetTime(&dt);
        rec[2] = dt.Year;
        rec[3] = dt.Month;
        rec[4] = dt.Date;
        rec[5] = dt.Hours;
        rec[6] = dt.Minutes;
        rec[7] = dt.Seconds;
    }
#else
    memset(rec + 2, 0, 6);
#endif

    rec[8]  = (uint8_t)(s->light_adc >> 8);
    rec[9]  = (uint8_t)(s->light_adc);
    rec[10] = (uint8_t)(s->mq2_adc >> 8);
    rec[11] = (uint8_t)(s->mq2_adc);

    dist_cm = (s->distance > 0.0f) ? (uint32_t)(s->distance + 0.5f) : 0;
    if (dist_cm > 65535UL) dist_cm = 65535UL;
    rec[12] = (uint8_t)(dist_cm >> 8);
    rec[13] = (uint8_t)dist_cm;

    rec[14] = mode;

    for (i = 0; i < EVENT_REC_SIZE - 1; i++) sum += rec[i];
    rec[EVENT_REC_SIZE - 1] = sum;

    /* 进入新扇区：若该扇区残留旧数据（环形回卷），先整扇区擦除 */
    if ((s_write_addr & (EVENT_SEC_SIZE - 1)) == 0)
    {
        uint8_t first[EVENT_REC_SIZE];

        W25QXX_Read(s_write_addr, first, EVENT_REC_SIZE);
        if (!rec_erased(first))
        {
            W25QXX_SectorErase(s_write_addr);
        }
    }

    /* 16B 记录 16 对齐，永不跨 256B 页边界 */
    W25QXX_PageWrite(s_write_addr, rec, EVENT_REC_SIZE);

    s_write_addr += EVENT_REC_SIZE;
    if (s_write_addr >= EVENT_LOG_SECTOR_COUNT * EVENT_SEC_SIZE)
    {
        s_write_addr = 0;
    }
    if (s_count < EVENT_LOG_CAPACITY) s_count++;
}

/**
  * @brief  当前存储的事件条数
  */
uint16_t EventLog_Count(void)
{
    return s_count;
}

/**
  * @brief  串口回显最近 last_n 条（从写指针倒序回读，新的在前）
  */
void EventLog_Dump(uint16_t last_n)
{
    int32_t  addr;
    uint16_t printed = 0;

    if (!s_present) return;

    USART1_Printf("[EVENTLOG] %u record(s), latest %u:\r\n", s_count, last_n);

    addr = (int32_t)s_write_addr - EVENT_REC_SIZE;
    while (printed < last_n && printed < s_count)
    {
        uint8_t rec[EVENT_REC_SIZE];

        if (addr < 0) addr = EVENT_LOG_SECTOR_COUNT * EVENT_SEC_SIZE - EVENT_REC_SIZE;

        W25QXX_Read((uint32_t)addr, rec, EVENT_REC_SIZE);
        if (!rec_valid(rec)) break;

        USART1_Printf("  20%02d-%02d-%02d %02d:%02d:%02d  %-12s L=%u M=%u D=%ucm mode=%d\r\n",
                      rec[2], rec[3], rec[4], rec[5], rec[6], rec[7],
                      DecisionEngine_GetAlertString((Alert_Type_t)rec[1]),
                      ((uint16_t)rec[8] << 8) | rec[9],
                      ((uint16_t)rec[10] << 8) | rec[11],
                      ((uint16_t)rec[12] << 8) | rec[13],
                      rec[14]);

        addr -= EVENT_REC_SIZE;
        printed++;
    }
    if (printed == 0)
    {
        USART1_SendString("  (empty)\r\n");
    }
}

/**
  * @brief  清空全部事件
  */
void EventLog_Clear(void)
{
    uint8_t s;

    if (!s_present) return;

    for (s = 0; s < EVENT_LOG_SECTOR_COUNT; s++)
    {
        W25QXX_SectorErase((uint32_t)s * EVENT_SEC_SIZE);
    }
    s_write_addr = 0;
    s_count = 0;
}
