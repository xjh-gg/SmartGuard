/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   RTC 实时时钟驱动 —— LSE/LSI 自动降级 + Wakeup Timer 秒中断
  *
  *  ?? 函数名加 SG_ 前缀避免与 ST 标准库 rtc.h 冲突：
  *     - ST 库有 RTC_Init(RTC_InitTypeDef*) 和 RTC_GetTime(Format, Time*)
  *     - 本模块用 SG_RTC_Init() / SG_RTC_GetTime(RTC_DateTime_t*)
  *
  *  功能：
  *   1. SG_RTC_Init()           —— 初始化 RTC（LSE 优先，失败 fallback LSI）
  *   2. SG_RTC_SetWakeup1s()    —— 配置 Wakeup Timer 每秒触发 EXTI Line22 中断
  *   3. SG_RTC_GetTime()        —— 读取当前时间（BIN 格式）
  *   4. SG_RTC_SetDateTime()    —— 设置初始时间（仅首次，靠 BKP 记忆）
  *   5. RTC_IRQ_Callback()      —— 中断回调（在 stm32f4xx_it.c 的 RTC_WKUP_IRQHandler 调用）
  *
  *  注意事项：
  *   - RTC 域寄存器需要 PWR_BackupAccessCmd(ENABLE) 解除写保护
  *   - 设置时间前必须先 RTC_EnterInitMode()，写好后 RTC_ExitInitMode()
  *   - 写 Wakeup Counter 前必须等 RTC_FLAG_WUTWF 置位（RM0090 强制要求）
  ******************************************************************************
  */
#ifndef __RTC_H
#define __RTC_H

#include "stm32f4xx.h"

/* ========== 可配置参数 ========== */
#define RTC_INIT_MARKER   0x5245    /* 备份寄存器魔数："RC" 版本3 —— 改变此值可强制重新写入时间 */
#define RTC_BKP_REG       RTC_BKP_DR0

/* ========== 数据结构 ========== */
typedef struct
{
    uint8_t Year;     /* 0-99 (20xx) */
    uint8_t Month;    /* 1-12 */
    uint8_t Date;     /* 1-31 */
    uint8_t WeekDay;  /* 1-7 (Monday=1, Sunday=7) */
    uint8_t Hours;    /* 0-23 */
    uint8_t Minutes;  /* 0-59 */
    uint8_t Seconds;  /* 0-59 */
} RTC_DateTime_t;

/* ========== 全局回调（在中断里调用）========== */
extern volatile uint32_t g_rtc_tick_ms;  /* 每秒 +1000，可用来做非阻塞延时 */
extern volatile uint8_t  g_rtc_sec_flag;   /* 每秒置 1，主循环清 0 —— 零阻塞秒回调 */

/**
  * @brief  初始化 RTC（自动选 LSE，失败 fallback LSI）
  */
void SG_RTC_Init(void);

/**
  * @brief  设置初始日期时间（BCD 格式传给 STM32）
  * @note   只有 BKP 里没有 INIT_MARKER 时才真正写入，之后上电自动跳过
  */
void SG_RTC_SetDateTime(uint8_t year, uint8_t month, uint8_t date,
                        uint8_t hours, uint8_t minutes, uint8_t seconds);

/**
  * @brief  获取当前时间（BIN 格式，方便直接用）
  */
void SG_RTC_GetTime(RTC_DateTime_t *dt);

/**
  * @brief  配置 Wakeup Timer 每秒触发一次 EXTI Line22 中断
  * @param  callback  每秒回调函数指针（可为 NULL）
  *
  *  ?? EXTI Line22 是 RTC_WKUP 专用线（不是 Line20 USB HS / Line17 AlarmA）
  *  ?? 写 WUTR 前必须等 WUTWF 置位
  *  ?? 计数器 0 → (0+1)/ck_spre = 1 秒（ck_spre 由预分频器降至 1Hz）
  */
void SG_RTC_SetWakeup1s(void (*callback)(void));

/**
  * @brief  RTC Wakeup 中断回调 —— 在 stm32f4xx_it.c 的 RTC_WKUP_IRQHandler 里调用
  */
void RTC_IRQ_Callback(void);

/**
  * @brief  判断 RTC 是否已初始化过（BKP 有 marker = 跳过首次设置时间）
  */
uint8_t SG_RTC_IsInitialized(void);

/**
  * @brief  格式化输出时间/日期字符串
  */
void SG_RTC_FormatTime(char *buf, uint8_t bufLen);
void SG_RTC_FormatDate(char *buf, uint8_t bufLen);

#endif /* __RTC_H */
