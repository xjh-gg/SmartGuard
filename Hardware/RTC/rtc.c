/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   RTC driver for STM32F407 -- LSE/LSI auto-fallback + Wakeup Timer 1s interrupt
  *
  *  ?? Wakeup Timer ?????????? ST ? RTC_WakeUpCmd() ????? bug
  *  ?? ?? while ?????????????????????
  *  ?? ?????? [DBG] ???????????
  ******************************************************************************
  */
#include "rtc.h"
#include "usart.h"
#include "delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ========== 从编译日期字符串解析月份 ========== */
static uint8_t RTC_MonthFromCompileDate(const char *date)
{
    const char *m = date;
    if (strncmp(m, "Jan", 3) == 0) return 1;
    if (strncmp(m, "Feb", 3) == 0) return 2;
    if (strncmp(m, "Mar", 3) == 0) return 3;
    if (strncmp(m, "Apr", 3) == 0) return 4;
    if (strncmp(m, "May", 3) == 0) return 5;
    if (strncmp(m, "Jun", 3) == 0) return 6;
    if (strncmp(m, "Jul", 3) == 0) return 7;
    if (strncmp(m, "Aug", 3) == 0) return 8;
    if (strncmp(m, "Sep", 3) == 0) return 9;
    if (strncmp(m, "Oct", 3) == 0) return 10;
    if (strncmp(m, "Nov", 3) == 0) return 11;
    if (strncmp(m, "Dec", 3) == 0) return 12;
    return 1;
}

/* ========== 使用编译时间作为 RTC 初始时间 ==========
 * __DATE__ 格式示例: "Sep  8 2026"
 * __TIME__ 格式示例: "15:30:25"
 */
static void SG_RTC_SetCompileTime(void)
{
    uint8_t year  = (uint8_t)(atoi(__DATE__ + 7) - 2000); /* 年份后两位：2026 -> 26 */
    uint8_t month = RTC_MonthFromCompileDate(__DATE__);
    uint8_t date  = (uint8_t)atoi(__DATE__ + 4);          /* 日期 */
    uint8_t hour  = (uint8_t)atoi(__TIME__);
    uint8_t min   = (uint8_t)atoi(__TIME__ + 3);
    uint8_t sec   = (uint8_t)atoi(__TIME__ + 6);

    SG_RTC_SetDateTime(year, month, date, hour, min, sec);

    USART1_Printf("[RTC] Compile time set: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                  year, month, date, hour, min, sec);
}

/* ========== ?????????? ST ? bit ??? ========== */
#define RTC_WPR_UNLOCK()  do { RTC->WPR = 0xCA; RTC->WPR = 0x53; } while(0)
#define RTC_WPR_LOCK()    do { RTC->WPR = 0xFF; } while(0)

#define RTC_CR_WUTE       ((uint32_t)0x00000400)   /* CR bit 10 */
#define RTC_CR_WUCKSEL    ((uint32_t)0x00000007)   /* CR bits 0-2 */
#define RTC_CR_WUCKSEL_16 ((uint32_t)0x00000004)   /* ck_spre (1Hz when prescaler set) */
#define RTC_CR_WUCKSEL_RTC16 ((uint32_t)0x00000000) /* RTC/16 = 2048Hz with LSE */

#define RTC_ISR_WUTF      ((uint32_t)0x00000400)   /* ISR bit 10: wakeup triggered */
#define RTC_ISR_WUTWF     ((uint32_t)0x00000004)   /* ISR bit 2: WUTR writable */
/* RTC_ISR_INITF already defined in stm32f4xx.h ? don't redefine */

/* ========== Global state ========== */
volatile uint32_t g_rtc_tick_ms = 0;
volatile uint8_t  g_rtc_sec_flag = 0;
volatile uint8_t  g_rtc_ready = 0;
static void (*s_wkup_cb)(void) = 0;

/* ================================================================
 *                  SG_RTC_Init
 * ================================================================ */
void SG_RTC_Init(void)
{
    __IO uint32_t async = 0, sync = 0;
    uint8_t use_lse = 0;
    uint32_t wait;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    /* 禁用 PC13 的 RTC Tamper 功能，释放给 OLED_CS 使用 */
    RTC->TAFCR &= ~(RTC_TAFCR_TAMP1E | RTC_TAFCR_TAMPIE);

    /* ????LSE ?????????????? LSI */
    RCC_LSEConfig(RCC_LSE_ON);
    wait = 500000;
    while (wait-- && RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) { __NOP(); }

    if (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == SET) {
        use_lse = 1;
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        sync  = 0xFF;   async = 0x7F;
    } else {
        USART1_SendString("[RTC] LSE not ready, fallback LSI\r\n");
        RCC_LSICmd(ENABLE);
        wait = 500000;
        while (wait-- && RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) { __NOP(); }
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
        sync  = 0xF9;   async = 0x7F;
    }

    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();

    RTC_InitTypeDef rtcInit;
    rtcInit.RTC_AsynchPrediv = async;
    rtcInit.RTC_SynchPrediv  = sync;
    rtcInit.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init(&rtcInit);

    if (!SG_RTC_IsInitialized()) {
        SG_RTC_SetCompileTime();
        RTC_WriteBackupRegister(RTC_BKP_REG, RTC_INIT_MARKER);
    }

    USART1_Printf("[RTC] OK, source=%s\r\n", use_lse ? "LSE" : "LSI");
    PWR_BackupAccessCmd(DISABLE);
}

/* ================================================================
 *                  SG_RTC_SetWakeup1s
 *
 *  ?????????? [DBG] ???????
 *  ?? RM0090 ?21.8.25: Wakeup Timer ????
 * ================================================================ */
void SG_RTC_SetWakeup1s(void (*callback)(void))
{
    s_wkup_cb = callback;
    uint32_t wait;

    USART1_SendString("[DBG] WKUP: start config...\r\n");

    /* ??? ????????? Calendar ?? Init Mode????
     *  RM0090: WUTWF ??? Calendar ??? Init Mode ??????
     *  ???? RTC_Init() ? ExitInitMode ??????INITF ??? 1?
     *  ????? WUTWF ??????????
     */
    PWR_BackupAccessCmd(ENABLE);
    RTC_WPR_UNLOCK();

    USART1_Printf("[DBG] WKUP: before exit init, ISR=0x%08lX\r\n", (unsigned long)RTC->ISR);

    /* ???? Calendar Init Mode ?? ????????? */
    RTC_ExitInitMode();
    wait = 200000;
    while (wait-- && (RTC->ISR & RTC_ISR_INITF) != 0) { __NOP(); }

    if ((RTC->ISR & RTC_ISR_INITF) != 0) {
        USART1_SendString("[DBG] WKUP: INITF still 1 after ExitInitMode! Try again...\r\n");
        /* ??????????/? Init Mode */
        RTC_EnterInitMode();
        RTC_ExitInitMode();
        wait = 200000;
        while (wait-- && (RTC->ISR & RTC_ISR_INITF) != 0) { __NOP(); }
    }

    USART1_Printf("[DBG] WKUP: INITF cleared, ISR=0x%08lX\r\n", (unsigned long)RTC->ISR);

    /* Step 1: ? WUTE ?? ???????? Init Mode ? */
    RTC->CR &= ~RTC_CR_WUTE;
    USART1_SendString("[DBG] WKUP: WUTE cleared\r\n");

    /* Step 2: ? WUTWF ?? ????????????? WUTR ?? */
    wait = 2000000;
    while (wait-- && (RTC->ISR & RTC_ISR_WUTWF) == 0) { __NOP(); }
    if ((RTC->ISR & RTC_ISR_WUTWF) == 0) {
        USART1_SendString("[RTC] WARN: WUTWF timeout! Wakeup Timer aborted\r\n");
        RTC_WPR_LOCK();
        PWR_BackupAccessCmd(DISABLE);
        return;
    }
    USART1_SendString("[DBG] WKUP: WUTWF=1 OK\r\n");

    /* Step 3: ? WUTF ?????????????*/
    RTC->ISR = ~RTC_ISR_WUTF;   /* WUTF ? RC_W0???? 1?*/
    USART1_SendString("[DBG] WKUP: WUTF cleared\r\n");

    /* Step 4: 时钟源 = RTC/16 (直分频，绕开同步预分频器)
     *   LSE=32768Hz → RTC/16=2048Hz → WUTR=2047 → (2047+1)/2048 = 1.0秒 */
    RTC->CR &= ~RTC_CR_WUCKSEL;
    RTC->CR |= RTC_CR_WUCKSEL_RTC16;
    USART1_SendString("[DBG] WKUP: clock src = RTC/16 (2048Hz)\r\n");

    /* Step 5: 写计数器 2047 → 1 秒 */
    RTC->WUTR = 2047;
    USART1_SendString("[DBG] WKUP: WUTR=2047 (1s)\r\n");

    /* Step 6: ???? WUTWF ?? ? WUTR ??????? WUTWF */
    wait = 2000000;
    while (wait-- && (RTC->ISR & RTC_ISR_WUTWF) == 0) { __NOP(); }
    if ((RTC->ISR & RTC_ISR_WUTWF) == 0) {
        USART1_SendString("[RTC] WARN: WUTWF timeout after WUTR write! Try anyway\r\n");
    } else {
        USART1_SendString("[DBG] WKUP: WUTWF=1 after WUTR\r\n");
    }

    /* Step 7: ?? RTC Wakeup ?? */
    RTC_ClearITPendingBit(RTC_IT_WUT);
    RTC_ITConfig(RTC_IT_WUT, ENABLE);

    /* Step 8: EXTI Line22 ???RTC_WKUP ??????*/
    EXTI_InitTypeDef exti;
    exti.EXTI_Line    = EXTI_Line22;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    EXTI_ClearITPendingBit(EXTI_Line22);

    /* Step 9: NVIC RTC_WKUP_IRQn */
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = RTC_WKUP_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* Step 10: 使能 WUTE —— 重新解锁 WPR 确保写入生效 */
    RTC_WPR_UNLOCK();
    USART1_Printf("[DBG] WKUP: WPR re-unlocked, CR before WUTE=0x%08lX\r\n", (unsigned long)RTC->CR);
    RTC->CR |= RTC_CR_WUTE;
    USART1_Printf("[DBG] WKUP: WUTE written, CR=0x%08lX\r\n", (unsigned long)RTC->CR);

    /* Step 11: 等第一个 WUTF（用 delay_ms 真等 1.5 秒）*/
    delay_ms(1500);
    if ((RTC->ISR & RTC_ISR_WUTF) != 0) {
        USART1_SendString("[DBG] WKUP: [OK] first WUTF triggered! Timer works\r\n");
        RTC->ISR = ~RTC_ISR_WUTF;
        g_rtc_ready = 1;
    } else {
        USART1_Printf("[DBG] WKUP: WUTF not seen after 1.5s, ISR=0x%08lX\r\n", (unsigned long)RTC->ISR);
    }

    /* ?? */
    RTC_WPR_LOCK();
    PWR_BackupAccessCmd(DISABLE);

    USART1_SendString("[RTC] Wakeup 1s interrupt enabled\r\n");
}

/* ================================================================
 *                  RTC_IRQ_Callback
 * ================================================================ */
void RTC_IRQ_Callback(void)
{
    if (RTC_GetITStatus(RTC_IT_WUT) == SET)
    {
        RTC_ClearITPendingBit(RTC_IT_WUT);
        EXTI_ClearITPendingBit(EXTI_Line22);

        g_rtc_tick_ms += 1000;
        g_rtc_sec_flag = 1;

        if (s_wkup_cb) {
            s_wkup_cb();
        }
    }
}

/* ================================================================
 *                  ????
 * ================================================================ */
uint8_t SG_RTC_IsInitialized(void)
{
    return (RTC_ReadBackupRegister(RTC_BKP_REG) == RTC_INIT_MARKER);
}

void SG_RTC_SetDateTime(uint8_t year, uint8_t month, uint8_t date,
                        uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    uint32_t wait;

    PWR_BackupAccessCmd(ENABLE);
    RTC_WPR_UNLOCK();
    if (RTC_EnterInitMode() != SUCCESS) {
        USART1_SendString("[RTC] ERR: enter init mode failed\r\n");
        RTC_WPR_LOCK();
        PWR_BackupAccessCmd(DISABLE);
        return;
    }

    RTC_DateTypeDef d;
    d.RTC_Year    = ((year / 10) << 4) | (year % 10);
    d.RTC_Month   = ((month / 10) << 4) | (month % 10);
    d.RTC_Date    = ((date / 10) << 4) | (date % 10);
    d.RTC_WeekDay = 1;
    RTC_SetDate(RTC_Format_BCD, &d);

    RTC_TimeTypeDef t;
    t.RTC_H12     = RTC_H12_AM;
    t.RTC_Hours   = ((hours / 10) << 4) | (hours % 10);
    t.RTC_Minutes = ((minutes / 10) << 4) | (minutes % 10);
    t.RTC_Seconds = ((seconds / 10) << 4) | (seconds % 10);
    RTC_SetTime(RTC_Format_BCD, &t);

    RTC_ExitInitMode();

    /* 等待日历寄存器同步完成 */
    wait = 1000000;
    while (wait-- && (RTC->ISR & RTC_ISR_RSF) == 0) { __NOP(); }

    RTC_WPR_LOCK();
    PWR_BackupAccessCmd(DISABLE);
}

void SG_RTC_GetTime(RTC_DateTime_t *dt)
{
    if (!dt) return;
    RTC_DateTypeDef d;
    RTC_TimeTypeDef t;
    RTC_GetDate(RTC_Format_BIN, &d);
    RTC_GetTime(RTC_Format_BIN, &t);
    dt->Year    = d.RTC_Year;
    dt->Month   = d.RTC_Month;
    dt->Date    = d.RTC_Date;
    dt->WeekDay = d.RTC_WeekDay;
    dt->Hours   = t.RTC_Hours;
    dt->Minutes = t.RTC_Minutes;
    dt->Seconds = t.RTC_Seconds;
}

void SG_RTC_FormatTime(char *buf, uint8_t bufLen)
{
    if (!buf || bufLen < 9) return;
    RTC_DateTime_t dt;
    SG_RTC_GetTime(&dt);
    snprintf(buf, bufLen, "%02d:%02d:%02d", dt.Hours, dt.Minutes, dt.Seconds);
}

void SG_RTC_FormatDate(char *buf, uint8_t bufLen)
{
    if (!buf || bufLen < 12) return;
    RTC_DateTime_t dt;
    SG_RTC_GetTime(&dt);
    snprintf(buf, bufLen, "20%02d-%02d-%02d", dt.Year, dt.Month, dt.Date);
}
