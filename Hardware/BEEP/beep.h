/**
  ******************************************************************************
  * @file    beep.h
  * @brief   蜂鸣器驱动头文件（TIM13_CH1 PWM 输出，可播放任意音调/音符）
  *
  * 硬件连接（STM32F407ZE）：
  *   蜂鸣器正极 -> PF8 = TIM13_CH1（复用 AF9，PWM 方波驱动无源蜂鸣器发声）
  *   蜂鸣器负极 -> GND（板载三极管驱动，具体以开发板原理图为准）
  *
  * 定时参数：
  *   TIM13 挂载 APB1，总线时钟 84MHz
  *   PSC = 84 - 1 -> 计数器时钟 1MHz（每 1us 加 1）
  *   ARR 动态按频率设置：ARR = 1,000,000 / freq - 1
  *   占空比固定 50%，保证蜂鸣器发声最大
  *
  * 使用方法：
  *   Beep_Init();                          // 初始化（上电后调用一次，默认静音）
  *   Beep_SetFreq(440);  Beep_On();        // 发 440Hz（中音LA）持续响
  *   Beep_Off();                           // 立即静音
  *   Beep_PlayNote(NOTE_DO, 300);          // 播放"中音1" 300ms（阻塞式）
  ******************************************************************************
  */
#ifndef __BEEP_H__
#define __BEEP_H__

#include "stm32f4xx.h"
#include "delay.h"     /* Beep_PlayNote 内部使用 delay_ms() */

/* ========== 蜂鸣器总开关（比赛演示静音） ==========
   0 = 驱动层硬隔离：Beep_Init 不配置 PF8/TIM13，Beep_On/PlayNote 直接返回，
       任何模块误调用都不会发声，也不再有告警音的阻塞延时；
   1 = 恢复正常蜂鸣器功能。 */
#define BEEP_ENABLE     1

/* ========== 蜂鸣器硬件配置 ========== */
#define BEEP_PORT       GPIOF
#define BEEP_PIN        GPIO_Pin_8
#define BEEP_PIN_SRC    GPIO_PinSource8
#define BEEP_TIM        TIM13
#define BEEP_TIM_CLK    RCC_APB1Periph_TIM13
#define BEEP_AF         GPIO_AF_TIM13

/* ========== C 调自然大调音阶频率表（Hz） ========== */
/* ---- 低音区（第 3 八度，简单琴谱用） ---- */
#define NOTE_DO_LO      131     /* 低音 1（低Do / C3） */
#define NOTE_RE_LO      147     /* 低音 2（低Re / D3） */
#define NOTE_MI_LO      165     /* 低音 3（低Mi / E3） */
#define NOTE_SOL_LO     196     /* 低音 5（低Sol / G3） */

/* ---- 中音区（第 4 八度 C4 ~ B4） ---- */
#define NOTE_DO         262     /* 中音 1（Do  / C4） */
#define NOTE_RE         294     /* 中音 2（Re  / D4） */
#define NOTE_MI         330     /* 中音 3（Mi  / E4） */
#define NOTE_FA         349     /* 中音 4（Fa  / F4） */
#define NOTE_SOL        392     /* 中音 5（Sol / G4） */
#define NOTE_LA         440     /* 中音 6（La  / A4，国际标准音） */
#define NOTE_SI         494     /* 中音 7（Si  / B4） */

/* ---- 高音区（第 5 八度 C5 ~ A5） ---- */
#define NOTE_DO_HI      523     /* 高音 1（高Do / C5） */
#define NOTE_RE_HI      587     /* 高音 2（高Re / D5） */
#define NOTE_MI_HI      659     /* 高音 3（高Mi / E5） */
#define NOTE_FA_HI      698     /* 高音 4（高Fa / F5） */
#define NOTE_SOL_HI     784     /* 高音 5（高Sol / G5） */
#define NOTE_LA_HI      880     /* 高音 6（高La / A5） */

#define NOTE_REST        0      /* 休止符（不发声） */

/* ========== 常用音效频率（非标准音） ========== */
#define BEEP_OK_HI      NOTE_SOL    /* 成功提示用的高音（G5） */
#define BEEP_OK_MID     NOTE_MI     /* 成功提示用的中音（E5） */
#define BEEP_OK_LO      NOTE_DO     /* 成功提示用的低音（C5） */
#define BEEP_ERR        180         /* 错误提示用的低沉音（~180Hz） */

/* ========== 接口函数 ========== */
void Beep_Init(void);                                 /* GPIO + TIM13 PWM 初始化，默认静音 */
void Beep_On(void);                                   /* 开启 PWM 输出（开始发声） */
void Beep_Off(void);                                  /* 关闭 PWM 输出（立即静音） */
void Beep_SetFreq(uint16_t freq);                     /* 动态设置 PWM 频率（Hz），为 0 忽略 */
void Beep_PlayNote(uint16_t freq, uint16_t duration_ms); /* 播放一个音符（阻塞） */

#endif /* __BEEP_H__ */
