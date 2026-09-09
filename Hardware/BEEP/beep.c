/**
  ******************************************************************************
  * @file    beep.c
  * @brief   蜂鸣器驱动程序（TIM13_CH1 PWM 模式，可设置频率播放音符/音效）
  *
  * 原理：
  *   无源蜂鸣器需要一定频率的方波才能发声。使用定时器 PWM1 模式，
  *   通过修改自动重装载值 ARR 改变输出频率，通过比较寄存器 CCR1 固定
  *   50% 占空比，保证蜂鸣器在每个频率下音量最大。
  ******************************************************************************
  */
#include "beep.h"

/**
  * @brief  蜂鸣器初始化（GPIO 复用为 TIM13_CH1 + TIM PWM 配置，默认静音）
  */
void Beep_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef       TIM_OCInitStruct;

    /* 开时钟：PF8 + TIM13 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_APB1PeriphClockCmd(BEEP_TIM_CLK, ENABLE);

    /* PF8 配置为复用推挽输出 -> TIM13_CH1 */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = BEEP_PIN;
    GPIO_Init(BEEP_PORT, &GPIO_InitStruct);
    GPIO_PinAFConfig(BEEP_PORT, BEEP_PIN_SRC, BEEP_AF);

    /* ---- 时基：PSC=84-1 -> 计数频率 1MHz ---- */
    TIM_TimeBaseStruct.TIM_Prescaler         = 84 - 1;
    TIM_TimeBaseStruct.TIM_Period            = 1000 - 1;   /* 默认 ARR，之后按频率覆盖 */
    TIM_TimeBaseStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BEEP_TIM, &TIM_TimeBaseStruct);

    /* ---- PWM1 模式：CNT < CCR 时输出高电平，否则低 ---- */
    TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_Pulse       = (1000 - 1) / 2;   /* 默认 50% 占空比 */
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(BEEP_TIM, &TIM_OCInitStruct);

    /* 开启预装载，保证修改 ARR/CCR 时不产生 glitch */
    TIM_OC1PreloadConfig(BEEP_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BEEP_TIM, ENABLE);

    /* 默认关闭通道输出（静音）—— Beep_On() 再打开 */
    TIM_CCxCmd(BEEP_TIM, TIM_Channel_1, DISABLE);

    /* 启动 TIM13 计数器（PWM 输出与否取决于 CC1 通道开关） */
    TIM_Cmd(BEEP_TIM, ENABLE);
}

/**
  * @brief  蜂鸣器开始发声（打开 TIM13_CH1 PWM 输出）
  *         频率由最近一次 Beep_SetFreq() 决定，默认 1000Hz
  */
void Beep_On(void)
{
    TIM_CCxCmd(BEEP_TIM, TIM_Channel_1, ENABLE);
}

/**
  * @brief  蜂鸣器立即静音（关闭 TIM13_CH1 输出，ARR/CCR 保持不变）
  */
void Beep_Off(void)
{
    TIM_CCxCmd(BEEP_TIM, TIM_Channel_1, DISABLE);
}

/**
  * @brief  动态设置蜂鸣器发声频率
  * @param  freq: 目标频率（Hz）。为 0 时直接返回不处理
  *
  * 计算：
  *   计数频率 = 1MHz，周期 = 1us
  *   目标周期 = 1 / freq (s) = 1,000,000 / freq (us)
  *   ARR = 1,000,000 / freq - 1
  *   CCR = ARR / 2 （保持 50% 占空比）
  */
void Beep_SetFreq(uint16_t freq)
{
    uint32_t arr;

    if (freq == 0) return;

    arr = 1000000UL / freq - 1;
    if (arr > 65535) arr = 65535;   /* ARR 寄存器只有 16 位，最低 ~15Hz 兜底 */

    TIM_SetAutoreload(BEEP_TIM, arr);
    /* 强制产生一次更新事件，新 ARR 立即生效（不等到下一次溢出） */
    TIM_GenerateEvent(BEEP_TIM, TIM_EventSource_Update);
    /* 保持 50% 占空比 */
    TIM_SetCompare1(BEEP_TIM, arr / 2);
}

/**
  * @brief  播放一个音符（阻塞式，期间 CPU 不做其他事）
  * @param  freq:        频率（Hz），可用 NOTE_DO ~ NOTE_SI 或自定义
  * @param  duration_ms: 持续时长（毫秒）
  *
  * 典型用法：
  *   Beep_PlayNote(NOTE_DO, 200);   // 中音 Do 响 0.2 秒
  */
void Beep_PlayNote(uint16_t freq, uint16_t duration_ms)
{
    Beep_SetFreq(freq);
    Beep_On();
    delay_ms(duration_ms);
    Beep_Off();
}
