/**
  ******************************************************************************
  * @file    hcsr04.c
  * @brief   HC-SR04 超声波测距驱动（TIM8 输入捕获实现）
  *
  * 硬件连接：
  *   Trig = PC8  (GPIO 推挽)
  *   Echo = PC6  (TIM8_CH1, AF3)
  *
  * TIM8 配置：
  *   时钟 = 168MHz (APB2 默认不分频)
  *   Prescaler = 167 → 1MHz 计数频率（1 tick = 1us）
  *   ARR = 0xFFFF (自由溢出)
  *   IC1 (CH1) 捕获上升沿 → CCR1 存 Echo 拉高时刻
  *   IC2 (CH2) 捕获下降沿 → CCR2 存 Echo 拉低时刻
  *   Echo 高电平时间 = CCR2 - CCR1 (单位: us)
  *
  * 与旧 delay_us 轮询方案的优势：
  *   - 精确到微秒级，不受中断/调度延迟影响
  *   - 无阻塞轮询 CPU（只等两个边沿，不是 for 循环 30000 次）
  *   - 30ms 超时用 TIM 计数器差值判断，更准
  ******************************************************************************
  */
#include "hcsr04.h"
#include "delay.h"

volatile uint8_t g_hcsr04_err = HCSR04_ERR_OK;

/**
  * @brief  初始化 HC-SR04：Trig GPIO + Echo TIM8 输入捕获
  */
void HCSR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    /* ====== 开时钟 ====== */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    /* ====== Trig: PC8 推挽输出 ====== */
    GPIO_InitStruct.GPIO_Pin   = HCSR04_TRIG_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HCSR04_TRIG_PORT, &GPIO_InitStruct);
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    /* ====== Echo: PC6 AF3 (TIM8_CH1) ====== */
    GPIO_InitStruct.GPIO_Pin   = HCSR04_ECHO_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HCSR04_ECHO_PORT, &GPIO_InitStruct);
    GPIO_PinAFConfig(HCSR04_ECHO_PORT, GPIO_PinSource6, GPIO_AF_TIM8);

    /* ====== TIM8 时基：1MHz (1 tick = 1us) ====== */
    TIM_TimeBaseStruct.TIM_Prescaler         = 167;     /* 168MHz / (167+1) = 1MHz */
    TIM_TimeBaseStruct.TIM_Period            = 0xFFFF;  /* 自由溢出 */
    TIM_TimeBaseStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;        /* 仅高级定时器需要 */
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStruct);

    /* ====== CH1: 捕获 Echo 上升沿 → CCR1 ====== */
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM8, &TIM_ICInitStruct);

    /* ====== CH2: 捕获 Echo 下降沿 → CCR2 ====== */
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel     = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Falling;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI; /* TI1FP1 */
    TIM_ICInit(TIM8, &TIM_ICInitStruct);

    /* ====== 使能 TIM8 ====== */
    TIM_Cmd(TIM8, ENABLE);

    delay_ms(10);
}

/**
  * @brief  发送 Trig 脉冲 + 等待 Echo 返回
  * @retval Echo 高电平时间（us），失败返回 0
  */
static uint32_t HCSR04_GetEchoTimeUs(void)
{
    uint32_t start_tick = 0;
    uint32_t rising_ccr = 0;
    uint32_t falling_ccr = 0;
    uint32_t echo_us = 0;

    g_hcsr04_err = HCSR04_ERR_OK;

    /* ====== 清除上一次捕获残留 ====== */
    TIM_ClearFlag(TIM8, TIM_FLAG_CC1);
    TIM_ClearFlag(TIM8, TIM_FLAG_CC2);
    TIM_SetCounter(TIM8, 0);
    start_tick = TIM_GetCounter(TIM8);

    /* ====== Trig: 15us 高脉冲 ====== */
    GPIO_SetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    delay_us(HCSR04_TRIG_US);
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    /* ====== 等待上升沿 (CC1 flag) ======
       超时：从 start_tick 算起 30000 ticks = 30ms */
    while (TIM_GetFlagStatus(TIM8, TIM_FLAG_CC1) == RESET)
    {
        if (TIM_GetCounter(TIM8) - start_tick > HCSR04_TIMEOUT_US)
        {
            g_hcsr04_err = HCSR04_ERR_NO_RISING;
            return 0;
        }
    }
    rising_ccr = TIM_GetCapture1(TIM8);

    /* ====== 等待下降沿 (CC2 flag) ====== */
    while (TIM_GetFlagStatus(TIM8, TIM_FLAG_CC2) == RESET)
    {
        if (TIM_GetCounter(TIM8) - rising_ccr > HCSR04_TIMEOUT_US)
        {
            g_hcsr04_err = HCSR04_ERR_NO_FALLING;
            return 0;
        }
    }
    falling_ccr = TIM_GetCapture2(TIM8);

    /* ====== Echo 时间 = 下降沿 - 上升沿 (均为 us) ====== */
    echo_us = falling_ccr - rising_ccr;
    return echo_us;
}

float HCSR04_GetDistance(void)
{
    uint32_t t = HCSR04_GetEchoTimeUs();
    if (t == 0) return 0.0f;
    return (float)t / 58.0f;
}

uint32_t HCSR04_GetDistanceMM(void)
{
    uint32_t t = HCSR04_GetEchoTimeUs();
    if (t == 0) return 0;
    return t * 17 / 100;
}

void HCSR04_GetDistanceBoth(uint32_t *cm, uint32_t *mm)
{
    uint32_t t = HCSR04_GetEchoTimeUs();
    if (t == 0) { *cm = 0; *mm = 0; return; }
    *cm = t / 58;
    *mm = t * 17 / 100;
}
