/**
  ******************************************************************************
  * @file    delay.c
  * @brief   延时驱动实现：SysTick 1ms 中断 + 阻塞延时 + 全局节拍
  *
  * 原理：
  *   SysTick_Config(SystemCoreClock / 1000) -> SysTick 每 1ms 触发一次中断，
  *   中断里把 g_tick 自增（非阻塞计时用），并递减 TimingDelay（阻塞延时用）。
  ******************************************************************************
  */
#include "delay.h"

/* 阻塞延时的倒计时值（SysTick_Update 里递减，为 0 时延时结束） */
static volatile uint32_t TimingDelay = 0;

/* 全局 1ms 节拍：主循环可用 g_tick 差值做非阻塞定时（见 main.c 呼吸灯） */
volatile uint32_t g_tick = 0;

/* DWT  Cycle Counter 寄存器定义（Cortex-M4 内置，不占用 SysTick） */
#define DWT_CYCCNT      (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL        (*(volatile uint32_t *)0xE0001000)
#define SCB_DEMCR       (*(volatile uint32_t *)0xE000EDFC)

/**
  * @brief  使能 DWT 的 CYCCNT 计数器，用于微秒级精确延时
  */
static void DWT_Init(void)
{
    SCB_DEMCR |= (uint32_t)(1 << 24);   /* TRCENA = 1，使能调试跟踪 */
    DWT_CTRL  |= (uint32_t)(1 << 0);    /* CYCCNTENA = 1，启动周期计数器 */
}

/**
  * @brief  初始化延时：把 SysTick 配置为 1ms 中断
  * @note   SystemCoreClock = 168MHz（在 system_stm32f4xx.c 中定义）
  */
void Delay_Init(void)
{
    /* 启动 DWT 周期计数器，供 delay_us 使用 */
    DWT_Init();

    /* SystemCoreClock/1000 -> 每 1ms 溢出一次，触发 SysTick 异常 */
    SysTick_Config(SystemCoreClock / 1000);
}

/**
  * @brief  阻塞延时（毫秒）
  * @param  ms: 延时毫秒数
  * @note   在中断服务函数里调用会阻塞该中断，教学示例中用于消抖；
  *         实际工程建议改用 g_tick 非阻塞方式。
  */
void delay_ms(uint32_t ms)
{
    TimingDelay = ms;
    while (TimingDelay != 0)
    {
    }
}

/**
  * @brief  SysTick 中断更新函数（由 stm32f4xx_it.c 的 SysTick_Handler 调用）
  */
void SysTick_Update(void)
{
    g_tick++;                       /* 系统节拍 +1ms */
    if (TimingDelay != 0)
    {
        TimingDelay--;              /* 阻塞延时倒计时 */
    }
}

/**
  * @brief  微秒级阻塞延时（基于 DWT Cycle Counter，不占用 SysTick）
  * @param  us: 延时微秒数
  * @note   SystemCoreClock = 168MHz，最大单次延时约 25s（uint32_t 溢出前）
  */
void delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT_CYCCNT - start) < ticks)
    {
    }
}
