/**
  ******************************************************************************
  * @file    delay.c
  * @brief   延时驱动实现：SysTick 1ms 中断 + 微秒级时基 + 阻塞延时
  *
  * 原理：
  *   SysTick_Config(SystemCoreClock / 1000) -> SysTick 每 1ms 触发一次中断，
  *   中断里把 g_tick 自增（非阻塞计时用），并递减 TimingDelay（阻塞延时用）。
  *
  *   微秒级时基 = g_tick（整毫秒）+ SysTick 当前倒计数值 VAL（毫秒内余量），
  *   完全不依赖 DWT。实测部分 STM32F4 板卡经老旧调试器反复烧录后 DWT 的
  *   CYCCNT 会失准/停走（必须彻底断电才能恢复），而 SysTick 始终由 HCLK
  *   独立驱动，时序可靠。
  ******************************************************************************
  */
#include "delay.h"

#if USE_FREERTOS
/* FreeRTOS 接管调度后，任务内毫秒延时必须让出 CPU，不能再忙等（否则低优先级
   任务无法运行，协作式忙等也会白白烧 CPU）。调度器启动前的初始化阶段（ESP8266
   握手/基线学习都在 vTaskStartScheduler 之前）还没有任务可调度，此时用
   SysTick 微秒时基忙等——SysTick 中断仍维护 g_tick。 */
#include "FreeRTOS.h"
#include "task.h"
#endif

#if !USE_FREERTOS
/* 阻塞延时的倒计时值（SysTick_Update 里递减，为 0 时延时结束） */
static volatile uint32_t TimingDelay = 0;
#endif

/* 全局 1ms 节拍：任务/中断均可用 g_tick 差值做非阻塞定时。
   FreeRTOS 构建时仍由 stm32f4xx_it.c 的 SysTick_Handler 每 1ms 自增，
   与 FreeRTOS 自己的 xTickCount（同源于同一个 SysTick 中断）并行存在。 */
volatile uint32_t g_tick = 0;

/* SysTick 寄存器（Cortex-M4 内核定时器，HCLK 驱动，不受调试器/DWT 影响）
   注意 SysTick 挂在 SCS 空间，基址 0xE000E000（不是 0xE0000000） */
#define SYSTICK_CTRL    (*(volatile uint32_t *)0xE000E010)
#define SYSTICK_LOAD    (*(volatile uint32_t *)0xE000E014)
#define SYSTICK_VAL     (*(volatile uint32_t *)0xE000E018)

/**
  * @brief  读取当前微秒时基（g_tick 整毫秒 + SysTick VAL 毫秒内余量）
  * @retval 开机以来的微秒数（uint32 约 71.6 分钟回绕，差值比较不受影响）
  * @note   两次读 g_tick 夹住 VAL，处理恰好跨 SysTick 重装载边界的情况：
  *         若两次读数之间发生过中断重装载，则用新的 g_tick 重读 VAL。
  */
static uint32_t micros_now(void)
{
    uint32_t t1;
    uint32_t t2;
    uint32_t v;
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    t1 = g_tick;
    v  = SYSTICK_VAL;
    t2 = g_tick;
    if (t1 != t2)
    {
        t1 = t2;
        v  = SYSTICK_VAL;
    }

    /* LOAD+1 个 HCLK 周期为 1ms；VAL 向下计数，余量 = LOAD - VAL */
    return (t1 * 1000U) + ((SYSTICK_LOAD - v) / cycles_per_us);
}

/**
  * @brief  初始化延时：把 SysTick 配置为 1ms 中断
  * @note   SystemCoreClock = 168MHz（在 system_stm32f4xx.c 中定义）。
  *         FreeRTOS 构建时这里照常启动 SysTick（上电初始化阶段的 g_tick 时基），
  *         之后 vPortSetupTimerInterrupt() 会以相同的 1ms 周期重配一次 SysTick，
  *         中断服务仍走本项目自己的 SysTick_Handler（见 stm32f4xx_it.c）。
  */
void Delay_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

/**
  * @brief  阻塞延时（毫秒）
  * @param  ms: 延时毫秒数
  * @note   FreeRTOS 构建：调度器运行后委托 vTaskDelay() 让出 CPU（其他任务可
  *         正常抢占执行）；调度器启动前（上电初始化序列）用 SysTick 微秒时基
  *         忙等（误差 <0.1ms）。裸机构建：沿用 TimingDelay 倒计时（中断递减）。
  *         任何情况下都不要在中断 ISR 里调用本函数。
  */
void delay_ms(uint32_t ms)
{
#if USE_FREERTOS
    uint32_t start;
    uint32_t wait_us;

    if (ms == 0U)
    {
        return;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        /* 调度器启动前：SysTick 微秒时基忙等，不依赖任何 OS 服务，
           也不受 DWT 失准影响（实测老旧 J-Link OB 烧录后 DWT 会停走） */
        start   = micros_now();
        wait_us = ms * 1000U;
        while ((micros_now() - start) < wait_us)
        {
        }
    }
#else
    TimingDelay = ms;
    while (TimingDelay != 0)
    {
    }
#endif
}

/**
  * @brief  SysTick 中断更新函数（由 stm32f4xx_it.c 的 SysTick_Handler 调用）
  */
void SysTick_Update(void)
{
    g_tick++;                       /* 系统节拍 +1ms（FreeRTOS 下同样保留） */
#if !USE_FREERTOS
    if (TimingDelay != 0)
    {
        TimingDelay--;              /* 阻塞延时倒计时 */
    }
#endif
}

/**
  * @brief  微秒级阻塞延时（基于 SysTick g_tick+VAL 合成微秒时基）
  * @param  us: 延时微秒数
  * @note   不使用 DWT，分辨率约 1us，因此 I2C/DHT11/OLED 等位操作时序在任何
  *         调试器连接状态下都稳定；实际值可能比标称长 0~2us（忙等轮询粒度），
  *         对位操作只有放宽建立/保持时间的作用。
  */
void delay_us(uint32_t us)
{
    uint32_t start = micros_now();

    while ((micros_now() - start) < us)
    {
    }
}
