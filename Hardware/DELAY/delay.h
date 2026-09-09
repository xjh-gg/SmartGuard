/**
  ******************************************************************************
  * @file    delay.h
  * @brief   延时驱动（基于 SysTick 1ms 中断）
  *   提供：
  *     Delay_Init()    启动 SysTick，每 1ms 进一次中断
  *     delay_ms(ms)    阻塞延时（毫秒）
  *     g_tick          全局 1ms 节拍，供主循环做非阻塞计时（差值比较）
  ******************************************************************************
  */
#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f4xx.h"

void Delay_Init(void);              /* 配置 SysTick 为 1ms 中断，并启动 DWT */
void delay_ms(uint32_t ms);         /* 阻塞延时，单位 ms */
void delay_us(uint32_t us);         /* 阻塞延时，单位 us（基于 DWT） */
void SysTick_Update(void);          /* 由 stm32f4xx_it.c 的 SysTick_Handler 调用 */

extern volatile uint32_t g_tick;    /* 系统节拍：每 1ms 自增 1 */

#endif /* __DELAY_H__ */
