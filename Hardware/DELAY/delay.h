/**
  ******************************************************************************
  * @file    delay.h
  * @brief   延时驱动（基于 SysTick：g_tick 1ms 节拍 + VAL 毫秒内余量）
  *   提供：
  *     Delay_Init()    启动 SysTick，每 1ms 进一次中断
  *     delay_ms(ms)    毫秒延时（FreeRTOS 下任务内自动委托 vTaskDelay 让出 CPU，
  *                     调度器启动前的初始化阶段退化为 SysTick 微秒时基忙等）
  *     delay_us(us)    微秒级忙等延时（g_tick+VAL 合成时基，不依赖 DWT）
  *     g_tick          全局 1ms 节拍，供非阻塞计时（差值比较），FreeRTOS 下仍有效
  ******************************************************************************
  */
#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f4xx.h"

void Delay_Init(void);              /* 配置 SysTick 为 1ms 中断 */
void delay_ms(uint32_t ms);         /* 阻塞延时，单位 ms */
void delay_us(uint32_t us);         /* 阻塞延时，单位 us（SysTick 微秒时基） */
void SysTick_Update(void);          /* 由 stm32f4xx_it.c 的 SysTick_Handler 调用 */

extern volatile uint32_t g_tick;    /* 系统节拍：每 1ms 自增 1 */

#endif /* __DELAY_H__ */
