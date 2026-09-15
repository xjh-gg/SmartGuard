/**
  ******************************************************************************
  * @file    scheduler.h
  * @brief   时间片轮询调度器（非抢占式）—— 移植自课程示例工程 21调度器
  *
  * 原理：任务表统一管理所有周期任务，记录任务函数、执行周期、上次运行时间戳；
  *       主循环扫描任务表，到期执行；空闲时 __WFI 休眠，任意中断（含 SysTick
  *       1ms）自动唤醒。
  *
  * 与旧 loop_tick % N 写法的区别：周期基于真实毫秒节拍 g_tick，
  * 不受单圈任务耗时影响（消除旧主循环约 16% 的时漂）。
  ******************************************************************************
  */
#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "stm32f4xx.h"

/* 最大支持的任务数量 */
#define SCHED_TASK_MAX_NUM      10U

/* 任务函数指针类型 */
typedef void (*task_func_t)(void);

/* 任务控制块 */
typedef struct
{
    task_func_t func;        /* 任务函数入口 */
    uint32_t    period_ms;   /* 任务执行周期(ms)，0=每轮循环必执行 */
    uint32_t    last_ms;     /* 任务上次执行的时间戳 */
} task_t;

/* 调度器初始化，清空任务表 */
void scheduler_init(void);
/* 注册周期任务：0=成功，-1=失败（函数为空或任务表已满） */
int  scheduler_add_task(task_func_t func, uint32_t period_ms);
/* 启动调度器（永不返回，系统核心） */
void scheduler_run(void);

#endif /* __SCHEDULER_H */
