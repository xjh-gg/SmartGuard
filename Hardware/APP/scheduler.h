/**
  ******************************************************************************
  * @file    scheduler.h
  * @brief   协作式（非抢占）任务调度器 —— 对齐课程模板 sch_core，保留本项目增强
  *
  * 原理（与课程模板一致）：任务表统一管理所有周期任务，记录任务函数、执行周期、
  *       上次运行时间戳；主循环扫描任务表，到期执行；任务执行期间不会被其他
  *       任务打断，必须自己跑完后才把 CPU 让给下一个任务（协作式/非抢占）。
  *
  * 与课程模板 sch_core 的三处有意差异（均为增强，注释见 .c）：
  *   1. 时间源复用本项目的 g_tick（delay.c，SysTick 1ms），语义与模板的
  *      SysTick_GetTick() 相同，避免两套节拍计数器并存；
  *   2. 保留 __WFI 空闲休眠：所有任务扫描完毕后 CPU 休眠，任意中断
  *      （含 SysTick 1ms）自动唤醒，J-Link 单步调试若受影响注释该行即可；
  *   3. 允许 0 周期任务（每轮循环必执行）：本项目 task_fast 承担 MQTT 下行
  *      处理 + 按键扫描 + IWDG 喂狗，必须每轮运行。模板禁止 0 周期是防止
  *      学生把协作式调度退化成"死循环占满 CPU"，本项目的 0 周期任务只做
  *      微秒级轮询，且循环末尾有 __WFI 让出 CPU，风险可控。
  *
  * 新增任务步骤（无需改本文件，参考课程模板 task_list 的用法）：
  *   1. 在 main.c（或业务文件）编写 void 任务函数（无参数）；
  *   2. 在 s_task_table 注册表中增加一项：函数、周期、默认使能、名称。
  ******************************************************************************
  */
#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "stm32f4xx.h"

/* 最大支持的任务数量（对齐模板 SCH_MAX_TASKS） */
#define SCHED_TASK_MAX_NUM      16U

/* 任务使能标志：1 参与调度，0 被调度器跳过（对齐模板 SCH_TASK_ENABLE/DISABLE） */
#define SCHED_TASK_ENABLE       1U
#define SCHED_TASK_DISABLE      0U

/* 任务函数指针类型：无参数、无返回值 */
typedef void (*task_func_t)(void);

/* 任务控制块 TCB：静态数组管理，不用链表，便于单步调试 */
typedef struct
{
    task_func_t func;         /* 任务执行入口 */
    uint32_t    period_ms;    /* 任务执行周期(ms)，0=每轮循环必执行 */
    uint32_t    last_run_ms;  /* 任务上次执行的时间戳 */
    uint8_t     enable;       /* 使能标志 */
    const char *name;         /* 任务名称，串口调试/动态启停用，可为空 */
} task_t;

/* 任务注册表项（业务层批量注册时使用） */
typedef struct
{
    task_func_t func;         /* 任务函数 */
    uint32_t    period_ms;    /* 运行周期(ms) */
    uint8_t     enable;       /* 默认是否使能 */
    const char *name;         /* 任务名称 */
} sched_task_reg_t;

/* 调度器初始化，清空任务表 */
void    scheduler_init(void);
/* 注册周期任务（默认使能、无名称）：0=成功，-1=失败（函数为空或任务表已满） */
int     scheduler_add_task(task_func_t func, uint32_t period_ms);
/* 注册周期任务（完整参数）：0=成功，-1=失败 */
int     scheduler_add_task_ex(task_func_t func, uint32_t period_ms,
                              uint8_t enable, const char *name);
/* 按注册表批量注册：返回实际成功注册的数量 */
uint8_t scheduler_register_table(const sched_task_reg_t *table, uint8_t count);
/* 按名称开启/关闭指定任务：0=成功，-1=未找到 */
int     scheduler_enable_task(const char *name);
int     scheduler_disable_task(const char *name);
/* 按名称查找任务控制块（只读调试用）：找不到返回 0 */
task_t *scheduler_find_task(const char *name);
/* 启动调度器（永不返回，系统核心） */
void    scheduler_run(void);

#endif /* __SCHEDULER_H */
