/**
  ******************************************************************************
  * @file    scheduler.c
  * @brief   时间片轮询调度器实现 —— 移植自课程示例工程 21调度器
  *
  * 与示例的差异：
  *   - 时间源用本项目的 g_tick（delay.c，SysTick 1ms），语义与
  *     示例的 SysTick_GetTick() 相同
  *   - __WFI 保留：J-Link 单步调试若受影响，注释掉函数末尾那一行即可
  ******************************************************************************
  */
#include "scheduler.h"
#include "delay.h"   /* g_tick */

/* 任务表 */
static task_t  s_task_table[SCHED_TASK_MAX_NUM];
/* 已注册的合法任务数量 */
static uint8_t s_task_cnt = 0U;

/**
  * @brief  调度器初始化，清空任务表
  */
void scheduler_init(void)
{
    uint8_t i;

    for (i = 0; i < SCHED_TASK_MAX_NUM; i++)
    {
        s_task_table[i].func      = (task_func_t)0;
        s_task_table[i].period_ms = 0U;
        s_task_table[i].last_ms   = 0U;
    }
    s_task_cnt = 0U;
}

/**
  * @brief  注册周期任务
  * @param  func:      任务函数
  * @param  period_ms: 执行周期(ms)，0 表示每轮循环都执行
  * @retval 0=注册成功，-1=注册失败
  */
int scheduler_add_task(task_func_t func, uint32_t period_ms)
{
    if (func == (task_func_t)0)                return -1;   /* 无效任务 */
    if (s_task_cnt >= SCHED_TASK_MAX_NUM)      return -1;   /* 任务表已满 */

    s_task_table[s_task_cnt].func      = func;
    s_task_table[s_task_cnt].period_ms = period_ms;
    s_task_table[s_task_cnt].last_ms   = g_tick;
    s_task_cnt++;

    return 0;
}

/**
  * @brief  调度器主循环（系统核心，永不返回）
  * @note   非抢占式：任务按注册顺序依次执行，单个任务执行期间后面的任务必须等待
  */
void scheduler_run(void)
{
    uint32_t now;
    uint8_t  i;

    for (;;)
    {
        now = g_tick;                       /* 1、获取当前系统时间 */

        for (i = 0; i < s_task_cnt; i++)    /* 2、遍历所有已注册任务 */
        {
            if (s_task_table[i].func == (task_func_t)0)
                continue;

            if (s_task_table[i].period_ms == 0U)        /* 0 周期任务：每轮循环强制执行 */
            {
                s_task_table[i].func();
                s_task_table[i].last_ms = now;
            }
            else if ((now - s_task_table[i].last_ms) >= s_task_table[i].period_ms)
            {
                s_task_table[i].last_ms = now;          /* 时间到期：执行周期任务（跳帧语义） */
                s_task_table[i].func();
            }
        }

        __WFI();    /* 3、所有任务执行完毕，CPU 休眠；任意中断（含 SysTick 1ms）唤醒。
                       J-Link 单步调试若受影响，注释掉本行即可 */
    }
}
