/**
  ******************************************************************************
  * @file    scheduler.c
  * @brief   协作式（非抢占）任务调度器实现 —— 对齐课程模板 sch_core
  *
  * 时间判断用无符号减法：即使 g_tick 溢出回绕，(now - last) 仍然正确。
  * 先更新时间戳再执行任务：即使任务耗时较长，下一次仍按"理想周期"对齐，
  * 而不是再叠一次耗时（跳帧语义，与模板一致）。
  ******************************************************************************
  */
#include "scheduler.h"
#include "delay.h"   /* g_tick：本项目 SysTick 1ms 节拍，语义同模板的 SysTick_GetTick() */
#include <string.h>  /* strcmp：按名称查找任务 */

/* 任务数组：静态数组代替链表，遍历逻辑一眼能看懂 */
static task_t  s_task_table[SCHED_TASK_MAX_NUM];
/* 当前已注册任务个数 */
static uint8_t s_task_cnt = 0U;

/**
  * @brief  调度器初始化：先清零全部 TCB，避免上电 RAM 脏数据被当成有效任务
  */
void scheduler_init(void)
{
    uint8_t i;

    for (i = 0; i < SCHED_TASK_MAX_NUM; i++)
    {
        s_task_table[i].func        = (task_func_t)0;
        s_task_table[i].period_ms   = 0U;
        s_task_table[i].last_run_ms = 0U;
        s_task_table[i].enable      = SCHED_TASK_DISABLE;
        s_task_table[i].name        = (const char *)0;
    }
    s_task_cnt = 0U;
}

/**
  * @brief  注册周期任务（完整参数）
  * @param  func:      任务函数
  * @param  period_ms: 执行周期(ms)，0 表示每轮循环都执行
  * @param  enable:    SCHED_TASK_ENABLE / SCHED_TASK_DISABLE
  * @param  name:      任务名称（可为 0，仅调试/动态启停用）
  * @retval 0=注册成功，-1=注册失败
  * @note   与模板差异：允许 period_ms=0。模板禁止 0 是防止学生把协作式调度
  *         退化成死循环占满 CPU；本项目 0 周期仅用于 task_fast（MQTT 下行 +
  *         按键 + IWDG 喂狗，微秒级轮询），且循环末尾有 __WFI 让出 CPU。
  */
int scheduler_add_task_ex(task_func_t func, uint32_t period_ms,
                          uint8_t enable, const char *name)
{
    task_t *tcb;

    if (func == (task_func_t)0)            return -1;   /* 函数指针为空，调度时会跳进 0 地址 */
    if (s_task_cnt >= SCHED_TASK_MAX_NUM)  return -1;   /* 任务表已满 */

    tcb = &s_task_table[s_task_cnt];
    tcb->func      = func;
    tcb->period_ms = period_ms;
    tcb->enable    = enable;
    tcb->name      = name;

    /* last_run 记成当前节拍：注册后先等一个完整周期再执行，避免上电瞬间所有任务挤在一起 */
    tcb->last_run_ms = g_tick;

    s_task_cnt++;
    return 0;
}

/**
  * @brief  注册周期任务（兼容旧接口：默认使能、无名称）
  */
int scheduler_add_task(task_func_t func, uint32_t period_ms)
{
    return scheduler_add_task_ex(func, period_ms, SCHED_TASK_ENABLE, (const char *)0);
}

/**
  * @brief  按注册表批量注册任务
  * @param  table: 注册表首地址
  * @param  count: 表项数量
  * @retval 实际成功注册的数量
  */
uint8_t scheduler_register_table(const sched_task_reg_t *table, uint8_t count)
{
    uint8_t i;
    uint8_t ok = 0U;

    if (table == (const sched_task_reg_t *)0)
    {
        return 0U;
    }

    for (i = 0; i < count; i++)
    {
        if (scheduler_add_task_ex(table[i].func, table[i].period_ms,
                                  table[i].enable, table[i].name) == 0)
        {
            ok++;
        }
    }
    return ok;
}

/**
  * @brief  按名称查找任务控制块
  * @param  name: 任务名称（需与注册时完全一致）
  * @retval TCB 指针，找不到返回 0
  */
task_t *scheduler_find_task(const char *name)
{
    uint8_t i;

    if (name == (const char *)0)
    {
        return (task_t *)0;
    }

    for (i = 0; i < s_task_cnt; i++)
    {
        if (s_task_table[i].name != (const char *)0)
        {
            if (strcmp(s_task_table[i].name, name) == 0)
            {
                return &s_task_table[i];
            }
        }
    }
    return (task_t *)0;
}

/**
  * @brief  按名称开启指定任务
  * @param  name: 任务名称
  * @retval 0=成功，-1=未找到
  */
int scheduler_enable_task(const char *name)
{
    task_t *tcb = scheduler_find_task(name);

    if (tcb == (task_t *)0)
    {
        return -1;
    }

    tcb->enable = SCHED_TASK_ENABLE;
    /* 重新打开时刷新时间戳，避免"关闭很久后一打开立刻连跑多次"的错觉 */
    tcb->last_run_ms = g_tick;
    return 0;
}

/**
  * @brief  按名称关闭指定任务（不删除 TCB，调度器直接跳过）
  * @param  name: 任务名称
  * @retval 0=成功，-1=未找到
  */
int scheduler_disable_task(const char *name)
{
    task_t *tcb = scheduler_find_task(name);

    if (tcb == (task_t *)0)
    {
        return -1;
    }

    tcb->enable = SCHED_TASK_DISABLE;
    return 0;
}

/**
  * @brief  调度器主循环（系统核心，永不返回）
  * @note   协作式：这里是普通函数调用，不是 PendSV 上下文切换。当前任务
  *         func() 返回前，后面的任务绝对不会运行，这就是"非抢占"。
  *         函数返回 = 主动让出 CPU，随后才判断下一个任务。
  */
void scheduler_run(void)
{
    uint32_t now;
    uint8_t  i;
    task_t  *tcb;

    for (;;)
    {
        now = g_tick;                       /* 1、获取当前系统时间 */

        for (i = 0; i < s_task_cnt; i++)    /* 2、遍历所有已注册任务 */
        {
            tcb = &s_task_table[i];

            /* 未使能的任务直接跳过，实现动态启停，无需把任务从数组删除 */
            if (tcb->enable != SCHED_TASK_ENABLE)
            {
                continue;
            }

            if (tcb->func == (task_func_t)0)
            {
                continue;
            }

            if (tcb->period_ms == 0U)           /* 0 周期任务：每轮循环强制执行 */
            {
                tcb->func();
                tcb->last_run_ms = now;
            }
            else if ((now - tcb->last_run_ms) >= tcb->period_ms)
            {
                /* 未到周期则不执行，保证任务按配置频率运行 */
                tcb->last_run_ms = now;
                tcb->func();
            }
        }

        __WFI();    /* 3、所有任务执行完毕，CPU 休眠；任意中断（含 SysTick 1ms）唤醒。
                       J-Link 单步调试若受影响，注释掉本行即可 */
    }
}
