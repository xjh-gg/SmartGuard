/*
 * FreeRTOSConfig.h
 * SmartGuard - STM32F407ZE (Cortex-M4F) + Keil MDK ARMCC v5 + 标准外设库
 *
 * 对应官方 port：FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c
 * 节拍 1kHz，堆内存 heap_4，动态创建任务（不使用静态分配）。
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * 基础调度配置
 *----------------------------------------------------------*/
#define configUSE_PREEMPTION                    1       /* 抢占式调度 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1       /* CM4F 前导零指令选任务 */
#define configUSE_TICKLESS_IDLE                 0       /* 不用 tickless，空闲 WFI 由 idle hook 实现 */
#define configCPU_CLOCK_HZ                      168000000UL
#define configTICK_RATE_HZ                      ((TickType_t)1000)  /* 1 tick = 1ms，与旧 g_tick 对齐 */
#define configMAX_PRIORITIES                    7       /* 0(idle) ~ 6 */
#define configMINIMAL_STACK_SIZE                ((unsigned short)128) /* idle/timer 任务栈，单位 word */
#define configTOTAL_HEAP_SIZE                   ((size_t)(40 * 1024)) /* heap_4 静态堆，40KB 放在主 SRAM */
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0       /* 32 位 tick */
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    0

/*-----------------------------------------------------------
 * 同步原语 / 内存分配
 *----------------------------------------------------------*/
#define configUSE_MUTEXES                       1       /* I2C/OLED/日志 互斥锁 */
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0
#define configSTACK_DEPTH_TYPE                  uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

#define configSUPPORT_STATIC_ALLOCATION         0       /* 仅动态分配，免写 vApplicationGetIdleTaskMemory */
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/*-----------------------------------------------------------
 * 钩子函数（实现在 main.c）
 *----------------------------------------------------------*/
#define configUSE_IDLE_HOOK                     1       /* vApplicationIdleHook：__WFI 休眠（保留旧行为） */
#define configUSE_TICK_HOOK                     0       /* g_tick 仍由 stm32f4xx_it.c 的 SysTick_Handler 维护 */
#define configCHECK_FOR_STACK_OVERFLOW          2       /* 栈溢出检测方法 2（最严格） */
#define configUSE_MALLOC_FAILED_HOOK            1

/*-----------------------------------------------------------
 * 软件定时器（本项目不用，关闭以裁剪体积）
 *----------------------------------------------------------*/
#define configUSE_TIMERS                        0

/*-----------------------------------------------------------
 * Cortex-M4 中断优先级
 *   STM32F4 用 4 位优先级（高 4 位有效）；
 *   main() 中已把优先级分组改为 NVIC_PriorityGroup_4（4 位抢占，0 位子优先级）。
 *   USART1/2/3 ISR 抢占优先级=1（数值越小越高），不调用任何 FromISR API，
 *   故可以处于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 之上。
 *----------------------------------------------------------*/
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS                             __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS                             4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*-----------------------------------------------------------
 * 异常向量名映射
 *   SVC/PendSV 完全交给 FreeRTOS（stm32f4xx_it.c 不再定义同名函数）。
 *   SysTick 不做映射：本项目保留自己的 SysTick_Handler，调度器启动前只维护
 *   g_tick，启动后在其中调用 xPortSysTickHandler()（extern 声明见 it.c）。
 *----------------------------------------------------------*/
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler

/*-----------------------------------------------------------
 * 可选 API
 *----------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          0

/*-----------------------------------------------------------
 * 断言：移植/调试期捕获非法调用（如中断优先级配置错误）。
 * 正式发布时可改为空宏以缩减代码。
 *----------------------------------------------------------*/
#define configASSERT(x) \
    if ((x) == 0) { taskDISABLE_INTERRUPTS(); for (;;) { } }

#endif /* FREERTOS_CONFIG_H */
