# FreeRTOS 任务设计

> 代码入口：`User/main.c`（任务壳、同步对象、钩子）、`FreeRTOS/FreeRTOSConfig.h`（内核配置）。
> 裸机实现：`Hardware/APP/scheduler.c`，通过 `USE_FREERTOS=0` 回退启用。

---

## 1. 内核配置

| 配置项 | 值 | 说明 |
|---|---|---|
| 内核版本 | FreeRTOS V10.5.1 | `FreeRTOS/tasks.c` 头部版本号 |
| Port | `portable/RVDS/ARM_CM4F` | Cortex-M4F，使用 FPU 上下文 |
| 调度方式 | `configUSE_PREEMPTION = 1` | 抢占式；同优先级时间片轮转 |
| Tick | `configTICK_RATE_HZ = 1000` | 1 tick = 1 ms，与 SysTick 时基对齐 |
| 优先级数 | `configMAX_PRIORITIES = 7` | 0（idle）~ 6 |
| 堆 | `heap_4`，`configTOTAL_HEAP_SIZE = 40 KB` | 动态创建，不使用静态分配 |
| 栈溢出检测 | `configCHECK_FOR_STACK_OVERFLOW = 2` | 方法 2（最严格） |
| 软件定时器 | `configUSE_TIMERS = 0` | 本项目未使用 |
| 空闲钩子 | `configUSE_IDLE_HOOK = 1` | `vApplicationIdleHook()` 执行 `__WFI()` |
| 中断优先级 | `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5` | NVIC 分组 4 |
| 断言 | `configASSERT` 停机 | 非法调用时关中断死循环，便于调试器定位 |

> `configUSE_TICK_HOOK = 0`：`g_tick` 仍由 `stm32f4xx_it.c` 的 `SysTick_Handler` 维护，调度器启动后该中断同时调用 `xPortSysTickHandler()`。

---

## 2. 任务清单

| 任务 | 优先级 | 栈（word / 字节） | 周期 | 入口函数 | 职责 |
|---|---|---|---|---|---|
| `TaskFast` | 4 | 512 / 2 KB | 50 ms | `TaskFast()` | 按键扫描 `HandleBoardKey()` + `IWDG_Feed()` |
| `TaskTamper` | 3 | 320 / 1.25 KB | 200 ms | `TaskTamper()` | `TamperUpdate()`：MPU6050 防拆采样（5 Hz） |
| `TaskDecision` | 2 | 640 / 2.5 KB | 1000 ms | `TaskDecision()` | `SampleAllSensors()` + `DecisionEngine_Run()` + 本地告警执行 |
| `TaskComm` | 2 | 1024 / 4 KB | 50 ms 轮询 | `TaskComm()` | MQTT 收包、事件队列消费、遥测 / 心跳 / 健康检查、CAM 抓拍 |
| `TaskDisplay` | 1 | 640 / 2.5 KB | 500 ms | `TaskDisplay()` | RTC 刷新、OLED 主页 / 告警页、配置落盘 |

**优先级分配理由**

- `fast` 最高：按键响应与喂狗必须不被网络阻塞影响；
- `tamper` 次高：防拆是安全事件，5 Hz 采样需要稳定节拍；
- `decision` 与 `comm` 同为 2：决策需要网络上报，但两者不能同时操作串口；同优先级时间片轮转让二者公平推进；
- `display` 最低：OLED 整屏刷新约 50 ms，放在最低优先级避免拖慢决策与通信。

> 代码中 `PRIO_DECISION` 上方的注释写作"采集决策 2s"，实际任务周期为 `vTaskDelay(pdMS_TO_TICKS(1000))`，即 1 s；以代码为准。

---

## 3. 任务间通信

### 3.1 事件队列

```c
typedef enum {
    EVT_KEY_MODE = 1,   /* 物理按键切换模式，value = Guard_Mode_t */
    EVT_ALERT    = 2    /* 告警事件，value = Alert_Type_t */
} comm_evt_type_t;

typedef struct {
    uint8_t type;
    uint8_t value;
} comm_evt_t;

static QueueHandle_t s_comm_queue;   /* xQueueCreate(8, sizeof(comm_evt_t)) */
```

| 生产者 | 事件 | 消费者 | 消费者动作 |
|---|---|---|---|
| `TaskFast`（按键） | `EVT_KEY_MODE` | `TaskComm` | `MQTT_ReportModeStatus()` 补报 retain 模式 |
| `TaskDecision`（告警） | `EVT_ALERT` | `TaskComm` | MQTT 告警上报；入侵时触发 CAM 抓拍并发布图片 URL |

- 队列深度 8，元素 2 字节；`xQueueSend(..., 0)` 非阻塞，队列满时直接丢弃；
- 丢弃是可接受的：告警有 5 s 冷却与状态翻转判定，模式状态有 retain 最终一致兜底；
- `TaskComm` 每轮 `while (xQueueReceive(..., 0))` 一次性抽空队列。

### 3.2 互斥锁

| 锁 | 保护对象 | 使用点 |
|---|---|---|
| `s_i2c_mutex` | PB8/PB9 软件 I2C 总线 | `TaskTamper`（MPU6050）与 `TaskDecision` / `TaskDisplay`（AT24C02 配置读写） |
| `s_oled_mutex` | OLED 软件 SPI | `TaskDisplay` 周期刷新与 `TaskFast` 按键即时重绘 |

两把锁在 `main()` 中、调度器启动前创建；创建失败时打印 `[RTOS] kernel object create FAIL` 并停机。

### 3.3 USART 访问收敛

| 串口 | 外设 | 访问者 |
|---|---|---|
| USART1 | 调试日志 | 所有任务（内部 TX 锁保护） |
| USART2 | ESP32-CAM | 仅 `TaskComm` |
| USART3 | ESP8266 | 仅 `TaskComm`（初始化阶段除外） |

抢占式调度下，若允许多个任务同时操作 USART3，AT 指令与应答会交错。因此所有网络相关操作（MQTT 收包、上报、心跳、健康检查、抓拍）都收敛到 `TaskComm` 单任务执行。

---

## 4. 调度时序与阻塞点

```
时间轴（1 s 窗口，示意）
0 ms        50 ms       200 ms      500 ms                 1000 ms
|-----------|-----------|-----------|----------------------|
TaskFast    ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●   （每 50 ms）
TaskTamper      ●                   ●                        （每 200 ms）
TaskDisplay                             ●                    （每 500 ms）
TaskDecision                                                  ●（每 1000 ms）
TaskComm    ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●   （每 50 ms）
```

| 阻塞点 | 最坏耗时 | 影响范围 | 缓解措施 |
|---|---|---|---|
| ESP8266 AT 指令等待 | 数百 ms ~ 数秒 | 仅 `TaskComm` | 看门狗由 `TaskFast` 独立喂；防拆由 `TaskTamper` 继续采样 |
| MQTT 重连 | 可达数十秒 | 仅 `TaskComm` | `ESP8266_SetWaitHook(TamperWaitHook)` 在等待循环续采防拆 |
| CAM 抓拍 | 约 4 s（1 s 探测 + 3 s 等待） | 仅 `TaskComm` | `CAPTURE` 5 s 硬限速；等待循环内喂狗 |
| 基线重学 | 20 s | 仅 `TaskDecision` | 其他任务照常调度；函数内分段 `IWDG_Feed()` |
| 配置扇区擦除 | 约 1 s（内部 Flash 后端） | 仅执行擦除的任务 | 默认使用 AT24C02；内部 Flash 后端擦除前先喂狗 |

> 与裸机协作式调度的关键差异：旧实现中网络阻塞会冻结整个主循环（含喂狗与防拆）；FreeRTOS 下阻塞只影响 `TaskComm` 自身。

---

## 5. 中断与临界区约束

- NVIC 分组为 `NVIC_PriorityGroup_4`（4 位抢占优先级，0 位子优先级）；
- USART1/2/3 中断抢占优先级为 1，高于 `configMAX_SYSCALL_INTERRUPT_PRIORITY = 5`，因此**不得调用任何 FreeRTOS `FromISR` API**；
- USART ISR 只把字节写入环形缓冲区，解析在任务上下文完成；
- `SysTick_Handler` 在调度器启动前只维护 `g_tick`，启动后追加 `xPortSysTickHandler()`；
- `SVC_Handler` / `PendSV_Handler` 由 FreeRTOS port 接管，`stm32f4xx_it.c` 不再定义同名函数。

---

## 6. 钩子函数

| 钩子 | 行为 | 目的 |
|---|---|---|
| `vApplicationIdleHook()` | `__WFI()` | 无就绪任务时休眠，任意中断唤醒；保留裸机低功耗行为 |
| `vApplicationStackOverflowHook()` | `s_rtos_fault = 1`，关中断死循环 | J-Link 附加后读 `s_rtos_fault` 定位 |
| `vApplicationMallocFailedHook()` | `s_rtos_fault = 2`，关中断死循环 | 堆耗尽时停机而非随机崩溃 |

钩子可能在 `PendSV` 上下文被调用，因此只写全局标记，不使用互斥锁或阻塞串口。

---

## 7. 与裸机调度的对应关系

| 裸机任务（`scheduler.c`） | 周期 | FreeRTOS 对应 |
|---|---|---|
| `task_fast` | 0（每轮） | `TaskFast`，50 ms |
| `task_tamper` | 200 ms | `TaskTamper`，200 ms |
| `task_decision` | 2000 ms | `TaskDecision`，1000 ms |
| `task_display` | 1000 ms | `TaskDisplay`，500 ms |
| `task_telemetry` | 10 s | `TaskComm` 内部软周期 |
| `task_ping` | keepalive / 2 = 30 s | `TaskComm` 内部软周期 |
| `task_health` | 300 s | `TaskComm` 内部软周期（5 min） |
| `task_ml_collect` | 2 s | `TaskComm` 内部软周期（`ML_DATA_COLLECT=1` 时） |

回退方式：把 `Template.uvprojx` 的 `USE_FREERTOS` 改为 `0`，重新编译即可；业务函数体共用，不需要改驱动。

---

## 8. 已知限制

- `TaskDecision` 周期为 1 s，但 DHT11 仍按内部计数二分频为 2 s；
- `TaskComm` 的 CAM 抓拍是同步等待，尚未拆成"探测 → 等待 → 取图"的非阻塞状态机；
- 事件队列没有优先级或可靠重传，队列满时丢弃；
- `main.c` 中仍有部分跨任务共享变量使用 `volatile` 而非原子/队列（如 `g_manual_alert`、`g_relearn_request`），依赖单字节访问在 Cortex-M4 上的原子性。
