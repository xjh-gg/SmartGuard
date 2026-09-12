# SmartGuard Agent · 智能空间守护 Agent

> 基于 STM32F407ZE + 多传感器融合 + 端侧自适应决策的智能空间守护 Agent

- **目标**：在单片机上实现"感知 → 决策 → 告警 → 远程推送"的完整守护闭环

- **定位**：端侧轻决策 + 云端/边缘重 AI 的协作架构

- **芯片**：STM32F407ZE（ARM Cortex-M4 · 168MHz · 192KB RAM · 1MB Flash · 32位双精度 FPU）

- **当前版本**：**v0.6** — 裸机采集 + 自适应阈值（基线 20s + EWMA）+ 矩阵键盘切模式 + ESP8266 UDP 推送 + 5s 冷却防刷屏 + PowerShell Web Dashboard

***

## 目录结构

```
SmartGuard/
├── Core/                   # Cortex-M4 核心文件 + 启动文件
│   ├── startup_stm32f40_41xxx.s       # 汇编启动，设置向量表
│   └── stm32f4xx.h / stm32f4xx_conf.h # 标准库头 + 外设使能配置
├── Library/                # STM32F4xx 标准外设库（ST 官方）
│   ├── inc/                #   stm32f4xx_gpio.h / stm32f4xx_adc.h / ...
│   └── src/                #   stm32f4xx_gpio.c / stm32f4xx_adc.c / ...
├── Hardware/               # 硬件驱动层（每个模块一个文件夹）
│   ├── APP/                # ★ 应用层：在驱动层之上组合业务逻辑
│   │   ├── adaptive_threshold.c/h   # 基线学习 + EWMA 动态阈值
│   │   ├── decision_engine.c/h      # 模式状态机 + 告警触发 + ESP8266 宏开关
│   │   └── sensor_hub.c/h           # 统一采样入口 + 数据聚合结构体
│   ├── ADC/                # 光敏 PF7 + MQ-2 PA6 多通道 ADC
│   ├── BEEP/               # TIM13 PWM 蜂鸣器（PF8）
│   ├── DELAY/              # SysTick 1ms 中断 + g_tick 全局节拍
│   ├── DHT11/              # PG9 单总线温湿度（未接线时自动跳过）
│   ├── ESP8266/            # USART3 AT 指令 + UDP 上报（PB10/PB11）
│   ├── HCSR04/             # Trig=PC8 / Echo=PC6 超声波
│   ├── KEY/                # 独立按键（板载，预留）
│   ├── LED/                # 4 颗 LED（PF9/PF10/PE13/PE14）
│   ├── MATRIX_KEY/         # 4×4 矩阵键盘（列扫行读）
│   ├── MQ2/                # PA6 ADC1_IN6 烟雾（未接线时自动跳过）
│   ├── OLED/               # SSD1306 软件 SPI（PB12-PB15）
│   ├── PWM/                # TIM1/TIM14 PWM 调光
│   ├── SERVO/              # SG90 舵机（空实现，后续补）
│   └── USART/              # USART1/2/3 初始化 + Printf 重定向
├── User/                   # 主程序 + Keil 工程文件
│   ├── main.c              # ★ 入口函数：硬件初始化 → 基线学习 → 主循环
│   ├── Template.uvprojx    # Keil 工程文件
│   ├── Template.uvoptx     # Keil 选项缓存（删它能解决路径乱的问题！）
│   ├── stm32f4xx_it.c / h  # 中断处理：SysTick → g_tick++
│   └── system_stm32f4xx.c  # 时钟配置（168MHz PLL）
├── web_dashboard.ps1       # ★ PowerShell 本地 UDP 监听 + HTTP Web 面板
├── Object/                 # Keil 编译输出（.o / .d / .axf / .hex）
└── README.md               # ← 你正在看的这个文件
```

***

## 硬件连接总览

### 已接入并验证的模块 ✅

| 模块               | 协议        | 引脚                                                                 | 关键参数                                    | 验证状态                         |
| ---------------- | --------- | ------------------------------------------------------------------ | --------------------------------------- | ---------------------------- |
| **光敏电阻**         | ADC       | PF7 → ADC3\_IN5                                                    | 0\~4095，光照越强值越小（遮光/夜晚变大） | ✅ 基线学习数据稳定（sigma=14）         |
| **HC-SR04 超声波**  | GPIO      | Trig=PC8（输出）Echo=PC6（输入+中断）                                        | VCC **必须 5V**，3.3V 供电无回波                | ✅ 5.0cm 近距离检测正常              |
| **ESP8266 WiFi** | USART3    | PB10=TX → ESP8266 RXPB11=RX ← ESP8266 TX                           | 115200 / 8N1，**GND 共地**                 | ✅ WiFi 连上 + UDP 推送成功         |
| **4×4 矩阵键盘**     | GPIO      | **列（输出）**：PB7 / PA4 / PG15 / PC7**行（输入+上拉）**：PC9 / PB6 / PE6 / PA8 | 非阻塞扫描，返回 'A'~~'D','1'~~'9','0','\*','#' | ✅ A=HOME B=AWAY C=SLEEP 正常切换 |
| **蜂鸣器**          | TIM13 PWM | PF8 → TIM13\_CH1                                                   | 50% 占空比方波，板载三极管驱动                       | ✅ 告警时鸣响                      |
| **LED × 4**      | GPIO      | PF9 / PF10 / PE13 / PE14                                           | 低电平点亮（开漏输出）                             | ✅ 入侵=PF10亮，火灾=PF10+PE13亮     |
| **串口调试**         | USART1    | PA9=TX → USB-TTL RXPA10=RX ← USB-TTL TX                            | 115200 / 8N1 / **UTF-8**                | ✅ 完整启动日志正常输出                 |

### 已写驱动但未接线的模块 ⬜

| 模块             | 协议     | 引脚                                        | 备注                                                                               |
| -------------- | ------ | ----------------------------------------- | -------------------------------------------------------------------------------- |
| **MQ-2 烟雾**    | ADC    | PA6 → ADC1\_IN6                           | 模拟 AO 输出，需 5V 供电；未接时代码自动跳过（mq2\_adc 保持 0）                                        |
| **DHT11 温湿度**  | 单总线    | PG9                                       | 需 4.7kΩ 上拉到 3.3V；读取间隔 ≥ 1.5s；未接时 temp=25.0 humi=50.0（默认值）                        |
| **0.96" OLED** | 软件 SPI | CLK=PB0 / MOSI=PF11 / DC=PB1 / CS1=PC13 | ⚠️ **CS2 必须接 VCC** 禁用 GT20L1616 字库芯片；F407 软件 SPI 需 `delay_us(1)` 放慢时序；PC13 需禁用 RTC Tamper；VCC 接 5V |

### 后续扩展（驱动待写）🔮

| 模块                | 协议         | 引脚                                     | 用途            |
| ----------------- | ---------- | -------------------------------------- | ------------- |
| SG90 舵机           | TIM PWM    | PE9（TIM4\_CH4 预留）                      | 超声波云台，扩大扫描范围  |
| W25Q128 SPI Flash | 硬件 SPI     | PA5=SCK / PA6=MISO / PA7=MOSI / CS=PE4 | 掉电保存阈值 + 历史告警 |
| OV7670 摄像头        | DCMI + I2C | PC6-PC11（数据）+ PB6/SCL PB7/SDA          | 帧差法移动检测       |
| LD3320 语音识别       | SPI 或 UART | 待分配                                    | 本地关键词唤醒       |

***

## 软件架构

### 分层图

```
┌─────────────────────────────────────────────────────────────────────────┐
│  应用层（APP）                                                          │
│  ┌────────────────────┐  ┌────────────────────┐  ┌──────────────────┐  │
│  │  adaptive_threshold │  │   decision_engine  │  │    sensor_hub    │  │
│  │                    │  │                    │  │                  │  │
│  │ AT_Init()          │  │ DecisionEngine_    │  │ SensorHub_Init() │  │
│  │ AT_LearnBaseline() │  │   Init()           │  │ SensorHub_       │  │
│  │ AT_Update()        │← │ DecisionEngine_    │← │   Update()       │  │
│  │ AT_CheckFire()     │  │   SetMode()        │  │ SensorHub_GetData│  │
│  │ AT_CheckIntrusion()│  │ DecisionEngine_    │  │                  │  │
│  │                    │  │   Run()            │  │ 返回聚合结构体：  │  │
│  │ 20s 基线采样       │  │                    │  │ light_adc        │  │
│  │ mean + 3σ 阈值     │  │ · CheckFire()      │  │ mq2_adc          │  │
│  │ EWMA 慢跟随        │  │ · CheckIntrusion() │  │ distance         │  │
│  │ 异常拒绝吸收       │  │ · AutoLighting()   │  │ temperature      │  │
│  │                    │  │ · 告警→蜂鸣/LED    │  │ humidity         │  │
│  │                    │  │       → ESP8266    │  │                  │  │
│  └─────────┬──────────┘  └────────┬───────────┘  └────────┬─────────┘  │
│            │                       │                        │            │
├────────────┼───────────────────────┼────────────────────────┼────────────┤
│            ▼                       ▼                        ▼            │
│  驱动层 Hardware/*                                                      │
│  ┌──────┐ ┌──────┐ ┌──────┐ ┌───────┐ ┌──────┐ ┌──────┐ ┌──────────┐   │
│  │ ADC  │ │USART │ │ HCSR04│ │ESP8266│ │ BEEP │ │ LED  │ │MATRIX_KEY│   │
│  │PF7   │ │PA9-11│ │PC6/8 │ │PB10/11│ │ PF8 │ │PF9/10│ │PB7/PA4/..│   │
│  │      │ │      │ │      │ │AT指令 │ │ PWM  │ │PE13/14│ │PC9/PB6/..│   │
│  └──────┘ └──────┘ └──────┘ └───────┘ └──────┘ └──────┘ └──────────┘   │
│  ┌──────┐ ┌──────┐ ┌──────┐ ┌───────┐ ┌──────┐                          │
│  │ DELAY│ │ PWM  │ │OLED  │ │ DHT11 │ │ MQ2  │  ← 可选，未接线跳过       │
│  │SysTick│ │TIM1  │ │SPI   │ │ PG9   │ │ PA6  │                          │
│  └──────┘ └──────┘ └──────┘ └───────┘ └──────┘                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 数据结构 `Sensor_Data_t`（sensor\_hub.h）

```c
typedef struct {
    uint16_t light_adc;     // 光敏 ADC 值（PF7） 范围 0~4095
    uint16_t mq2_adc;       // MQ-2 烟雾 ADC 值（PA6） 范围 0~4095，未接=0
    float    distance;      // 超声波距离（HC-SR04） 单位 cm，无效=0.0
    float    temperature;   // DHT11 温度 单位 °C，未接=25.0
    float    humidity;      // DHT11 湿度 单位 %，未接=50.0
} Sensor_Data_t;
```

### 告警状态枚举（decision\_engine.h）

```c
typedef enum {
    ALERT_NONE = 0,          // 正常
    ALERT_INTRUSION,         // 入侵（距离突变 / 太近）
    ALERT_FIRE,              // 火灾（烟雾 + 温度联合）
    ALERT_ENV_ABNORMAL       // 环境异常（预留：光照突变等）
} Alert_Type_t;

typedef enum {
    MODE_HOME = 0,   // A 键 — 入侵检测关，只开火灾检测
    MODE_AWAY,       // B 键 — 离家，入侵+火灾全开
    MODE_SLEEP       // C 键 — 睡眠，入侵+火灾全开（后续改低亮度）
} Guard_Mode_t;
```

***

## main.c 启动流程逐行解析

### Phase 1 · 基础硬件初始化（行 58-69）

```c
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 中断优先级分组：抢占2位+响应2位
Delay_Init();    // 启动 SysTick 产生 1ms 中断 → 每进一次中断 g_tick++
LED_Init();      // GPIO PF9/PF10/PE13/PE14 配置为推挽输出（低电平亮）
Beep_Init();     // TIM13 配置为 PWM 模式，周期=1000，占空比=0（初始静音）
USART1_Init(115200);  // PA9=TX PA10=RX，115200/8N1
```

**g\_tick 怎么来的**：`stm32f4xx_it.c` 里的 `SysTick_Handler()` 调用 `SysTick_Update()`，后者让 `g_tick` 自增。`ESP8266_SendAlert` 的冷却判断就是用 `g_tick` 的差值。

### Phase 2 · 传感器初始化（行 72-79）

```c
ADC_Hardware_Init();  // ADC3 通道5（PF7光敏）+ ADC1 通道6（PA6 MQ-2）
HCSR04_Init();        // PC8 输出+PC6 输入，开启 PC6 边沿中断测 Echo 高电平时间
PWM_Init(PWM_PERIOD); // TIM14 控制 PF9 调光，TIM1 控制蜂鸣器 PWM
Matrix_Key_Init();    // 列=输出低电平，行=输入上拉，准备扫描
```

### Phase 3 · 可选硬件（行 82-101）

```c
// OLED：宏 USE_OLED=0 → 编译时直接去掉所有 OLED_Init 代码
#if USE_OLED
    OLED_Init();
#else
    USART1_SendString("[SKIP] OLED (not connected)\r\n");
#endif

// ESP8266：USE_ESP8266=1（在 decision_engine.h 里全局定义）
#if USE_ESP8266
    ESP8266_Init();  // → 内部流程见下方"ESP8266 AT 交互时序"
#endif
```

### Phase 4 · APP 层初始化 + 预热（行 104-109）

```c
SensorHub_Init();      // 什么都不做，预留给后续扩展
DecisionEngine_Init(); // s_mode=HOME, s_alert_last=NONE, s_dist_prev=0
AT_Init();             // s_bl.is_learned=0，标记基线未学习

SensorHub_Update();    // 预热一次 HC-SR04（第一次测距可能不准）
```

### Phase 5 · 基线学习（行 112-124）

```c
AT_LearnBaseline();    // ★ 阻塞约 20 秒，每秒采 1 次
```

### Phase 6 · 主循环（行 139-171）

```c
while (1)
{
    // 1. 键盘扫描（非阻塞，每 500ms 扫一次，足够）
    char key = Key_Scan();
    switch (key) {
        case 'A': DecisionEngine_SetMode(MODE_HOME);  break;
        case 'B': DecisionEngine_SetMode(MODE_AWAY);  break;
        case 'C': DecisionEngine_SetMode(MODE_SLEEP); break;
    }

    // 2. 采集所有传感器
    SensorHub_Update();

    // 3. 自适应阈值 EWMA 更新（吸收环境慢变化）
    AT_Update(d->light_adc, d->mq2_adc, d->temperature, d->humidity);

    // 4. 决策引擎（自动照明 → 火灾检测 → 入侵检测 → 执行告警动作）
    DecisionEngine_Run(d);

    delay_ms(500);  // 500ms 循环
}
```

***

## 自适应阈值算法 · 完整源码解析

### 为什么要自适应？

固定阈值（"距离 < 50cm 就告警"）有这些问题：

- **光敏**：白天 ADC=500，晚上可能升到 2500，同一个阈值怎么设？

- **超声波**：房间里放了新家具，背景距离从 200cm 变成 150cm，入侵阈值要跟着调

- **烟雾**：不同房间基线值不同，走廊里油烟大，客厅干净

### 算法参数（adaptive\_threshold.h）

```c
#define AT_LEARN_SECONDS       20    // 上电学习 20 秒
#define AT_EWMA_ALPHA          0.1f  // EWMA 系数（越小越慢跟随）
#define AT_SIGMA_K             3.0f  // 3σ 原则
#define AT_DISTANCE_DELTA_CM   15.0f  // 距离突变判断阈值（cm）
```

### 基线学习阶段（AT\_LearnBaseline）

```
时间轴:  0s    1s    2s   ...  19s   20s
         ↓     ↓     ↓          ↓     ↓
采样:    [1]   [2]   [3]  ...  [19]  [20]
         light light light      light light
         mq2   mq2   mq2        mq2   mq2
         temp  temp  temp       temp  temp
         humi  humi  humi       humi  humi
         
         存到 s_buf_light[0..19] 等环形缓冲区
```

然后计算：

```c
// 1. 均值：所有样本加起来 / 样本数
float m_light = mean_u16(s_buf_light, 20);
// 本次实测 = 2016

// 2. 总体标准差：每个样本与均值差的平方 → 求平均 → 开根号
float sigma_light = sigma_u16(s_buf_light, 20, m_light);
// 本次实测 = 14

// 3. 动态阈值：baseline + 3 * sigma
// 烟雾特殊处理：最少 400（防止环境 sigma 很小时阈值太低）
mq2_fire_threshold = m_mq2 + 3 * mq2_sigma;
if (mq2_fire_threshold < 400) mq2_fire_threshold = 400;
```

**数学公式**：

$$
\mu = \frac{1}{N}\sum\_{i=1}^{N} x\_i
$$

$$
\sigma = \sqrt{\frac{1}{N}\sum\_{i=1}^{N} (x\_i - \mu)^2}
$$

$$
\text{threshold} = \mu + k \cdot \sigma
$$

> 用**总体标准差**（除以 N 而不是 N-1），因为我们关心的是当前这 20 个样本的固有波动范围，不是要估计更大总体。

### 运行阶段 EWMA（AT\_Update）

**EWMA = 指数加权移动平均**，让 baseline 缓慢跟着环境变化移动：

```
新 baseline = 旧 baseline + α × (新采样值 - 旧 baseline)
```

等价于：新值权重 α，旧值权重 1-α。

α=0.1 → 吸收很慢，适合昼夜温度缓慢上升下降这种场景。

**关键设计：大幅偏离拒绝吸收**

```c
// 烟雾：偏离超过 2*sigma 不吸收（可能是真的烟雾！别学进去了）
if (fabsf(diff_mq2) < 2.0f * mq2_sigma) {
    mq2_ewma += 0.1f * diff_mq2;        // 小幅偏离 → 慢慢吸收
    mq2_fire_threshold = mq2_ewma + 3 * mq2_sigma;  // 阈值跟着 ewma 移动
}
// ↑ 如果真的有人点火，mq2 从 0 跳到 2000，差 > 2*sigma，不被吸收
// ↑ 阈值不会被拉高，下一次检测还能正常触发
```

光照特殊：因为昼夜变化大（白天 → 晚上可能差好几倍），所以用 2×α：

```c
// 光照用 2 倍 alpha（昼夜变化快）
light_ewma += (0.1f * 2.0f) * diff_light;
```

### 火灾检测（AT\_CheckFire）

```c
uint8_t AT_CheckFire(const Sensor_Data_t *data) {
    if (mq2_adc < mq2_fire_threshold) return 0;          // 烟雾不够
    if (temperature < temp_ewma + 2*temp_sigma &&
        temperature < 40.0f) return 0;                    // 温度也没异常
    return 1;                                              // 双条件满足 → 火灾！
}
```

**为什么要双条件？** 厨房炒菜烟雾可能很大（mq2\_adc 跳 2000），但温度正常（25°C）→ 不触发。真正火灾一定是烟雾大 + 温度上升。

### 入侵检测（AT\_CheckIntrusion）

```c
uint8_t AT_CheckIntrusion(data, dist_prev, dist_now) {
    // 方式1：距离突变（快变）— 有人快速靠近
    float delta = dist_prev - dist_now;  // 上一次距离 - 这一次
    if (delta > 15.0f && dist_now < 80.0f) return 1;
    // 例：上次 180cm，这次 150cm → delta=30 > 15 → 触发

    // 方式2：绝对阈值兜底 — 一直很近
    if (dist_now > 0.0f && dist_now < 30.0f) return 1;
    return 0;
}
```

***

## 决策引擎 · 状态机详解

### 告警动作表（decision\_engine.c DecisionEngine\_Run）

| 告警类型                 | 蜂鸣器 | LED1 PF10 | LED2 PE13 | PWM PE14 | 串口输出                            | ESP8266 推送                           |
| -------------------- | --- | --------- | --------- | -------- | ------------------------------- | ------------------------------------ |
| **ALERT\_NONE**      | 关   | 灭         | 灭         | 自动照明     | `[OK] 环境正常`                     | ❌ 不发                                 |
| **ALERT\_INTRUSION** | 常鸣  | **亮**     | 灭         | 自动照明     | `[ALERT] 入侵告警！距离=5.0cm`         | `{"type":"alert","msg":"intrusion"}` |
| **ALERT\_FIRE**      | 常鸣  | **亮**     | **亮**     | 自动照明     | `[ALERT] 火灾告警！MQ2=450 温度=45.0C` | `{"type":"alert","msg":"fire"}`      |

### 关键设计：状态变化才触发

```c
// decision_engine.c 行 167
if (alert != s_alert_last) {
    switch (alert) { ... }
    s_alert_last = alert;
}
```

**效果**：手一直放在超声波前面，主循环每 500ms 都会检测到"距离很近"，但只有**第一次**会触发告警动作（蜂鸣器响 + LED 亮 + UDP 发）。手离开后 → `alert=NONE` → 执行恢复动作（蜂鸣器停 + LED 灭 + 打印正常）。手再靠近 → 又触发一次。

### 模式切换时的重置

```c
void DecisionEngine_SetMode(Guard_Mode_t mode) {
    if (mode != s_mode) {
        s_mode = mode;
        Beep_Off();            // 关蜂鸣器
        LED_OFF(1); LED_OFF(2); // 关所有告警灯
        s_alert_last = ALERT_NONE;  // 清告警状态
        s_dist_prev = 0.0f;         // 清距离差分
    }
}
```

**效果**：AWAY 模式下正在告警 → 按 A 切到 HOME → 蜂鸣器立刻停 + 所有告警状态清除。

### ESP8266 5 秒冷却（双重保险）

```c
// esp8266.c
static uint32_t s_last_send_tick = 0;
#define SEND_COOLDOWN_MS  5000

void ESP8266_SendAlert(const char *msg) {
    uint32_t now = g_tick;  // delay.h 的 1ms 全局节拍
    if ((now - s_last_send_tick) < SEND_COOLDOWN_MS) return;  // 冷却中
    s_last_send_tick = now;
    // ... 发送 JSON
}
```

**两层防刷屏**：

1. `decision_engine.c` — 状态变化才调用 SendAlert（正常不会重复调）
2. `esp8266.c` — 5 秒内即使被调也直接 return（保险）

***

## ESP8266 · AT 指令完整交互时序

```
STM32 (USART3)                          ESP8266
     │                                     │
     │─── AT──────────────────────────────>│  // 测试模块是否在线
     │<─────────────────────── OK ────────│
     │                                     │
     │─── AT+CWMODE=1─────────────────────>│  // 设为 STA 模式
     │<─────────────────────── OK ────────│
     │                                     │
     │─── AT+CWJAP="<your-ssid> 0997",──>│  // 连接 WiFi（约 10 秒）
     │    "6Z5{58n8"                       │
     │<─────────────────── WIFI CONNECTED ─│
     │<─────────────────────── OK ────────│
     │                                     │
     │─── AT+CIPMUX=0─────────────────────>│  // 单连接模式
     │<─────────────────────── OK ────────│
     │                                     │
     │─── AT+CIPSTART="UDP",──────────────>│  // 建立 UDP 通道
     │    "192.168.137.1",12345             │  // 目标：电脑 PowerShell 监听
     │<───────── CIPSTART UDP FAIL ────────│  // ← 常见！ESP8266 固件版本差异
     │                                     │    但 UDP 包实际能正常发
     │─── AT+CIPSEND=30───────────────────>│  // 准备发 30 字节
     │<─────────────────────── > ─────────│  // ESP8266 提示可以发了
     │─── {"type":"alert","msg":"intrusion"}──>│  // 发告警 JSON
     │<─────────────────────── OK ────────│
     │                                     │
     │        ... PowerShell 收到 UDP 包，写入 alerts.json ...
```

**CIPSTART 报 FAIL 但能正常收发**：这是 ESP8266 不同固件版本的常见差异。某些固件在 UDP 模式下 CIPSTART 不返回 OK，但通道实际已建立。代码里已经做了容错——即使 CIPSTART 失败也继续尝试发送。

***

## Web Dashboard · 完整数据流

```
┌─────────────────────────────────────────────────────────────────────────┐
│  STM32 (SmartGuard)                                                     │
│                                                                         │
│  HC-SR04 超声波 ──→ 距离 < 30cm 或 突变 > 15cm                          │
│                          ↓                                              │
│  decision_engine.c: CheckIntrusion() → ALERT_INTRUSION                 │
│                          ↓                                              │
│  ESP8266_SendAlert("intrusion")                                         │
│    ├─ 冷却检查（5s 内不重复发） ✅                                       │
│    └─ USART3: AT+CIPSEND + JSON                                         │
│                          ↓                                              │
│  ESP8266 WiFi ──→ 192.168.137.1:12345 （UDP）                            │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Windows PowerShell (web_dashboard.ps1)                                 │
│                                                                         │
│  ① UDP 监听 socket: 0.0.0.0:12345                                       │
│     ↓                                                                   │
│     收到 JSON 包 → 解析 → 追加到 alerts.json                            │
│     alerts.json: [{"ts":"2026-09-01T17:39:17","ip":"192.168.137.96",    │
│                    "msg":"intrusion"}, ...]                             │
│                                                                         │
│  ② HTTP 监听 listener: localhost:8080                                   │
│     ↓                                                                   │
│     浏览器请求 / → 返回 HTML（含 auto-refresh 脚本）                     │
│     浏览器请求 /alerts → 返回 alerts.json                               │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  浏览器 http://localhost:8080                                           │
│                                                                         │
│  +--------------------------------------------------+                   │
│  │ SmartGuard Dashboard  UDP:12345 | Auto-refresh 2s │                  │
│  +--------------------------------------------------+                   │
│  │ Latest                                            │                   │
│  │ intrusion  (刚刚)                                 │                   │
│  +--------------------------------------------------+                   │
│  │ History                                           │                   │
│  │ 17:39:17  192.168.137.96  intrusion               │                   │
│  │ 17:39:12  192.168.137.96  intrusion  (冷却过期后) │                   │
│  │ 17:39:08  192.168.137.96  fire (模拟烟雾)         │                   │
│  +--------------------------------------------------+                   │
└─────────────────────────────────────────────────────────────────────────┘
```

***

## 驱动层 API 速查（完整版）

### LED（led.c / led.h）

```c
void LED_Init(void);
void LED_ON(uint8_t x);       // 1=PF9, 2=PF10, 3=PE13, 4=PE14
void LED_OFF(uint8_t x);
void LED_Toggle(uint8_t x);
void LED_AllOn(void);
void LED_AllOff(void);
```

**寄存器细节**：`GPIOF->MODER` 配置 PF9/PF10 为输出模式（01），`GPIOF->OTYPER` 设为开漏，`GPIOF->BSRR` 低电平点亮。

### ADC（adc.c / adc.h）

```c
void ADC_Hardware_Init(void);
uint16_t ADC_ReadPhotoresistor(void);   // PF7 / ADC3_IN5，0~4095
uint16_t ADC_ReadMQ2(void);             // PA6 / ADC1_IN6，0~4095
uint16_t ADC_ReadPotentiometer(void);   // PA5 / ADC1_IN5（预留）
```

### HC-SR04 超声波（hcsr04.c / hcsr04.h）

```c
void HCSR04_Init(void);
float HCSR04_GetDistance(void);        // 单位 cm，超时返回 0.0
uint32_t HCSR04_GetDistanceMM(void);  // 单位 mm，整数
uint8_t g_hcsr04_err;                 // 错误码（全局可读）
```

**工作原理**：

1. Trig（PC8）发 ≥10us 高电平脉冲
2. HC-SR04 自动发 8 个 40kHz 超声波脉冲
3. Echo（PC6）拉高电平，高电平持续时间 = 声波出去+回来的时间
4. PC6 中断：上升沿记录 t1，下降沿记录 t2 → 距离 = (t2-t1) × 340m/s / 2

| 错误码 | 含义             | 常见原因                     |
| --- | -------------- | ------------------------ |
| 0   | 成功             | —                        |
| 1   | Echo 上升沿未出现    | VCC 没接 5V / Trig-Echo 接反 |
| 2   | Echo 上升沿后超时无下降 | 距离 > 4m / 反射面角度太大        |

### ESP8266 WiFi（esp8266.c / esp8266.h）

```c
void ESP8266_Init(void);                              // 连 WiFi + 建 UDP，约 15 秒
uint8_t ESP8266_ConnectUDP(char *ip, uint16_t port);  // 建 UDP 通道
uint8_t ESP8266_SendUDP(uint8_t *data, uint16_t len); // 发原始数据
void ESP8266_SendAlert(const char *msg);              // ⭐ 发告警 JSON，5 秒冷却
void ESP8266_SendData(const char *topic, const char *payload);
```

**esp8266.h 配置项**：

```c
#define ESP8266_WIFI_SSID       "<your-ssid> 0997"
#define ESP8266_WIFI_PASSWORD   "6Z5{58n8"
#define ESP8266_UDP_SERVER_IP   "192.168.137.1"
#define ESP8266_UDP_SERVER_PORT 12345
#define ESP8266_USART_BAUD      115200
```

### 蜂鸣器（beep.c / beep.h）

```c
void Beep_Init(void);
void Beep_On(void);                   // TIM13 PWM 开 → 发声
void Beep_Off(void);                  // PWM 关 → 静音
void Beep_SetFreq(uint16_t freq);     // 改频率（Hz），20Hz~20kHz
void Beep_PlayNote(uint16_t freq, uint16_t duration_ms);  // 阻塞播放
```

**频率计算公式**：`TIM13->ARR = SystemCoreClock / (TIM13_Prescaler+1) / freq - 1`
默认预分频 168-1，所以 ARR = 1000000 / freq - 1（1MHz 时基）

### 矩阵键盘（matrix\_key.c / matrix\_key.h）

```c
void Matrix_Key_Init(void);
char Key_Scan(void);   // 非阻塞，返回 'A'~'D','1'~'9','0','*','#'，无键=0
```

**扫描原理**：

```
步骤1：PC9=低（第一行）→ 读 PB7/PA4/PG15/PC7 → 哪个变低就是哪一列按下
步骤2：PB6=低（第二行）→ 读 PB7/PA4/PG15/PC7
...
```

**物理布局**：

```
    列0(PB7)  列1(PA4)  列2(PG15)  列3(PC7)
行0(PC9) [ 1 ]  [ 2 ]  [ 3 ]  [ A ]
行1(PB6) [ 4 ]  [ 5 ]  [ 6 ]  [ B ]
行2(PE6) [ 7 ]  [ 8 ]  [ 9 ]  [ C ]
行3(PA8) [ * ]  [ 0 ]  [ # ]  [ D ]
```

### DELAY（delay.c / delay.h）

```c
void Delay_Init(void);        // 启动 SysTick，1ms 中断一次
void delay_ms(uint32_t ms);   // 阻塞延时（用 DWT 忙等，不依赖中断）
void delay_us(uint32_t us);   // 微秒延时
extern volatile uint32_t g_tick;  // ★ 系统运行毫秒数（中断里自增）
```

### 决策引擎（decision\_engine.h 宏开关）

```c
// Hardware/APP/decision_engine.h 顶部
#define USE_OLED        0   // 0=无 OLED 也能编译运行
#define USE_ESP8266     1   // 1=启用 ESP8266 远程推送
```

**为什么宏放头文件？** 放 main.c 里只有 main.c 知道这个宏，decision\_engine.c 编译时 `USE_ESP8266` 没定义 → `#if` 不生效 → ESP8266 代码被编进去但链接时找不到符号 → 报错。放公共头文件里，所有 include 它的 .c 文件都能看到。

***

## 完整串口日志参考

```
==== SmartGuard Agent v0.6 ====
Mode: A=HOME  B=AWAY  C=SLEEP
HW: OLED=0  ESP8266=1
================================
[OK] ADC 光敏PF7
[OK] HCSR04
[OK] PWM
[OK] MatrixKey
[SKIP] OLED (not connected)
[...] ESP8266 connecting to <your-ssid> 0997...
[...] ESP8266 connecting UDP to 192.168.137.1:12345...
[...] ESP8266 AT+CIPMUX=0
  -> CIPMUX=0 OK
[...] ESP8266 AT+CIPSTART="UDP","192.168.137.1",12345
  -> CIPSTART UDP FAIL (may be firewall, try anyway)
[OK] ESP8266 UDP connected
[OK] ESP8266 WiFi connected
--------------------------------
Entering baseline learning (20s)...
[AT] 开始基线学习，共 20 秒
[AT] 采样  1/20  light=2018 mq2=   0 temp=25.0 humi=50.0
[AT] 采样  2/20  light=2050 mq2=   0 temp=25.0 humi=50.0
...
[AT] 采样 20/20  light=2004 mq2=   0 temp=25.0 humi=50.0
[AT] 基线学习完成
[AT] light_base=2016 sigma=14
[AT] mq2_base=   0 sigma=0 fire_th= 400
[AT] temp_base=25.0 sigma=0.0  humi_base=50.0 sigma=0.0
--------------------------------
BOOT OK  Mode: HOME
================================
[MODE] AWAY 离家模式 - 入侵检测开
[ALERT] 入侵告警！距离=5.0cm
[ESP8266] UDP sent: {"type":"alert","msg":"intrusion"}
[OK] 环境正常
```

***

## 快速上手（带排错点）

### Step 1 · 硬件接线

| 接线            | 要注意的坑                                                |
| ------------- | ---------------------------------------------------- |
| HC-SR04 VCC   | **必须 5V**，3.3V 上电后模块反复复位 → 超声波没回波                    |
| ESP8266 GND   | **必须和 STM32 GND 共地**，否则 ESP8266 收到的 TX 信号浮空 → 不响应 AT |
| ESP8266 TX/RX | TX→RX、RX→TX（交叉接），接反 AT 指令石沉大海                        |
| 矩阵键盘 列3       | PG15 是 FSMC 引脚，注意不要和 SRAM 冲突                         |

### Step 2 · 配置 WiFi

编辑 `Hardware/ESP8266/esp8266.h`：

```c
#define ESP8266_WIFI_SSID       "你的WiFi名（注意有没有空格！）"
#define ESP8266_WIFI_PASSWORD   "你的WiFi密码"
#define ESP8266_UDP_SERVER_IP   "192.168.137.1"   // 电脑的局域网 IP（ipconfig 查）
#define ESP8266_UDP_SERVER_PORT 12345
```

### Step 3 · Keil 编译

```
1. 完全关闭 Keil（.uvoptx 会缓存旧路径！）
2. 双击 User/Template.uvprojx
3. 魔术棒 → Target → Device: STM32F407ZE
4. 魔术棒 → C/C++ → Define: STM32F40_41xxx, USE_STDPERIPH_DRIVER
5. 魔术棒 → C/C++ → Include Paths（全部 Hardware 子目录 + APP）
6. Rebuild → Download
```

### Step 4 · 启动 Web Dashboard

```powershell
# 1. 开 Windows 防火墙（首次会弹）
netsh advfirewall firewall add rule name="SmartGuard UDP 12345" dir=in action=allow protocol=UDP localport=12345

# 2. 启动 Dashboard
cd "d:\Projects\cq2610\CQ2610\08智能硬件开发\01STM32单片机\SmartGuard"
powershell -ExecutionPolicy Bypass -File web_dashboard.ps1

# 3. 浏览器打开
start http://localhost:8080
```

### Step 5 · 验证流程

| # | 操作             | 预期                      | 排查                                  |
| - | -------------- | ----------------------- | ----------------------------------- |
| 1 | Reset + 等 20 秒 | `BOOT OK Mode: HOME`    | 基线学习卡在某个采样 → 传感器没接好                 |
| 2 | 按 B 键          | `[MODE] AWAY`           | 按键没反应 → 接反了行列                       |
| 3 | 手靠近超声波         | `[ALERT] 入侵告警！距离=5.0cm` | 一直 0.0cm → VCC 没接 5V 或 Trig/Echo 接反 |
| 4 | 看 Dashboard    | 1 条告警                   | 没收到 → 防火墙/IP 错/ESP8266 GND 没共地      |
| 5 | 手离开            | `[OK] 环境正常`             | —                                   |
| 6 | 5 秒内再靠近        | 蜂鸣器响但**不发新 UDP**        | —                                   |
| 7 | 等 5 秒再靠近       | 蜂鸣器响 + **发新 UDP**       | 冷却机制验证 ✅                            |

***

## 故障排查（真实踩过的坑）

### 🔴 HC-SR04 一直 0.0cm

```
现象：串口距离值固定 0.0，g_hcsr04_err=1
原因：VCC 接了 3.3V
解决：改接 5V。3.3V 供电时 HC-SR04 内部振荡器不稳定，无法产生回波
```

### 🔴 ESP8266 连不上 WiFi

```
现象：串口一直刷 "AT+CWJAP ... busy p..."
原因1：SSID 字符串里有不可见字符（复制粘贴时带了零宽空格）
解决：手动输入 SSID，不要复制粘贴
原因2：密码里有特殊字符（!@#$），AT 指令需要转义
解决：换纯字母数字密码；或在 AT 指令里加转义
```

### 🔴 ESP8266 UDP 收不到

```
现象：STM32 串口显示 "UDP sent: {...}"，但 PowerShell 没收到
原因1：Windows 防火墙拦截入站 UDP 12345
解决：netsh advfirewall firewall add rule name="SG UDP" dir=in action=allow protocol=UDP localport=12345
原因2：ipconfig 查错 IP（用了 192.168.1.1 而实际是 192.168.137.1）
原因3：ESP8266 和电脑不在同一网段
解决：确认 WiFi 是同一个（电脑连的是 <your-ssid> 0997 吗？）
```

### 🔴 告警重复发送 80+ 条/秒

```
现象：手放在超声波前面不动，Dashboard 被刷屏
原因：无冷却机制，decision_engine.c 每次状态检测都触发 SendAlert
修复位置：esp8266.c ESP8266_SendAlert()
修复方案：加 g_tick 5 秒冷却判断（SEND_COOLDOWN_MS=5000）
状态：✅ 已修复 v0.6
```

### 🔴 串口中文乱码

```
现象：串口助手显示 "娴忚鍣ㄦ墦寮" 等
原因：Keil 源文件是 GB2312，串口助手用 UTF-8
解决：Keil → Edit → Configuration → Editor → Encoding → UTF-8 with BOM
      串口助手编码也改成 UTF-8
```

### 🔴 Keil 编译报旧路径错

```
现象：改了文件夹名后编译报找不到 .h 文件，路径是旧的
原因：.uvoptx 缓存了旧路径
解决：完全关闭 Keil → 删除 User/Template.uvoptx → 重新打开 .uvprojx
```

### 🔴 OLED 不亮

```
现象：接好 PD6/PD7，OLED_Init() 返回 0（探测失败）
原因：PD6/PD7 是 FSMC SRAM 引脚！原厂开发板连了外部 SRAM，这些引脚被占用
解决：改接到 PB12-PB15（这 4 个引脚在 GEC-M4 上完全空闲）
     PB12=CS, PB13=SCLK, PB14=DC, PB15=MOSI
```

### 🔴 矩阵键盘只有 B 键能用

```
现象：按 1/2/3/4 等都没反应，只有 B 键有
原因：行列映射错误（接反了列和行）
解决：列是输出（PB7/PA4/PG15/PC7），行是输入（PC9/PB6/PE6/PA8）
      参考 matrix_key.h 的 KEY_ROW/KEY_COL 定义核对接线
```

### 🔴 ESP8266 CIPSTART 报 FAIL 但实际能发

```
现象：串口打印 "CIPSTART UDP FAIL"，但 UDP 包正常收发
原因：ESP8266 固件版本差异（某些版本 UDP 模式下 CIPSTART 不返回 OK）
解决：忽略这个日志。只要 CIPSEND 能正常回复 OK 就行
```

***

## 面试话术 · 完整版

### Q：你这个项目用了什么技术栈？

> "STM32F407ZE 裸机开发，ARM Cortex-M4 内核，标准外设库。传感器层是光敏 ADC + HC-SR04 超声波 + 4×4 矩阵键盘 + ESP8266 WiFi。软件上用了 **自适应阈值（均值+3σ+EWMA）** 做异常检测，**状态机** 管理 HOME/AWAY/SLEEP 三种模式，**AT 指令时序控制** ESP8266 做 UDP 推送。PC 端用 PowerShell 写了一个 UDP 监听 + HTTP Web Dashboard 接收告警。"

### Q：自适应阈值是怎么实现的？为什么不用固定阈值？

> "固定阈值的问题是环境变化时要么漏报要么误报——比如白天光敏 ADC 500，晚上 2500，一个固定阈值怎么设？
>
> 我用的是**基线学习 + 3σ 异常检测 + EWMA 动态更新**。上电后先阻塞 20 秒，每秒采一次所有传感器，算均值 μ 和标准差 σ，阈值设为 μ + 3σ（覆盖 99.7% 正常波动）。运行期每 500ms 用 EWMA（α=0.1）慢慢更新 baseline，让它跟着昼夜变化移动。但**偏离超过 2σ 的值拒绝吸收**——防止真异常被'学进去'。
>
> 实测数据：我房间光敏 baseline=2016，σ=14，3σ=42，正常范围 \[1974, 2058]。闪光灯直射到 \~1000 会触发，但云飘过到 2030 不会。烟雾检测我用了双条件——MQ2 超阈值 **且** 温度明显上升，避免厨房炒菜触发火灾告警。"

### Q：怎么防止 ESP8266 重复发送告警？

> "两层防护。第一层在决策引擎里——只有状态变化时才触发 SendAlert（`alert != s_alert_last`），同一告警状态不会重复推。第二层在 SendAlert 内部做 **5 秒冷却**，用 SysTick 的全局 g\_tick 差值判断，5 秒内调用直接 return。这样即使哪里漏了状态判断也不会刷屏。ESP8266 AT 指令是阻塞发的，冷却可以避免占着 USART。"

### Q：为什么 OLED 原来不亮？

> "踩了 FSMC 引脚的坑。GEC-M4 开发板原厂接了外部 SRAM，PD6/PD7 是 FSMC 数据线，被 SRAM 占用了。一开始用这两个引脚驱动 OLED SPI 当然不行。后来查原理图发现 PB12-PB15 完全空闲，就把 OLED SPI 改接过去——CS=PB12, SCLK=PB13, DC=PB14, MOSI=PB15。这类坑在 STM32 开发里很常见，引脚复用前一定要查原理图。"

### Q：后续有什么扩展方向？

> "短期想接 DHT11 + MQ-2 把火灾检测补全，再接 OLED 做本地显示。中期想上 FreeRTOS 做真正的多任务——现在是裸机 while(1) 轮询，基线学习那 20 秒整个系统卡死在采样里。上 RTOS 后可以把采样、决策、ESP8266 发送分成独立任务，互不阻塞。然后 MQTT 上云，参考课程资料里的阿里云 MQTT 工程。长期的话 OV7670 摄像头做帧差法移动检测，或者用 CMSIS-NN 跑一个轻量分类模型做端侧异常检测。"

***

## Keil 工程常见问题

### Q1 · 编译报旧路径错误

关闭 Keil → 删除 `User/Template.uvoptx`（**不是 .uvprojx！**）→ 重新打开。

### Q2 · `#include "xxx.h"` 找不到

魔术棒 → C/C++ → Include Paths，加上所有 Hardware 子目录：

```
..\Hardware\BEEP
..\Hardware\DELAY
..\Hardware\LED
..\Hardware\USART
..\Hardware\ADC
..\Hardware\KEY
..\Hardware\MATRIX_KEY
..\Hardware\DHT11
..\Hardware\MQ2
..\Hardware\HCSR04
..\Hardware\ESP8266
..\Hardware\PWM
..\Hardware\OLED
..\Hardware\SERVO
..\Hardware\APP
```

### Q3 · `Error: L6218E: No room in the heap`

USART1\_Printf 格式化输出吃栈。魔术棒 → Target → 增大 Heap Size（如 0x400）和 Stack Size（如 0x800）。

### Q4 · 中文串口输出乱码

Keil 编辑器：Edit → Configuration → Editor → Encoding → **UTF-8 with BOM** → 保存所有 .c/.h 文件。串口助手也用 UTF-8。

***

## 开发路线图

| 阶段      | 版本       | 核心能力                                                                                         | 状态      |
| ------- | -------- | -------------------------------------------------------------------------------------------- | ------- |
| ✅ 已完成   | **v0.6** | 裸机采集 + 自适应阈值（基线 20s + 3σ + EWMA）+ 矩阵键盘切模式 + ESP8266 UDP 推送 + 5s 冷却 + Web Dashboard + 状态变化防刷屏 | ✅       |
| 📋 v0.7 | **v0.7** | OLED 本地显示（PB12-PB15 SPI）+ DHT11 温湿度 + MQ-2 烟雾 + 多传感器融合告警 UI                                  | 待接硬件    |
| 📋 v1.0 | **v1.0** | FreeRTOS 多任务（采样/决策/通信独立任务）+ MQTT 上云（参考 `03其他资料/MQTT`）+ 阈值 Flash 掉电保存                         | \~1.5 周 |
| 📋 v2.0 | **v2.0** | CMSIS-NN 端侧异常检测（Isolation Forest 或 TinyML）+ OV7670 帧差法 + 舵机云台                                | \~2 周   |
| 📋 v3.0 | **v3.0** | 鸿蒙智联（HarmonyOS Connect）+ RK3568 大模型离线推理 + 多设备 Mesh 组网                                        | \~3 周   |

***

## 已知课程资料索引

| 资源                     | 路径                            | 用途                              |
| ---------------------- | ----------------------------- | ------------------------------- |
| HC-SR04 说明书 / 原理图      | `01模块资料/HC-SR04超声波模块/`        | 确认 Trig/Echo 电平、VCC 范围          |
| OLED SSD1306 资料        | `01模块资料/IIC/096寸OLED模块资料/`    | OLED 驱动参考（注意改 SPI 后要换软件）        |
| SG90 舵机资料              | `01模块资料/SG90舵机180度/`          | 舵机 PWM 参数（50Hz, 1.5ms 中值脉宽）     |
| ESP8266 示例工程           | `02示例工程/13无线通信/`              | AT 指令交互、串口 DMA 接收参考             |
| MQTT 完整工程（ESP8266+阿里云） | `03其他资料/MQTT/MQTT/MYCODE/`    | v1.0 MQTT 上云参考                  |
| GEC-M4 原理图             | `00工具软件/06 原理图/GEC-M4原理图.pdf` | 查引脚复用（FSMC / TIM / USART / ADC） |

***

## 编码约定

| 项     | 规则                                                                  |
| ----- | ------------------------------------------------------------------- |
| 源码编码  | 所有 `.c` / `.h` 文件使用 **UTF-8 with BOM**                              |
| 串口编码  | USART1 固定 **115200 / 8N1 / UTF-8**                                  |
| 变量命名  | 全局 `g_` 前缀，局部静态 `s_`，模块名 snake\_case                                |
| 注释语言  | 中文（配合 UTF-8 with BOM）                                               |
| 驱动结构  | 每个硬件模块一个文件夹，内含 `<name>.c` + `<name>.h`                              |
| 头文件保护 | `#ifndef __MODULE_NAME_H__` / `#define` / `#endif`                  |
| 硬件开关宏 | 集中在 `Hardware/APP/decision_engine.h` 顶部（如 `USE_ESP8266=1`），**全局生效** |
| 阻塞延时  | 统一用 `delay_ms()` / `delay_us()`；不要自己写 `while(n--) __NOP()`          |
| 错误处理  | 所有 AT 指令、传感器读取都检查返回值，失败时有日志输出                                       |

***

## License

个人学习与课程项目使用。
