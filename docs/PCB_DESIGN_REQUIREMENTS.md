# SmartGuard PCB 设计需求规格书

> 交付对象：PCB 布局工程师  
> 版本：v0.7.2  
> 日期：2026-10-20

---

## 1. 项目概述

SmartGuard 是基于 STM32F407ZE 的智能空间守护系统，集成 6 路传感器、端侧自适应阈值、手写 MQTT 上云、事件日志黑匣子及 Web 实时监控。本规格书供 PCB 设计阶段使用，所有引脚分配以当前量产固件源码为唯一依据。

---

## 2. 核心 MCU

| 项 | 规格 |
|---|---|
| 型号 | STM32F407ZE |
| 主频 | 168 MHz |
| Flash | 1 MB |
| SRAM | 192 KB |
| GPIO 电平 | 3.3V（部分引脚 5V-tolerant，见第 6 节） |
| 封装 | LQFP-144 |

---

## 3. 电源架构

系统需**两路电源**：5V 与 3.3V。5V 经板载 LDO 降压给 MCU 及 3.3V 外设。

| 电源轨 | 用途 | 建议电流容量 | 备注 |
|---|---|---|---|
| **5V** | OLED、HC-SR04、MQ-2、ESP32-CAM | ≥1 A | ESP32-CAM 峰值 ≥500mA，需独立粗线或局部铜皮 |
| **3.3V** | MCU、ESP8266、MPU6050、AT24C02、W25Qxx、DHT11、光敏、LED、蜂鸣器 | ≥500 mA | ESP8266 峰值约 300mA |

### 供电注意事项
- **共地**：所有模块 GND 必须与单片机 GND 单点共地，ESP32-CAM 与 ESP8266 的 GND 不得经细长走线回流。
- **5V 模块**：OLED、HC-SR04、MQ-2 必须接 5V；3.3V 供电会导致功能异常（OLED 可能微亮/不亮，HC-SR04 无回波）。
- **ESP32-CAM**：独立 5V 供电，≥500mA，建议预留 micro-USB 焊盘或直接引出 5V/GND 排针。

---

## 4. 外设接口与引脚分配

### 4.1 按功能汇总

| 功能 | 模块 | 引脚 | 接口类型 | 电平 | 方向 |
|---|---|---|---|---|---|
| 显示 | SSD1306 OLED | PB0(SCLK) PB1(DC) PF11(MOSI) PC13(CS) | 软件 SPI | 5V | MCU→ |
| 测距 | HC-SR04 | PC8(Trig) PC6(Echo) | GPIO + TIM8 输入捕获 | 5V | 双向 |
| WiFi | ESP8266 | PB10(USART3_TX) PB11(USART3_RX) | USART3 115200 | 3.3V | 双向 |
| 摄像头 | ESP32-CAM | PA2(USART2_TX) PA3(USART2_RX) | USART2 115200 | 3.3V | 双向 |
| 温湿度 | DHT11 | PG9 | 单总线 | 3.3V | 双向 |
| 光照 | 光敏电阻 | PF7(ADC3_IN5) | ADC | 3.3V | MCU← |
| 烟雾 | MQ-2 | PA4(ADC1_IN4) | ADC | 5V | MCU← |
| 姿态 | MPU6050 | PB8(SCL) PB9(SDA) | 软件 I2C | 3.3V | 双向 |
| 存储 | AT24C02 | PB8(SCL) PB9(SDA) | 软件 I2C（与 MPU6050 同总线） | 3.3V | 双向 |
| 日志 Flash | W25Qxx | PB14(CS) PB3(SCK) PB4(MISO) PB5(MOSI) | 软件 SPI | 3.3V | 双向 |
| 按键 S1 | HOME 模式 | PA0 | GPIO 输入，内部上拉 | 3.3V | MCU← |
| 按键 S2 | AWAY 模式 | PE2 | GPIO 输入，内部上拉 | 3.3V | MCU← |
| 按键 S3 | SLEEP 模式 | PE3 | GPIO 输入，内部上拉 | 3.3V | MCU← |
| 按键 S4 | 告警测试/日志回放 | PE4 | GPIO 输入，内部上拉 | 3.3V | MCU← |
| 蜂鸣器 | 无源蜂鸣器 | PF8(TIM13_CH1) | PWM | 3.3V | MCU→ |
| LED0 | 告警灯 | PF9 | GPIO 输出，低电平亮 | 3.3V | MCU→ |
| LED1 | — | PF10 | GPIO 输出，低电平亮 | 3.3V | MCU→ |
| LED2 | — | PE13 | GPIO 输出，低电平亮 | 3.3V | MCU→ |
| LED3 | 云端指示灯 | PE14 | GPIO 输出，低电平亮 | 3.3V | MCU→ |
| 调试 | SWD | PA13(SWDIO) PA14(SWCLK) | J-Link / ST-Link | 3.3V | — |
| RTC | 低速晶振 | PC14(OSC32_IN) PC15(OSC32_OUT) | LSE 32.768kHz | — | — |

### 4.2 按 GPIO 排序（供原理图核对）

| GPIO | 功能 | 备注 |
|---|---|---|
| PA0 | 按键 S1（HOME） | 低电平有效，内部上拉 |
| PA2 | USART2_TX → ESP32-CAM | 3.3V |
| PA3 | USART2_RX ← ESP32-CAM | 3.3V |
| PA4 | ADC1_IN4 ← MQ-2 | 5V 输入，建议 RC 滤波 |
| PA13 | SWDIO | 调试器 |
| PA14 | SWCLK | 调试器 |
| PB0 | OLED SCLK | 5V-tolerant |
| PB1 | OLED DC | 5V-tolerant |
| PB3 | W25Qxx SCK | JTDO 复用，调试器须 SWD 模式 |
| PB4 | W25Qxx MISO | NJTRST 复用，调试器须 SWD 模式 |
| PB5 | W25Qxx MOSI | — |
| PB6 | — | 矩阵键盘已弃用，可留测试点 |
| PB7 | — | 矩阵键盘已弃用，可留测试点 |
| PB8 | I2C SCL（MPU6050 + AT24C02） | 软件 I2C，需外部 4.7kΩ 上拉到 3.3V |
| PB9 | I2C SDA（MPU6050 + AT24C02） | 同上 |
| PB10 | USART3_TX → ESP8266 | — |
| PB11 | USART3_RX ← ESP8266 | — |
| PB14 | W25Qxx CS | — |
| PC6 | TIM8_CH1 ← HC-SR04 Echo | 5V-tolerant |
| PC7 | — | 矩阵键盘已弃用 |
| PC8 | GPIO → HC-SR04 Trig | — |
| PC13 | OLED CS | 与 RTC Tamper 复用，须软件禁用 Tamper |
| PC14 | OSC32_IN | LSE 晶振 |
| PC15 | OSC32_OUT | LSE 晶振 |
| PE2 | 按键 S2（AWAY） | 低电平有效 |
| PE3 | 按键 S3（SLEEP） | 低电平有效 |
| PE4 | 按键 S4（告警测试/日志回放） | 低电平有效，支持短按/长按 |
| PE6 | — | 矩阵键盘已弃用 |
| PE13 | LED2 | 低电平亮 |
| PE14 | LED3 / TIM1_CH4 PWM | 与 LED 复用 |
| PF7 | ADC3_IN5 ← 光敏电阻 | 3.3V |
| PF8 | TIM13_CH1 → 蜂鸣器 | PWM 驱动无源蜂鸣器 |
| PF9 | LED0 / TIM14_CH1 PWM | 与 LED 复用 |
| PF10 | LED1 | 低电平亮 |
| PF11 | OLED MOSI | — |
| PG9 | DHT11 DATA | 建议 4.7kΩ 上拉到 3.3V |
| PG15 | — | 矩阵键盘已弃用 |

---

## 5. 通信协议细节

| 总线 | 速率/模式 | 特殊要求 |
|---|---|---|
| OLED SPI | 软件 SPI，Mode 0 | F407@168MHz 时 bit-bang 需延时约 1μs；CS2 必须接 VCC 禁用字库芯片 |
| W25Qxx SPI | 软件 SPI，Mode 0 | 支持 W25Q128；注意 SR 写保护位自动解除逻辑 |
| I2C（MPU6050+AT24C02）| 软件 I2C，~100kHz | 两器件同总线，地址 0x68 与 0x50 不冲突；必须外接上拉电阻 |
| USART2 | 115200-8-N-1 | 与 ESP32-CAM 通信，仅发指令/收 URL |
| USART3 | 115200-8-N-1 | 与 ESP8266 AT 指令通信；CH_PD 必须上拉 |

---

## 6. 关键设计约束

### 6.1 引脚复用冲突（已踩坑）

| 引脚 | 功能 A | 功能 B | 处理方案 |
|---|---|---|---|
| **PF9** | LED0 | TIM14_CH1 PWM | 共用，但使用 PWM 时普通 GPIO 控制失效 |
| **PE14** | LED3 | TIM1_CH4 PWM | 同上 |
| **PC13** | OLED CS | RTC Tamper | 软件必须禁用 Tamper，否则 OLED 初始化后黑屏 |
| **PB3/PB4** | W25Qxx SCK/MISO | JTAG JTDO/NJTRST | 调试器必须用 SWD 模式，禁用 JTAG |
| **PB8/PB9** | I2C（MPU6050+AT24C02）| — | 矩阵键盘原设计也占用 PB8/PB9，已弃用，无冲突 |

### 6.2 5V-tolerant 引脚

以下引脚可安全接收 5V 输入，其余 GPIO 严禁直接接 5V：
- **PB0, PB1, PC6**（OLED SCLK/DC、HC-SR04 Echo）

MQ-2 的 PA4 接 5V 模拟输出，但 PA4 **不是** 5V-tolerant；MQ-2 输出为模拟信号，经分压或确认其输出不超过 3.3V。实际当前代码直接读 PA4 ADC，请确认 MQ-2 模块输出已做限幅，或改用 5V-tolerant ADC 引脚（如需要请告知）。

> ⚠️ 当前工程实际：MQ-2 模块模拟输出在安静环境约 180（12bit ADC 量程内），未超 3.3V，可直接接 PA4。若换用其他 MQ-2 模块，请先测输出摆幅。

### 6.3 上拉电阻

| 位置 | 阻值 | 说明 |
|---|---|---|
| I2C SCL/SDA（PB8/PB9）| 4.7kΩ | 必须外接，软件 I2C 无内部上拉 |
| DHT11 DATA（PG9）| 4.7kΩ | 建议外接，增强单总线稳定性 |
| 按键 S1~S4 | 内部上拉（代码配置）| 无需外部电阻，但可并 100nF 消抖电容 |
| ESP8266 CH_PD | 10kΩ → VCC | 模块使能，必须拉高 |

### 6.4 晶振

| 晶振 | 频率 | 用途 | 负载电容 |
|---|---|---|---|
| HSE | 8 MHz（或 25 MHz）| 系统主时钟 | 按晶振规格 |
| LSE | 32.768 kHz | RTC 实时时钟 | 6~12pF 典型值 |

---

## 7. 连接器建议

| 接口 | 建议连接器 | 备注 |
|---|---|---|
| 电源输入 | DC-005 或 USB-C | 5V 输入，建议加 TVS 保护 |
| SWD 调试 | 2.54mm 4P 排针（VCC/GND/SWDIO/SWCLK）| 或预留 JTAG 转接焊盘 |
| ESP32-CAM | 2.54mm 4P 排针（5V/GND/TX/RX）| 独立供电，可拔插 |
| ESP8266 | 2.54mm 4P 排针（3.3V/GND/TX/RX）| 建议 CH_PD 在板上已上拉 |
| OLED | 8P 2.54mm 排母或 FPC | 注意 CS2 已在板上接 VCC |
| 传感器（DHT11/光敏/MQ-2/HC-SR04）| 2.54mm 排针或 XH2.54 | 按实际模块接口匹配 |
| 蜂鸣器 | 2P 排针或直插焊盘 | 无源蜂鸣器 |

---

## 8. 布局建议

1. **模拟地与数字地**：MQ-2（PA4）、光敏（PF7）为模拟输入，走线远离 USART/SPI 高速信号，建议单点接地或星型接地。
2. **电源去耦**：ESP8266、ESP32-CAM 附近各放 100μF + 100nF；MCU 每个 VDD 引脚配 100nF。
3. **天线避让**：ESP8266、ESP32-CAM 天线区域禁止铺铜，远离蜂鸣器/电机等噪声源。
4. **按键走线**：S1~S4 走线短，远离蜂鸣器和 5V 电源线，避免串扰导致伪触发。
5. **W25Qxx 走线**：SPI 信号线尽量等长，SCK 串联 22Ω 阻抗匹配，靠近 MCU 放置。

---

## 9. 弃用/移除模块（板上无需预留）

| 模块 | 弃用原因 |
|---|---|
| 4×4 矩阵键盘 | 与 MQ-2 抢 PA4；模式切换已由 S1~S4 覆盖 |
| SG90 舵机 | 驱动已移除，项目中未使用 |

---

## 10. 版本与核对

| 核对项 | 状态 |
|---|---|
| 引脚与 `Hardware/*/*.h` 源码一致 | ✅ |
| 引脚与 README.md 硬件清单一致 | ✅ |
| 5V/3.3V 区分与实测一致 | ✅ |
| 复用冲突已标注 | ✅ |
| 弃用模块已排除 | ✅ |

> 如固件引脚发生变更，请以 `Hardware/` 目录下各 `.h` 文件中的 GPIO 宏定义为第一依据，并同步更新本文档。
