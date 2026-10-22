# 构建与烧录

> 工程入口：`User/Template.uvprojx`（Keil MDK 5）。
> 本文命令均在仓库根目录 `SmartGuard/` 下执行。

---

## 1. 构建环境

| 项 | 要求 |
|---|---|
| IDE | Keil MDK 5（uVision 5） |
| 编译器 | ARMCC V5（AC5，`ARM\ARMCC`） |
| 器件包 | STM32F4xx Device Family Pack（工程已包含所需头文件与启动文件） |
| 外设库 | STM32F4xx 标准外设库（仓库内 `Library/`） |
| RTOS | FreeRTOS V10.5.1（仓库内 `FreeRTOS/`） |
| 调试器 | J-Link（SWD） |
| 串口工具 | 任意串口助手，115200 8N1 |

工程使用 **标准外设库 + 手写初始化**，不依赖 STM32CubeMX 生成的 HAL 代码。

---

## 2. 首次编译前的必要准备

WiFi 与巴法云凭据已外置，真实值不入库。首次编译前必须从模板复制：

```text
Hardware/ESP8266/wifi_config.h.example  ->  Hardware/ESP8266/wifi_config.h
Hardware/ESP8266/bemfa_config.h.example ->  Hardware/ESP8266/bemfa_config.h
```

然后填写自己的 SSID / 密码（以及可选的巴法云 UID / Topic）。

> 缺少 `wifi_config.h` 或 `bemfa_config.h` 时，`esp8266.h` 找不到头文件，整个工程编译失败。这是有意设计：避免把真实凭据提交进仓库。
>
> 当前固件主链路使用自建 mosquitto + MQTT，巴法云相关代码保留但非主路径。

---

## 3. 编译

### 3.1 Keil 图形界面

1. 打开 `User/Template.uvprojx`；
2. 选择目标 `Template`；
3. `Project → Rebuild all target files`（建议全量，见 3.4）；
4. 确认输出窗口为 `0 Error(s), 0 Warning(s)`。

### 3.2 命令行全量编译

```powershell
& "<Keil安装目录>\UV4\UV4.exe" -r "User/Template.uvprojx" -j0 -o build_log.txt
Get-Content build_log.txt -Tail 5
```

期望输出：

```text
Program Size: Code=51568 RO-data=6504 RW-data=372 ZI-data=55692
"..\Object\Template.axf" - 0 Error(s), 0 Warning(s).
```

> 上述数值来自 2026-10-22 当前工作区的全量构建记录（`Object/Template.build_log.htm`），会随代码改动变化。

### 3.3 产物

| 文件 | 说明 |
|---|---|
| `Object/Template.axf` | 调试用 ELF |
| `Object/Template.hex` | 烧录用 Intel HEX |
| `Object/Template.build_log.htm` | Keil 构建日志 |

`Object/`、`*.hex`、`*.axf`、`build_log.txt` 均已被 `.gitignore` 排除。

### 3.4 关于"陈旧构建"

Keil 增量构建在个别情况下会复用旧目标文件，出现"0 Error / 0 Warning 但改动未编入"的假阳性。发布或演示前请使用 **Rebuild all target files**，并核对 `Program Size` 是否随改动变化。

---

## 4. 烧录

### 4.1 Keil 内下载

1. J-Link 连接 SWD：SWDIO、SWCLK、GND、3V3（目标板供电方式按实际选择）；
2. `Options for Target → Debug` 选择 J-Link / SWD；
3. `Flash → Download`（或工具栏 Load）。

### 4.2 命令行下载

```powershell
& "<Keil安装目录>\UV4\UV4.exe" -f "User/Template.uvprojx"
```

> J-Link 的 SWD 速率与复位方式按实际探针配置。PB3/PB4 为 JTAG 复用脚，必须使用 SWD 模式。

### 4.3 烧录后的注意事项

| 现象 | 原因 | 处理 |
|---|---|---|
| MPU6050 原始值恒为 0 | 多次 J-Link 烧录后器件停留在睡眠模式 | **彻底断电再上电**（软复位无效） |
| DWT 时基异常 / 启动卡住 | 反复烧录后 DWT 计数器失效 | `delay.c` 已带 SysTick 后备时基；必要时彻底断电 |
| 调试 halt 后复位 | J-Link halt 冻结 CPU，IWDG 继续计时 | 正常现象；调试时可考虑冻结 IWDG |

---

## 5. 串口与启动日志

USART1 为 115200 8N1。端口号以设备管理器实际枚举为准（不同电脑不同），不要照搬文档中的 COM 号。

正常启动的关键日志：

```text
====================================
  SmartGuard v0.7.2 - Space Guard Agent (FreeRTOS)
====================================
[INIT] LED...OK
[INIT] BEEP...OK
[INIT] ADC...OK
[INIT] HCSR04...OK
[INIT] DHT11...OK
[INIT] BoardKey...OK
[INIT] OLED SSD1306...OK
[INIT] ESP8266 WiFi...OK
[INIT] ESP32-CAM...OK (online)
[INIT] RTC Calendar...OK
[INIT] MPU6050...OK WHO_AM_I=0x68
[INIT] AT24C02 EEPROM...OK
[INIT] W25QXX SPI Flash...OK
[BASELINE] Restored from FLASH (skip 20s learn)
[INIT] IWDG 2s watchdog...OK
[MAIN] Entering SmartGuard main loop...
[RTOS] scheduler starting (FreeRTOS V10.5.1)
```

首次上电（EEPROM 无有效配置）会改为：

```text
[BASELINE] Learning 20s baseline...
[AT] baseline learning: 20 s
...
[AT] 基线学习完成
[BASELINE] Saved to FLASH OK
```

---

## 6. 关键配置宏

| 宏 | 位置 | 默认 | 说明 |
|---|---|---|---|
| `USE_STDPERIPH_DRIVER` | `Template.uvprojx` | 定义 | 使用标准外设库 |
| `STM32F40_41xxx` | `Template.uvprojx` | 定义 | 器件系列 |
| `USE_FREERTOS` | `Template.uvprojx` | `1` | 1=FreeRTOS，0=裸机协作式调度 |
| `USE_OLED` | `decision_engine.h` | 1 | OLED 开关 |
| `USE_ESP8266` | `decision_engine.h` | 1 | WiFi 开关 |
| `USE_DHT11` | `decision_engine.h` | 1 | 温湿度开关 |
| `USE_ML` | `decision_engine.h` | 1 | 端侧 MLP 开关 |
| `USE_RTC` | `decision_engine.h` | 1 | RTC 开关 |
| `USE_MQ2` | `adc.h` | 1 | 烟雾通道开关 |
| `USE_ESP32_CAM` | `esp32_cam.h` | 1 | CAM 联动开关 |
| `BEEP_ENABLE` | `beep.h` | 1 | 蜂鸣器总开关（0=完全静音） |
| `CONFIG_USE_EEPROM` | `config_store.h` | 1 | 1=AT24C02，0=内部 Flash |
| `ML_DATA_COLLECT` | `main.c` | 0 | 1=输出 ML 采集 CSV |

> 修改 `USE_FREERTOS` 后必须全量重建，并确认 `Template.uvprojx` 中的 FreeRTOS 文件组仍参与编译。

---

## 7. 网络与 broker 准备

固件通过 ESP8266 连接 2.4 GHz 热点，再连 TCP 1883 的 MQTT broker。仓库不包含 broker 配置，需要自行安装（例如 Windows 版 mosquitto）并至少配置：

```conf
listener 1883
listener 9001
protocol websockets
allow_anonymous true
```

- 固件：`MQTT_SERVER` 与端口在 `esp8266.h` 中定义（默认局域网地址），按实际修改；
- Web 看板：使用 `ws://<broker>:9001`；
- 设备侧与 Web 侧是同一 broker 的两个监听端口。

---

## 8. 源码编码约束

本仓库 `.c` / `.h` 源文件统一为 **UTF-8 with BOM**。Keil ARMCC V5 在中文 Windows 下按 GBK 扫描源文件：

- 去掉 BOM 或批量转换为 GBK 会导致中文注释吞掉行尾、编译错误成百上千；
- 同一个文件出现多个 BOM 也会导致 `#870-D` 等错误；
- **不要**对源文件执行批量编码转换脚本。

---

## 9. 常见构建问题

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `cannot open source input file "wifi_config.h"` | 未从 `.example` 复制凭据头 | 按第 2 节复制并填写 |
| 大量 `#870-D` / 中文注释相关错误 | 源文件 BOM 丢失或重复 | 恢复为单个 UTF-8 BOM，不要转 GBK |
| `0 Error` 但功能未更新 | 增量构建复用旧目标文件 | Rebuild all |
| 链接后空间不足 | 开启了未使用的模块 | 关闭对应 `USE_*` 宏 |
| J-Link 无法连接 | 使用了 JTAG 或 PB3/PB4 被占用 | 改为 SWD，检查接线与目标供电 |
| 编译通过但串口无输出 | 波特率 / 端口错误，或 USART1 未初始化 | 确认 115200 与正确 COM 口 |

---

## 10. 相关文档

- 硬件接线与排错：[hardware.md](hardware.md)、[HARDWARE_CHECKLIST.md](HARDWARE_CHECKLIST.md)
- RTOS 任务与内核配置：[freertos-tasks.md](freertos-tasks.md)
- 测试与验证记录：[test-report.md](test-report.md)
