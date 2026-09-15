# SmartGuard · 智能空间守护系统

> STM32F407ZE 裸机 + 6 路传感器 + 端侧自适应阈值 + 手写 MQTT 上云的空间守护原型

- **一句话**：设备自己学出"这个房间正常是什么样"，只在统计显著偏离时告警，本地蜂鸣/显示的同时把状态推上云。
- **不做**：不依赖云端推理、不跑 RTOS、不挂外部存储。
- **芯片**：STM32F407ZE（Cortex-M4 · 168 MHz · 192 KB RAM · 1 MB Flash）
- **构建**：Keil MDK v5 + J-Link，当前 `0 Error(s), 0 Warning(s)`
- **配套**：支持 MQTT 双向通信 + Web 前端（Vue 3）实时监控面板

## 项目背景

常见的门禁/烟雾报警器有三个通病：**阈值写死**（换个房间就得重调）、**误报多**（一片阴影扫过就响）、**断网即失效**。SmartGuard 想验证的做法是把决策留在端侧——先学环境基线，只在显著偏离时告警，网络只负责通知。

第二个目的是工程训练：手写 MQTT 3.1.1 报文而不是接 SDK，用内部 Flash 扇区做掉电恢复而不是外挂存储，把每一类编译警告归零。这些"重复造轮子"的部分，恰好是项目里最值得讲的部分。

## 系统架构

```
感知层   光敏 PF7 · MQ-2 PA4 · DHT11 PG9 · HC-SR04 PC8/PC6 · MPU6050 PB8/PB9
           ↓ SensorHub_Update()，统一聚合为 Sensor_Data_t
决策层   adaptive_threshold  20s 基线 + σ 带 + EWMA 慢跟随
         decision_engine     HOME/AWAY/SLEEP 模式机 + 连续确认窗 + 告警冷却
         ml_inference        MLP 10→16→8→1，边界地带辅助二分类
           ↓ Alert_Type_t
执行层   蜂鸣 PF8 · OLED 本地面板 · LED · USART1 日志(115200)
           ↓
通信层   ESP8266(USART3 PB10/PB11) —AT/TCP→ 手写 MQTT 3.1.1 → mosquitto
持久层   CONFIG_STORE @ Flash Sector 7（magic + CRC32，基线与模式掉电恢复）
守护层   IWDG 2s 独立看门狗 · RTC 唤醒 · NTP 对时
```

## 硬件清单

| 模块 | 引脚 | 接口 | 状态 |
|---|---|---|---|
| SSD1306 0.96" OLED | SCLK PB0 / DC PB1 / MOSI PF11 / CS PC13 | 软件 SPI | ✅ 英文界面 |
| DHT11 温湿度 | PG9 | 单总线 | ✅ 2s 节流 |
| MQ-2 烟雾 | PA4 / ADC1_IN4 | ADC | ✅ VCC 5V，安静环境基线 ~180 |
| 光敏电阻 | PF7 / ADC3_IN5 | ADC | ✅ 光越强 ADC 越小 |
| HC-SR04 测距 | TRIG PC8 / ECHO PC6 | TIM8 输入捕获 | ✅ 30ms 超时返回 0 |
| MPU6050 | SCL PB8 / SDA PB9 | 软件 I2C | ✅ 防拆/震动 |
| ESP8266 ESP-12F | TX PB10 / RX PB11 | USART3 AT | ✅ MQTT 打通 |
| 板载按键 S1~S4 | PA0 / PE2 / PE3 / PE4 | GPIO 轮询 + 内部上拉 | ✅ 切模式 + 手动告警，实测 1 按 1 触发 |
| 有源蜂鸣器 | PF8 | GPIO/PWM | ✅ |
| 板载 LED D0~D3 | PF9 / PF10 / PE13 / PE14 | GPIO | ⚠️ 驱动编号与宏名错位，见表下注 |
| ESP32-CAM | 独立供电与串口 | HTTP | ⚠️ 二级视觉，未参与决策 |
| SG90 舵机 | — | — | ❌ 驱动为空桩 |
| 4×4 矩阵键盘 | COL2 = PA4 | GPIO | ❌ 弃用（与 MQ-2 抢 PA4） |

> **LED 编号错位（踩过坑）**：`led.c` 的 switch case 是 **1-based**，`led.h` 的宏是 **0-based**，
> 于是 `LED_ON(1)`=PF9、`(2)`=PF10、`(3)`=PE13、`(4)`=PE14，而 `LED_ON(0)` 落 `default: break` 是空操作。
> PF9 是告警灯，主循环无告警分支每 2s `LED_OFF(1)` 清一次，不能当持久指示灯；
> PE13（板上丝印 D3）只由 MQTT 下行回调驱动，用作云端控制指示灯。

## 软件功能

| 功能 | 实现要点 | 位置 |
|---|---|---|
| 基线自学习 | 上电阻塞采 20s，算均值 ± σ 得初始阈值 | `adaptive_threshold.c:71-152` |
| 自适应火灾阈值 | `fire_th = max(mq2_base + 3σ, 400)`，学习期与运行期两处同语义 | `adaptive_threshold.c:135` `:180` |
| 防基线污染 | 偏离过大不并入 EWMA：MQ-2 >2σ、温湿度 >3σ 拒绝吸收 | `AT_Update()` `:171-181` |
| 慢速跟随 | EWMA α = 0.01，适应昼夜/季节渐变 | `adaptive_threshold.h:31` |
| 连续确认窗 | 入侵需累计 3 次命中才出告警，触发后清零重累积 | `decision_engine.c:152` |
| 防拆检测 | 8 格滑动窗口（命中 >=3 次 @8dps） | 见 `main.c` `TamperUpdate()` |
| 告警冷却 | 同类告警 5s 内不重复上报 | `esp8266.c` `SEND_COOLDOWN_MS` |
| 掉电恢复 | Flash Sector 7 存基线+模式，CRC 校验通过则跳过 20s 学习 | `config_store.c` |
| 端侧 TinyML | 4 路原始量 → 10 维特征（含偏差、倒数、平方）→ 标准化 → MLP | `ml_inference.h:89` |
| 手写 MQTT | CONNECT/SUBSCRIBE/PUBLISH/PINGREQ，QoS 0，剩余长度单字节 | `esp8266.c:552-1064` |
| 云端下行控制 | `+IPD` 两级状态机收帧 → `MQTT_OnCmdReceived` 驱动 PE13；实测 19/19 条命令全响应 | `esp8266.c:792-967`、见 `main.c` `MQTT_OnCmdReceived()` |
| **MQTT retain 灯状态自动上报** | 固件每次 MQTT 会话就绪（首次连接 / 自动重连 / NTP 重连 / CheckHealth 重建）时，往 `home/light/status` 发 `retain=1` 的 `on` / `off`；新订阅者（前端、CLI）一连上立刻收到最后一条真实状态 | `esp8266.c:100` `:700` `:1339`、`esp8266.h:96`、`main.c:128-174` |
| **MQTT retain 模式状态自动上报** | 固件每次 MQTT 会话就绪、以及收到 MQTT 模式命令 / 物理按键切模式时，往 `home/mode/status` 发 `retain=1` 的 `HOME` / `AWAY` / `SLEEP`；新订阅者一连上立刻收到最后一次真实模式 | `esp8266.c:101` `:702` `:1341`、`esp8266.h:97`、`main.c:154-164`、`main.c:438-440` |
| 运行期阈值观测 | `[DBG] MQ2/th/T` 每 10s 一条；安静环境应恒为 `th=400`，低于此值即下限失效 | 见 `main.c` 主循环 `%100` 分支 |
| NTP 对时 | UDP 123；失败回退编译时间 | `esp8266.c:1066-1243` |
| 看门狗 | IWDG 2s，基线学习完成后才启动 | 见 `main.c` `main()` 初始化段 |

## Web 前端

配套的实时监控面板，是**独立项目、位于本仓库的同级目录**，不被本仓库跟踪。

| 项 | 内容 |
|---|---|
| 位置 | `D:\Projects\workspace\smartguard-web\` |
| 技术栈 | Vue 3 + Vite + Pinia + ECharts + mqtt.js（另含 vue-router、dayjs、@lucide/vue） |
| 页面 | Dashboard / History / Settings / About（`src/views/*View.vue`） |
| 连接方式 | MQTT over WebSocket → `ws://192.168.137.1:9001` |
| **订阅 topic** | 同时订阅 `home/sensor/status`（遥测 + ALERT 告警）、`home/light/status`（固件 retain 的真实灯状态）与 `home/mode/status`（固件 retain 的真实模式状态）。后两者用于纠正按钮 / chip 状态，不再只依赖 localStorage 乐观更新 |
| **按钮同步** | `ControlPanel.vue` 同时注册 `onLightStatus` 与 `onModeStatus` 回调，前者收到 `on` / `off` 更新 `ledOn`，后者收到 `HOME` / `AWAY` / `SLEEP` 更新当前模式 chip。Broker retained 消息在 WebSocket CONNECT 后极短时间内送达，刷新页面后按钮与 chip 100ms 内显示真实状态 |
| 配置入口 | broker 地址由 `.env.local` 的 `VITE_MQTT_URL` 提供（`src/services/mqtt-client.js`），未设置则前端拒绝连接并在控制台报错 |
| 启动 | `npm run dev` → 浏览器打开 <http://192.168.137.1:8080>（`vite.config.js` 固定 `host: 0.0.0.0`、`port: 8080`） |

> ⚠️ 设备侧走 TCP 1883、Web 侧走 WebSocket 9001，是同一 broker 的两个监听端口；
> 但 9001 的 WebSocket 监听在本仓库内**没有对应的配置或启动脚本**（见"已知限制"），
> 只克隆本仓库无法直接把前端连上。

## 目录结构

```
SmartGuard/
├── Core/            启动文件 + 中断向量
├── Library/         ST 标准外设库
├── Hardware/        驱动层，一模块一目录
│   ├── APP/         ★ sensor_hub / adaptive_threshold / decision_engine
│   ├── ML/          自动生成的 MLP 权重头
│   ├── CONFIG_STORE Flash 持久化
│   └── ADC BEEP DELAY DHT11 ESP32_CAM ESP8266 HCSR04 IIC KEY LED
│      MATRIX_KEY MPU6050 MQ2 OLED PWM RTC SERVO USART WDG
├── User/            main.c + Keil 工程 Template.uvprojx
├── v1_train/        数据采集 + sklearn 训练 + 权重头生成
├── tools/           OLED 字库生成脚本
├── docs/            硬件核对清单、诊断报告、历史 README
│   └── logs/        串口验证日志（.gitignore 已忽略）
└── Object/          编译产物（已 gitignore）
```

## 快速上手

**1. 编译**（命令行全量重编译）

```bash
"D:/Keil_v5/UV4/UV4.exe" -r User/Template.uvprojx -j0 -o build_log.txt
tail -3 build_log.txt        # 期望 0 Error(s), 0 Warning(s)
```

**2. 烧录**：J-Link 接 SWD，Keil 内按 F7，或 `UV4 -f User/Template.uvprojx`。

**3. 起 broker**（仓库无脚本，需自行安装 Windows mosquitto 原生服务）

```bash
# 安装 mosquitto（官网下载 Windows 版）后，编辑
# C:\Program Files (x86)\mosquitto\mosquitto.conf，新增：
listener 1883                      # TCP 1883：固件 MQTT over AT+MQTTTCONN
listener 9001 protocol websockets  # WebSocket 9001：前端 mqtt.js over ws://
# 然后：
net start mosquitto                # 启动服务
# 浏览器连 ws://192.168.137.1:9001
# 前端订阅 home/light/status 会立即收到 broker retained
```

**4. 观察遥测**（每 10s 一条）

```bash
mosquitto_sub -h 192.168.137.1 -p 1883 -t 'home/#' -v
# home/sensor/status  T=27,H=45,D=120,L=1255
```

**5. 本地交互**：S1/S2/S3 切 HOME/AWAY/SLEEP 并落盘，S4 触发一次手动告警测试。
串口 COM11 @115200（STM32）、COM12（ESP32-CAM）；上电后 OLED 显示基线学习进度。

## 项目难点与解决

| 难点 | 为什么难 | 怎么解决 |
|---|---|---|
| 中文 Windows GBK 误解析 | 无 BOM 的 UTF-8 源文件里，中文注释会吞掉紧随的 `\r\n` 转义，报错点飘到下一行 | 所有 .c/.h 统一 UTF-8 **with BOM**；`.gitignore` 反向要求**无 BOM**，用 `git check-ignore -v` 验 |
| 屏蔽死代码引出 7 个 Error | 用未闭合 `/*` 注掉整段代码，补上闭合就"激活"了引用已删宏的代码 | 死代码一律 `#if 0 ... #endif` 隔离；每次结构性改动后立刻编译验证 |
| 固定阈值误报 | 同一阈值在白天/夜里、开灯/关灯下表现完全不同 | 20s 基线 + σ 带判异常 + EWMA 慢跟随 + 大偏离拒绝吸收 |
| HC-SR04 偶发 D=0 | 回波超时被当成 0cm，直接命中硬阈值造成误报 | 超时返回 0，决策入口判 `distance <= 0` 直接放过并衰减 hit |
| 边界地带难判 | 40~120cm 既可能是人也可能是窗帘阴影 | 该区间交给 MLP，硬阈值与自适应阈值优先级更高 |
| 每次上电重学 20s | 掉电重启后要重新等基线，演示时不友好 | Flash Sector 7 + magic + CRC32，校验通过即恢复基线与模式 |
| 无现成 MQTT 库 | ESP8266 只有 AT 通道，塞不进 SDK | 手拼固定头/剩余长度/变量头；心跳按 keepalive/2 触发，掉线重连重建会话 |
| 下行帧被吃掉 | AT 应答读取循环和 SEND OK 等待窗（50~200ms）会把期间到达的 `+IPD` 帧一并读走丢弃 | 5 处 blind flush 与 SEND OK 循环改成"字节一到就喂解析器"；`MQTT_FeedByte` 剥前缀、`MQTT_ParseFrame` 解包，带长度越界卫兵 |
| 火灾阈值反复误报 | 阈值有两个写入点，运行期那处漏掉 400 下限，安静环境塌到 ~201 后又被 2σ 吸收门锁死 | 两处统一为 `max(ewma+3σ, 400)`；实测打火机 MQ2 冲到 1082 只出 1 条告警，按旧阈值 234 会多 3 条误报 |
| 一次按压触发多次 | 按键配 `NOPULL`，空闲态全靠外部上拉，浮空被 10Hz 轮询读成伪边沿 | 改 `GPIO_PuPd_UP`；实测 2 次按压 → 2 条打印严格 1:1，不需要软件消抖 |
| 热复位后开机死循环 | IWDG 只能掉电关闭、跨热复位存活，而 `ESP8266_Reset` 的 2s 等待正好等于 2000ms 超时且零喂狗 | 改 `20×(delay_ms(100)+IWDG_Feed())`；冷启动本来安全是因为 `IWDG_Init` 排在基线学习之后 |

## 已知限制

1. **AWAY 与 SLEEP 行为一致**：模式机仅在日志与 OLED 文本上区分，判据无分支差异。
2. **火灾温度关是死代码**：DHT11 实测 20 次采样恒为 `temp=26.0 humi=41.0` → `temp_sigma=0.0`，`adaptive_threshold.c:203` 的 `temperature < temp_ewma + 2σ` 退化成 `26.0 < 26.0` = false，`&&` 短路 → 温度关对任何温度恒放行。这块板的火灾检测实质是**纯烟雾单判据**。
3. **火灾检测不受模式门控**：任何模式下 MQ-2 越限都告警（安全上保守，但非设计意图）。
4. **`loop_tick` 比真实时间慢约 16%**：主循环一圈实测 ~116ms 而非 `delay_ms(100)` 的 100ms（采样 + printf + MQTT 都计入），派生间隔同比拉长——2s 状态行实测 2.32s、10s 遥测实测 11.6s、5s 告警冷却实测 7.19s。RTC 由 NTP 同步、走时准确，漂的只有这个伪时钟。
5. **看门狗长延时只修了 1 处**：`ESP8266_Reset` 已分段喂狗，但仍有 5 处 ≥1s 的裸 `delay_ms` 不喂狗，最危险的是 `adaptive_threshold.c:87`（在 20 次学习循环里跑 20 遍）。目前不发作只因基线学习排在 `main.c:485` 的 `IWDG_Init()` 之前——把学习挪到看门狗之后、或加运行期重学基线，立刻爆。
6. **按键零消抖**：`KEY_Scan()` 是纯电平读取 + 优先级链，靠 `HandleBoardKey()` 的边沿检测兜底。内部上拉已消除浮空伪边沿（实测 1:1），机械抖动本身仍未过滤。
7. **引脚复用冲突**：PF9 同时是 LED0 与 TIM14 PWM，PE14 同时是 LED3 与 TIM1 PWM，PC13 同时是 OLED CS 与 RTC 输出。叠加上面那张 LED 错位表，`LED_ON(n)` 必须先换算成实际引脚再下结论。
8. **`AT_TEMP_RISE_PER_MIN`（`adaptive_threshold.h:30`）是死宏**，零引用，升温速率检测未实现。
9. **MQ-2 接线正常，但封装层 `MQ2_Init/MQ2_Read` 是死代码**：`Hardware/MQ2/mq2.c` 的 `MQ2_Init()` / `MQ2_Read()` 全项目零调用；实际采样走 `Hardware/ADC/adc.c:136` 的 `ADC_ReadMQ2()`，由 `sensor_hub.c:66` 每轮主循环调用。`adc.h:46` 默认 `USE_MQ2 = 1`，实测串口日志 `[DBG] MQ2=360 th=400`、基线学习 `mq2_base=324 sigma=17`，火灾分支真实启用。原 README 误写"未接线"系 v0.4 时代残留。
10. **凭据已外置但 git 历史仍含旧明文**：WiFi 凭据与巴法云 UID/TOPIC 均已外置到 gitignored 的 `wifi_config.h` / `bemfa_config.h`（模板见各自 `.example`），但 `esp8266.h` 与 `docs/README_v0.6_legacy.md` 的历史版本、以及 `origin/master` 仍含旧明文，公开仓库前必须处理；凭据已轮换。
11. **注释滞后于代码**：`decision_engine.h:8` 写 OLED 用 PB3/PB5/PB9，实际 PB0/PF11/PC13 且 PB9 是软件 I2C SDA；`iwdg.h:17` 写 1s，实际 2s；**启动横幅打印 `v0.4`，版本记录已演进到 `v0.6.2`**（源码 `main.c:451` 未修正）。
12. **MQTT 断网后恢复需 30s~5min，取决于网络状况**：这不是重试退避，而是**健康检查周期**决定的。`ESP8266_CheckHealth()` 只在 `loop_tick % 3000` 时调用（标称 300s，叠加第 4 条的 16% 偏差实测约 348s），其内部还要求 `s_send_fail_count >= 3` 才重建 MQTT；该计数在每次 `AT+CIPSEND` 拿不到 `>` 时 +1，而遥测每约 11.6s 发一次，攒满 3 次约需 35s。故断网恰好发生在健康检查之前 → 约 30~40s 恢复；刚错过一次健康检查 → 最坏要等满一个周期，约 5~5.8min。
13. **broker 为外部 Windows 原生 mosquitto 服务，仓库内无配置无启动脚本**：实测进程名 `mosquitto`（PID 5684，非 Docker），同一进程同时监听 **TCP 1883**（固件 MQTT）与 **WebSocket 9001**（前端 WebSocket）。配置文件位于 `C:\Program Files (x86)\mosquitto\mosquitto.conf`，不在本仓库也不在上级目录。仓库声称的 `cd .. && ./start_mqtt.bat` 在 `D:\Projects\workspace\` 及其上级 **全盘不存在**。读者要复现前端连接，必须**自行安装 Windows mosquitto 原生服务**，在 `mosquitto.conf` 里新增一条 `listener 9001 protocol websockets`，然后启动服务。
14. **编译验证需注意陈旧构建**：Keil UV4 的增量构建在某些情况下会使用陈旧目标文件，导致"0 Error 0 Warning"但实际未编入改动（2026-09 实测：一次构建 ZI-data 未随新增静态缓冲增长，下次全量 Rebuild 才补入 +128）。请用 `User\build_verify.bat` 验证：脚本会检查"Rebuild target"全量标记并比对所有源文件与目标文件的时间戳，避免假阳性。

## 版本记录

| 阶段 | 内容 | 提交 |
|---|---|---|
| v0.4 | 裸机采集 + 自适应阈值 + 多传感器融合 + USART1 日志 | `1786839` |
| v0.4.x | σ 掉电恢复、告警统一出口、非阻塞按键、HC-SR04 时序、RTC 实时时间、MQ-2 引脚修正 | `3df6ee1`…`1269a72` |
| 收尾 | 源文件统一 UTF-8 BOM、OLED 英文界面、目录整理 | `6a15137` `fa89681` |
| 上云 | 巴法云 TCP 遥测（后下线，`#if 0` 保留备查） | `b81dd05`…`092c58e` |
| 当前 | 自建 MQTT broker + 下行控制回调；编译警告 13→0；仓库瘦身（厂商资料移出、文档归档） | `6f6f398`…`b16511c` |
| 本轮 | MQTT 下行两级状态机 + 丢包窗口；火灾阈值 400 下限；按键内部上拉；`[DBG]` 观测行；`ESP8266_Reset` 分段喂狗 | `a227282`…`5afef93` |
| v0.6.1 | MQTT 下行控制 + 防拆检测（滑动窗口）+ OLED 主界面重构 + Web 前端 | `0dd3314` `fca83f4` |
| v0.6.2 | MQTT retain 双向状态同步（灯状态 + 模式状态）；物理按键切模式同步上报 broker；修复"模式命令污染灯状态 topic"bug | `e603dc3` |

ML 指标（`v1_train/training_report.txt`，485 样本 / 388 训练 / 97 测试）：Accuracy 0.938、Recall 1.000、F1 0.900。该报告首行写的结构 `10→16→1` 与部署头文件实际数组 `10→16→8→1` 不一致，以 `ml_inference.h` 为准。

上一版 943 行的 README（含逐条调试记录、AT 交互日志、排障问答）未删除，归档在 `docs/README_v0.6_legacy.md`。

## 许可证

课程/个人项目，暂未指定正式许可证。ST 标准外设库版权归 STMicroelectronics；SSD1306、MPU6050 驱动源自公开教学例程并重写。
