# SmartGuard · 智能空间守护系统

> STM32F407ZE 裸机 + 6 路传感器 + 端侧自适应阈值 + 手写 MQTT 上云的空间守护原型

- **一句话**：设备自己学出"这个房间正常是什么样"，只在统计显著偏离时告警，本地蜂鸣/显示的同时把状态推上云。
- **不做**：不依赖云端推理、不跑 RTOS（协作式任务调度器）。
- **芯片**：STM32F407ZE（Cortex-M4 · 168 MHz · 192 KB RAM · 1 MB Flash）
- **构建**：Keil MDK v5 + J-Link，当前 `0 Error(s), 0 Warning(s)`
- **配套**：支持 MQTT 双向通信 + Web 前端（Vue 3）实时监控面板

## 项目背景

常见的门禁/烟雾报警器有三个通病：**阈值写死**（换个房间就得重调）、**误报多**（一片阴影扫过就响）、**断网即失效**。SmartGuard 想验证的做法是把决策留在端侧——先学环境基线，只在显著偏离时告警，网络只负责通知。

第二个目的是工程训练：手写 MQTT 3.1.1 报文而不是接 SDK，用板载 EEPROM/SPI Flash 手写存储层（环形日志、撕裂校验）而不是接现成文件系统，把每一类编译警告归零。这些"重复造轮子"的部分，恰好是项目里最值得讲的部分。

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
持久层   AT24C02 配置掉电恢复（magic+CRC32）· W25Qxx 告警事件环形日志 · VBAT 备份域 RTC
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
| AT24C02 EEPROM 256B | PB8/PB9（与 MPU6050 同总线，0x50） | 软件 I2C | ✅ 配置掉电存储 |
| W25Qxx SPI Flash | CS PB14 / SCK PB3 / MISO PB4 / MOSI PB5 | 软件 SPI | ✅ 告警事件日志（16KB/1024 条） |
| ESP8266 ESP-12F | TX PB10 / RX PB11 | USART3 AT | ✅ MQTT 打通 |
| 板载按键 S1~S4 | PA0 / PE2 / PE3 / PE4 | GPIO 轮询 + 内部上拉 | ✅ 四键全部**低电平有效**（含 PA0，实测为准；课程模板注释称 WK_UP 高有效，与本板不符） |
| 无源蜂鸣器 | PF8 | TIM13_CH1 PWM | ✅ 分级告警音（入侵三连音/火灾双音/防拆长音） |
| 板载 LED D0~D3 | PF9 / PF10 / PE13 / PE14 | GPIO | ⚠️ 驱动编号与宏名错位，见表下注 |
| ESP32-CAM | USART2 PA2/PA3 + 独立 5V 供电 | 串口指令 + HTTP | ✅ 告警抓拍 + 12 张历史环（seq 序号）+ MJPEG 实时流 |
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
| 掉电恢复 | AT24C02 存基线+模式（magic+CRC32，双后端可切回内部 Flash），跳过 20s 学习 | `config_store.c` |
| 告警事件日志 | W25Qxx 环形追加写 16B/条（1024 条容量），断电黑匣子，开机回显最近 5 条 | `event_log.c` |
| 存储自证自愈 | 开机"擦→写→读回→比对"自检；SR1/SR2/SR3 诊断，BP/CMP/WPS 写保护位自动解除（共享课程板常见遗留锁定，本板实测 WPS=1 全阵列锁定）；写后读回校验 | `event_log.c` `w25qxx.c` |
| 备份指针交叉验证 | 备份 SRAM 指针恢复后抽查末条记录，与 FLASH 实际内容脱节自动回退 4 扇区全扫描 | `event_log.c` |
| 远程抓拍/告警测试 | MQTT 命令 `CAPTURE`（立即拍照并 retain 回推 URL）、`ALARM`（等价 S4 短按，走完整告警链路） | `main.c MQTT_OnCmdReceived()` |
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
| 页面 | Dashboard / History / About（`src/views/*View.vue`，Settings 已移除） |
| 看板交互 | CAM 卡片：实时 MJPEG（fetch+AbortController 拉流，Vite 代理同源）↔ 12 张抓拍胶片条（seq 终身序号 + HTTP 缓存，切换秒开）；告警时间线：类型/时间双重筛选、≤60s 连续同型聚为事件簇、中文展示；History：分层存储（2h×10s 原始点 + 7d×5min 均值桶），距离曲线叠加告警竖标 |
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
| 命令 flood 打挂 MQTT 会话 | 网页 LED 按钮快速连点 → 下行命令 flood → 固件每条命令都 retain 上报 → 上报把 `SEND OK` 应答窗口挤掉 → 固件误判 session lost 重连，重连窗口内"上行停、下行活"（表现为 DEV 离线却能控灯） | Web 端 `ControlPanel.vue` 加 300ms 命令节流（TOO FAST 提示 + 乐观翻转回滚）；同类隐患与加固方案见下节"同类隐患排查" |

## 同类隐患排查（命令/请求洪峰）

由 LED flood 事故（2026-09-22，现象"DEV 离线但可控灯"）引发的全栈复查，按"用户操作/定时任务能否无节制地发命令、发请求、写存储"逐一审计。✅ = 已修复，⏳ = 方案已定待实施，ℹ️ = 有风险但可接受。

### Web 前端（smartguard-web）

| # | 风险点 | 说明 | 解决方案 | 状态 |
|---|---|---|---|---|
| W1 | LED/MODE/ACTIONS 按钮连点 | 快速连点造成 MQTT 命令 flood，是本次事故直接原因 | `sendCmd()` 统一 300ms 节流，被拦截提示 `TOO FAST`；LED 乐观翻转在发送失败时回滚 | ✅ |
| W2 | `selectMode` 乐观更新不回滚 | 命令被节流拒发时 mode chip 已切换且已写 localStorage，UI 与设备不一致，要等固件下次 retain 上报才纠正 | 与 `toggleLed` 对齐：`sendCmd` 返回 false 时回滚 `mode` 再 `persistState()` | ✅ |
| W3 | `CamShotCard` 404 自愈补拍绕过节流 | 缩略图 404 时直接调裸 `mqtt.publish('CAPTURE')`，不走 `sendCmd` 的 300ms；broker retain 的 seq 超前于 CAM 环内容时（环被刷/重烧），每张新 URL 404 都会再补一条 CAPTURE，而固件侧每条 CAPTURE 离线时阻塞主循环约 4s | 三重防 flood：同一 URL 只补一次 + 最小补拍间隔 10s + 连续 3 次补拍仍 404 放弃自愈（成功出图清零计数） | ✅ |
| W4 | LIVE 快速 ON→OFF→ON 叠加拉流 | OFF 时旧 `liveLoopRun` 正阻塞在 `reader.read()`，`abort()` 的异常到达前若已重新 ON，旧循环 while 条件仍成立继续跑，与新循环并存 → 两条 `/stream` 同时打到单客户端 ESP32-CAM，第二个连接最长排队 300s | 拉流代数 `liveRunId`：每次切换递增，旧循环每轮迭代校验代数、失代立即退出 | ✅ |
| W5 | 定时器叠加 | 已审计全部 `setInterval/setTimeout`（TopBar 1s、AppLayout 450ms、hint 2s、toast 3s、sensor.js 1s debounce 写盘），均"先 clear 再建"或单例组件 | 无需处理 | ℹ️ |
| W6 | localStorage 高频写入 | 遥测 10s/条触发约 300KB 全量写（有 1s debounce 合并），告警受固件 5s 冷却限制，实时帧只走 blob URL 不落盘 | 频率可接受；若低端机 jank 可把 history 写盘改 `requestIdleCallback` | ℹ️ |

### 固件（STM32F407）

| # | 风险点 | 说明 | 解决方案 | 状态 |
|---|---|---|---|---|
| F1 | **下行命令零去重零限速** | `MQTT_OnCmdReceived()` 是 flood 事故在固件侧的镜像：修复前唯一防线是 Web 端 300ms 节流，任何其他 MQTT 客户端（`mosquitto_pub`、第二个浏览器标签）都可绕过 | **命令不丢、只合并上报**：on/off/HOME/AWAY/SLEEP 立即生效，retain 上报 500ms 最多一条（`ReportLightCoalesced`/`ReportModeCoalesced`），被合并的挂起由 `task_fast` 的 `CmdReportFlush()` 补报最后一态（最终一致）；CAPTURE 硬限速 5s 直接丢弃；ALARM 靠单槽覆盖+5s 冷却天然去重 | ✅ |
| F2 | CAPTURE 阻塞主循环 ~4s | CAM 离线时 1s 探测 + 3s 等待，协作调度器整体冻结（仅靠 ReadLine 内喂狗撑住 IWDG）；连发 10 条 ≈ 40s 冻结 | 已受 F1 的 CAPTURE 5s 硬限速保护（连发最坏退化为每 5s 一次 4s 阻塞，不再持续冻结）；进一步可改状态机非阻塞（探测→等待→取图拆成三个 tick 阶段） | ✅（限速兜底）/ ⏳（非阻塞化留答辩后） |
| F3 | `s_pending_cmd` 单槽覆盖 | 连发时后命令静默覆盖先命令，无队列无 NACK | 与 F1 的限速配合后即天然缓解；如需可靠可改 2~4 槽环形队列 | ℹ️ |
| F4 | 已有正面防护 | 告警 5s 重复冷却、EventLog 仅状态翻转落盘、Config 脏标志 1s 合并写、60s 重连退避、遥测 10s/心跳 30s 节拍 | — | ✅ |
| F5 | SEND OK 等待窗不容忍 +IPD 交错 | 2026-09-22 flood 实测：F1 已把 20 条命令的上报压到 2 条，但其中 1 条上报的 SEND OK 等待窗仍被 flood 尾包的 `+IPD` 挤掉 → 误判 session lost 重连一次（~2s 自愈并重报最终态，功能无损）。根因是"下行帧与 SEND OK 窗口碰撞"的残留形态 | `ESP8266_MQTTPublish` 的 SEND OK 等待循环在窗口内收到 `+IPD` 时继续等 SEND OK（字节喂解析器但不提前判失败），超时后再判丢失 | ⏳（答辩后） |

**F1 实测记录（2026-09-22，mosquitto_pub 连发 20 条交替 on/off）**：20 条命令仅触发 2 条 retain 上报（首条 on + 末条 off，合并生效）；中间 18 条被 `s_pending_cmd` 单槽覆盖（状态型命令可接受）；最终态收敛正确（retained = 末条命令 off）；F5 残留的碰撞导致 1 次会话重连，~2s 自愈并自动重报灯态/模式，系统功能全程无损。

### ESP32-CAM

| # | 风险点 | 说明 | 解决方案 | 状态 |
|---|---|---|---|---|
| C1 | `/snapshot` HTTP 端点无节流无鉴权 | 同网段任何客户端可连续触发拍照，把 12 张历史环快速刷掉（真正的入侵证据被顶出环），并与串口 CAPTURE 抢 camera fb | 端点加最小间隔（如 2s，间隔内返回 429 或上一次缓存帧）；固件只在受信任热点内运行，鉴权非必需 | ⏳（答辩后） |
| C2 | 串口 CAPTURE 无速率限制 | `g_snapshot_busy` 防并发 + STM32 同步等响应使命令天然串行化，打爆风险在 STM32 侧（见 F1/F2）而非此处 | 随 F1 落地即闭环 | ℹ️ |
| C3 | `/stream` 单线程排队 | 已有 300s 上限 + 写失败快速退出；第二个 /stream 连接最长排队 300s（与 W4 叠加时 Vite 代理连接挂起） | 随 W4 落地即闭环 | ℹ️ |

**剩余优先级**：F5 > C1 > F2 非阻塞化 > 其余（W2/W3/W4 + F1 限速已于 2026-09-22 修复并实测，固件编译 0 Error 0 Warning 验证通过）。至此命令 flood 链路形成 Web 节流（第一道）+ 固件上报合并/CAPTURE 硬限速（第二道）双层防线；F5 是 flood 下仍会偶发一次重连的最后残留。

## 已知限制

1. ~~**AWAY 与 SLEEP 行为一致**~~（v0.7.1 已补全）：SLEEP 现在关闭超声波入侵检测（起夜不误报）与自动照明（睡觉不亮灯），保留防拆与火灾；HOME 关入侵/防拆、照明自动；AWAY 全布防。
2. **火灾温度关是死代码**：DHT11 实测 20 次采样恒为 `temp=26.0 humi=41.0` → `temp_sigma=0.0`，`adaptive_threshold.c:203` 的 `temperature < temp_ewma + 2σ` 退化成 `26.0 < 26.0` = false，`&&` 短路 → 温度关对任何温度恒放行。这块板的火灾检测实质是**纯烟雾单判据**。
3. **火灾检测不受模式门控**：任何模式下 MQ-2 越限都告警（安全上保守，但非设计意图）。
4. ~~**`loop_tick` 比真实时间慢约 16%**~~（已修复）：主循环已改为协作式任务调度器（`scheduler.c`，周期基于真实 `g_tick`，空闲 `__WFI`），时漂消除；2s 决策/10s 遥测/30s 心跳均为真实时间。
5. ~~**看门狗长延时只修了 1 处**~~（v0.7.1 已修）：`AT_LearnBaseline` 学习循环已分段喂狗；`ESP8266_Reset` 等长等待均有界喂狗。
6. ~~**按键零消抖**~~（v0.7.1 已修）：`HandleBoardKey()` 增加 50ms 消抖窗，S4 双击误触发已消除；S4 改为短按（松开 <1.5s）手动告警、长按 ≥1.5s 回放事件日志。
7. **引脚复用冲突**：PF9 同时是 LED0 与 TIM14 PWM，PE14 同时是 LED3 与 TIM1 PWM，PC13 同时是 OLED CS 与 RTC 输出。叠加上面那张 LED 错位表，`LED_ON(n)` 必须先换算成实际引脚再下结论。
8. ~~**`AT_TEMP_RISE_PER_MIN` 是死宏**~~（v0.7.1 已删除）。
9. ~~**MQ2 封装层是死代码**~~（已移除）：实际采样走 `Hardware/ADC/adc.c` 的 `ADC_ReadMQ2()`，`adc.h` 默认 `USE_MQ2 = 1`，火灾分支真实启用。
10. **凭据已外置但 git 历史仍含旧明文**：WiFi 凭据与巴法云 UID/TOPIC 均已外置到 gitignored 的 `wifi_config.h` / `bemfa_config.h`（模板见各自 `.example`），但 `esp8266.h` 与 `docs/README_v0.6_legacy.md` 的历史版本、以及 `origin/master` 仍含旧明文，公开仓库前必须处理；凭据已轮换。
11. **注释滞后于代码**：`decision_engine.h:8` 写 OLED 用 PB3/PB5/PB9，实际 PB0/PF11/PC13 且 PB9 是软件 I2C SDA；`iwdg.h:17` 写 1s，实际 2s。~~启动横幅印 `v0.4`~~（已修正为 v0.7）。
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
| v0.7 | 协作式调度器对齐课程模板（任务表注册/按名启停/`__WFI`）；存储四层重构：AT24C02 配置 + W25Qxx 告警事件环形日志 + VBAT RTC；入侵拍照全链路（ESP32-CAM → MQTT retain → Web 出图）；P3 加固（分段喂狗、snprintf、TXE/TC 有界等待、MQTT 跨帧拼包、重连等待钩子续采防拆） | — |
| v0.7.1 | SLEEP 语义补全（关入侵/照明，留防拆/火灾）；备份 SRAM 写指针；分级告警音；S4 消抖 + 长按回放日志；死宏清理 | — |
| v0.7.2 | 按键极性实测修正（四键全低有效，含 PA0）；W25Qxx 写保护自愈（SR 诊断 + BP/CMP/WPS 解除 + 写自检 + 读回校验 + 指针交叉验证）；CAM 12 张历史环 + seq 终身序号 + HTTP 缓存头；MQTT `CAPTURE`/`ALARM` 远程命令；Web 看板重构（实时/胶片条、聚合时间线、History 7 天分层、About 项目文档页） | — |

ML 指标（历史 TF 管线报告，485 样本 / 388 训练 / 97 测试）：Accuracy 0.938、Recall 1.000、F1 0.900。TF 管线（`train.py` + STM32Cube.AI 路线）已删除——它与 sklearn 管线写同一个 `scaler_params.h` 但符号不兼容；现行唯一管线为 `v1_train/train_sklearn.py`（结构 10→16→8→1，与部署的 `ml_inference.h` 一致），最新指标以重新运行 `train_sklearn.py` 输出的分类报告为准。

上一版 943 行的 README（含逐条调试记录、AT 交互日志、排障问答）未删除，归档在 `docs/README_v0.6_legacy.md`。

## 许可证

课程/个人项目，暂未指定正式许可证。ST 标准外设库版权归 STMicroelectronics；SSD1306、MPU6050 驱动源自公开教学例程并重写。
