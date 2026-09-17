# SmartGuard P0 修复 构建版本与测试日志归档

> 归档时间：2026-09-16
> 用途：保存 P0（MQTT 半死恢复 + ESP32-CAM IWDG）修复的构建版本信息与原始测试证据。
> 注意：原始 `mqtt_watch.log` 已在测试收尾时删除，本文件保留了全部关键原始时间戳与日志行。

## 1. 构建版本

| 项 | 值 |
|---|---|
| Git HEAD | `bacac5ab0319a2f8bcb010df028ffdd3400e0765` |
| Commit 时间 | 2026-09-16 20:13:53 +0800 |
| 修改文件 | `Hardware/ESP8266/esp8266.c`（+42/-14）、`Hardware/ESP32_CAM/esp32_cam.c`（+5/-0） |
| 构建方式 | `User/build_verify.bat`（Keil UV4 -r 全量重建 + 时间戳审计） |
| Program Size | `Code=35612 RO-data=6200 RW-data=176 ZI-data=9832` |
| Error / Warning | 0 Error / 1 Warning（oled.c 既有未引用函数，非本次引入） |
| 栈 / 堆 | Stack 8KB（startup_stm32f40_41xxx.s:41）/ Heap 512B |
| 烧录 | J-Link `loadfile Object/Template.hex` O.K. |

## 2. P0-1 MQTT 断网/恢复测试（2026-09-16 19:23–20:14）

测试方法：Node MQTT watcher（ws://192.168.137.1:9001 订阅 home/sensor/status）+ J-Link 内存读数（s_mqtt_ok / s_send_fail_count / s_last_attempt / 任务表）。

原始关键日志行（按时间）：

```
[19:23:30] watcher start
[19:23:31] broker connected
[19:23:38] (+8s)   T=26,H=62,D=270,L=1431,M=616     ← 首条遥测
[19:26:28] (+178s) ALERT:Intrusion!                  ← 真实入侵告警（后续 19:27:52 / 19:28:10 / 19:28:26 共 4 条，含 D=3/7/30cm）
[19:31:08] (+458s) T=26,H=60,D=269,L=1480,H=673      ← 断网前最后一条遥测
[19:31:16] broker connection closed                  ← broker 停止（services.msc 手动停止 Mosquitto Broker 服务）
[19:31:19]+ watcher 进入每 ~3s reconnecting/closed 循环（持续至 19:58:21）

板端（J-Link 内存证据）：
  ~19:31:26   s_mqtt_ok: 1 → 0（首次失败检出，约断网后 10s）
              s_send_fail_count: 0 → 1
              s_last_attempt = 469,280（首次完整重连尝试）
  后续采样    s_last_attempt 两次采样均落在 60s 退避网格
              （uptime 381,826 时 s_last_attempt=364,921 等）
  断网全程    任务表 last_ms 持续推进，27 分钟无复位、调度器存活

[19:58:24] broker connected（watcher 侧恢复，用户重启服务）
[19:58:59] (+2129s) T=26,H=62,D=317,L=1562,M=437     ← broker 恢复后首条遥测
[19:59:06]+ 遥测恢复 10s 周期，无缺口、无重复
```

**恢复耗时**：broker 恢复（19:58:24）→ 遥测恢复（19:58:59）= **35 秒**（≤60s 退避 + 握手 + 10s 遥测周期，符合设计）。

**断网检测耗时**：断网（19:31:16）→ 会话标记失效 ≈ 10 秒。

## 3. P1 MQTT 下行回归测试（2026-09-16 20:39–20:40）

原始日志（mqtt_downlink_test.cjs 实测输出，全量）：

```
[20:39:57] connected, subscribing status topics
[20:39:57] <- home/light/status: off          ← retained 初始状态 ✓
[20:39:57] <- home/mode/status: AWAY          ← retained 初始状态 ✓
[20:39:58] -> publish home/light/cmd "off"
[20:39:59] <- home/light/status: off          (1s 内回显)
[20:40:01] -> publish home/light/cmd "on"
[20:40:02] <- home/light/status: on           (1s 内回显，LED 实际点亮)
[20:40:04] -> publish home/light/cmd "HOME"
[20:40:05] <- home/mode/status: HOME          (1s 内回显)
[20:40:06] <- home/sensor/status: T=26,H=60,D=270,L=1439,M=392
[20:40:07] -> publish home/light/cmd "AWAY"
[20:40:08] <- home/mode/status: AWAY          (1s 内回显)
[20:40:10] DONE. lightEcho=["off","off","on"] modeEcho=["AWAY","HOME","AWAY"]
```

结论：下行命令 → 固件执行 → retained 状态回读，全闭环 4/4 通过，延迟均 ≤1s。

## 4. 复位归因记录（重要，避免后人误判）

测试期间板子出现 3 次复位，**全部确认为审计方 J-Link halt 调试动作诱发**（halt 冻结 SysTick 与任务，IWDG 按 LSI 墙钟继续走时，≥2s 未喂即复位），与固件无关。时间点：19:33:48、19:47:55 各一次紧随探针操作。工程建议：调试构建可设 `DBGMCU_APB1_FZ` 的 `DBG_IWDG_STOP` 位，使 halt 时冻结看门狗。

## 5. 未验证项（截至归档时）

| 项 | 原因 |
|---|---|
| ESP32-CAM 在线 3s 拍照路径（ReadLine 3000ms） | 无 CAM 硬件接入 |
| CheckHealth fail_count>=3 分支实际触发 | 需 broker 停机 ~7 分钟 + 内存采样 |
| OLED 画面内容 | 板载 CH340↔PA9 串口链路物理断开（J-Link 注入字节零接收实证），画面未目视 |
| 断网慢启动完整根因 | 网络正常时启动 30-60s 已实测；极端慢启动（>15min）放大因子未完全定位 |
