/**
  * @file    main.c
  * @brief   SmartGuard v0.7.2 — 智能空间守护系统主程序
  *
  * 硬件清单 (已全部验证):
  *   ADC         — 光敏电阻 PF7/ADC3_IN5, MQ-2 烟雾 PA4/ADC1_IN4
  *   HCSR04      — Trig=PC8, Echo=PC6
  *   DHT11       — PG9 (一次读回 temp+humi)
  *   BoardKey    — S1=PA0, S2=PE2, S3=PE3, S4=PE4 (板载按键，全部低电平有效)
  *   LED         — PF9/PF10/PE13/PE14 (低电平亮), BEEP: PF8 (TIM13 PWM)
  *   OLED SSD1306 — PB0=CLK PF11=MOSI PB1=DC PC13=CS1, CS2接VCC
  *   MPU6050     — 软件 IIC PB8/PB9 (0x68)，与 AT24C02(0x50) 同总线
  *   W25Qxx      — 软件 SPI CS=PB14 SCK=PB3 MISO=PB4 MOSI=PB5
  *   ESP8266     — USART3 PB10/PB11, 115200bps
  *   ESP32-CAM   — USART2 PA2/PA3, 115200bps
  */

/* ============ 编译开关（必须在 include 之前定义）============ */
#define USE_BOARD_KEY   1
#define USE_MPU6050     1   /* 0=未接 MPU6050  1=已接，启用 6 轴传感器 */


/* ============ TinyML 数据采集 ============ */
/* 1=每 2s 输出 CSV 行到串口 + ESP8266 UDP，用于后期训练 sklearn MLP */
#define ML_DATA_COLLECT  0

#include "stm32f4xx.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "adc.h"
#include "hcsr04.h"
#include "dht11.h"
#if USE_BOARD_KEY
#include "key.h"
#endif
#include "sensor_hub.h"
#include "decision_engine.h"   /* 已经 include USE_OLED/USE_ESP8266 定义 */
#include "adaptive_threshold.h"
#if USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#else
#include "scheduler.h"
#endif
#include "oled.h"
#include "iwdg.h"
#include "config_store.h"
#include "at24c02.h"
#include "w25qxx.h"
#include "event_log.h"
#include <math.h>          /* fabsf：TamperUpdate 滑动窗口判定用 */

#if USE_RTC
#include "rtc.h"
#endif

#if USE_ESP8266
#include "esp8266.h"
#include <string.h>      /* strcmp for MQTT_OnCmdReceived */
#endif

#include "esp32_cam.h"  /* 必须在 #if USE_ESP32_CAM 之前包含：宏定义在此头文件中 */

#if USE_MPU6050
#include "mpu6050.h"
#endif

    /* ========== 假数据测试模式（测试用，测完请改 TEST_FAKE_INPUT 为 0） ========== */
    #define TEST_FAKE_INPUT      0

    #if TEST_FAKE_INPUT
    static const float s_fake_dist_quiet[] = {
        250,250,250, 248,248,248, 252,252,252,
        249,249,249, 251,251,251, 250,250,250,
        50,50,50,
        250,250,250,
        137,137,137,
        250,250,250,
        52,52,52,
        250,250,250
    };

    static const float s_fake_dist_intrude[] = {
        245,245,245, 245,245,245,
        14,14,14,
        56,56,56,
        24,24,24,
        56,56,56,
        12,12,12
    };

    static const float *s_fake_seq = s_fake_dist_intrude;
    static uint8_t s_fake_seq_len =
        sizeof(s_fake_dist_intrude) / sizeof(s_fake_dist_intrude[0]);
    static uint8_t s_fake_idx = 0;

    void Test_SwitchSeq(uint8_t which)
    {
        if (which == 1) {
            s_fake_seq = s_fake_dist_quiet;
            s_fake_seq_len = sizeof(s_fake_dist_quiet) / sizeof(s_fake_dist_quiet[0]);
        } else if (which == 2) {
            s_fake_seq = s_fake_dist_intrude;
            s_fake_seq_len = sizeof(s_fake_dist_intrude) / sizeof(s_fake_dist_intrude[0]);
        }
        s_fake_idx = 0;
    }
    #endif

    /* ============ 全局状态 ============ */
static Sensor_Data_t g_sensor;
static Guard_Mode_t  g_mode    = MODE_AWAY;   /* 默认离家模式：入侵检测开启 */
static Alert_Type_t  g_alert   = ALERT_NONE;
/* 按键/MQTT 手动触发的测试告警。跨任务：comm 任务写、decision 任务读，
   enum 单字访问在 M4 上原子，加 volatile 防止编译器缓存到寄存器。 */
static volatile Alert_Type_t g_manual_alert = ALERT_NONE;
static uint32_t      g_last_alarm_tick = 0;
/* 长按 S3 置位：请求决策任务重新学习 20s 基线（换房间/进纸箱演示）。
   fast 任务写、decision 任务读，单字节标志用 volatile 即可，无需队列。 */
static volatile uint8_t g_relearn_request = 0;

#if USE_FREERTOS
/* ============ FreeRTOS 对象（main 启动早期创建，任务启动前就绪） ============ */

/* 按键任务 -> 通信任务、决策任务 -> 通信任务 的事件队列。
   设计原因：ESP8266(USART3 AT) 与 ESP32-CAM(USART2) 的所有访问必须串行化，
   原来的协作式调度天然互斥，改成抢占后统一收敛到 TaskComm 一个任务里执行。 */
typedef enum {
    EVT_KEY_MODE = 1,   /* 物理按键切换模式，value=Guard_Mode_t：需补报 broker */
    EVT_ALERT    = 2    /* 告警事件，value=Alert_Type_t：MQTT 上报 + 入侵抓拍 */
} comm_evt_type_t;

typedef struct {
    uint8_t type;       /* comm_evt_type_t */
    uint8_t value;      /* 事件参数 */
} comm_evt_t;

static QueueHandle_t     s_comm_queue = (QueueHandle_t)0;
static SemaphoreHandle_t s_i2c_mutex  = (SemaphoreHandle_t)0;  /* MPU6050/AT24C02 共线 */
static SemaphoreHandle_t s_oled_mutex = (SemaphoreHandle_t)0;  /* OLED 软件 SPI */

/* 任务优先级（数字越大越高；0 为 idle，configMAX_PRIORITIES=7） */
#define PRIO_FAST       4   /* 按键 + 看门狗 */
#define PRIO_TAMPER     3   /* 防拆 5Hz */
#define PRIO_DECISION   2   /* 采集决策 2s */
#define PRIO_COMM       2   /* 网络收发（与 decision 同优先级，时间片轮转） */
#define PRIO_DISPLAY    1   /* OLED/RTC 1s */

/* 任务栈深（单位 word=4B）：comm 需要最大（MQTT/NTP 帧拼包路径局部变量多） */
#define STACK_FAST      512
#define STACK_TAMPER    320
#define STACK_DECISION  640
#define STACK_COMM      1024
#define STACK_DISPLAY   640

static void PostCommEvent(uint8_t type, uint8_t value)
{
    comm_evt_t evt;
    evt.type  = type;
    evt.value = value;
    if (s_comm_queue != (QueueHandle_t)0)
    {
        /* 满了直接丢（告警 5s 冷却会重发；模式状态有 retain 最终一致兜底） */
        (void)xQueueSend(s_comm_queue, &evt, 0);
    }
}
#endif /* USE_FREERTOS */

#if USE_OLED
static char          s_time_buf[10] = "00:00:00";   /* 当前时间，OLED 状态栏用 */
static const char   *s_wifi_status  = "N ";         /* 默认无 WiFi 图标 */
static const char   *s_mqtt_status  = "M-";         /* MQTT 会话状态，主界面 Row 1 用 */
static uint8_t       s_cam_ok       = 0;            /* ESP32-CAM 拍照结果 */
#endif

/* MPU6050 零偏校准值（初始化时学习） */
static float   s_gyro_zero_gx = 0.0f;
static float   s_gyro_zero_gy = 0.0f;
static float   s_gyro_zero_gz = 0.0f;

/* ============ MQTT 回调（替代原巴法云） ============ */

/* 灯状态跟踪（供 MQTT_ReportLightStatus 和 MQTT_OnCmdReceived 使用）：
   on/off 命令控制的是 ESP32-CAM 补光灯（Flash LED, GPIO4）；
   初值 'off' 与 CAM 固件上电默认关灯一致。 */
static const char *g_light_state = "off";

/* ---- 下行命令加固（2026-09-22 LED flood 事故后） ----
   事故链：网页连点 → 命令 flood → 每条命令都 retain 上报 → 上报把 SEND OK
   应答窗口挤掉 → 固件误判 session lost 重连（"DEV 离线却能控灯"）。
   Web 端 300ms 节流只是第一道防线，任何其他 MQTT 客户端（mosquitto_pub、
   第二个浏览器标签）都可绕过，此处为固件侧第二道防线。
   原则：命令不丢（立即生效，UI 与设备不脱节），只合并 retain 上报——
   灯/模式上报 500ms 最多一条，被合并的挂起，由 task_fast 在窗口过后
   补报最后一态（保证最终一致）。
   CAPTURE 例外：单次执行最坏阻塞主循环 ~4s（1s 探测 + 3s 等待），
   硬限速 5s，间隔内直接丢弃（拍照非状态同步，丢弃安全）。
   ALARM 不加固：g_manual_alert 单槽覆盖 + 决策层 5s 冷却已天然去重。 */
#define CMD_REPORT_MIN_GAP_MS   500u
#define CMD_CAPTURE_MIN_GAP_MS  5000u
static uint32_t s_light_rep_tick    = 0;
static uint8_t  s_light_rep_pending = 0;
static uint32_t s_mode_rep_tick     = 0;
static uint8_t  s_mode_rep_pending  = 0;
static uint32_t s_last_capture_tick = 0;

static void ReportLightCoalesced(void)
{
    if ((g_tick - s_light_rep_tick) >= CMD_REPORT_MIN_GAP_MS) {
        s_light_rep_tick = g_tick;
        ESP8266_MQTTPublish(MQTT_TOPIC_STATUS, g_light_state, 1);
    } else {
        s_light_rep_pending = 1;   /* 窗口内连续翻转：挂起，稍后补报最终态 */
    }
}

static void ReportModeCoalesced(void)
{
    if ((g_tick - s_mode_rep_tick) >= CMD_REPORT_MIN_GAP_MS) {
        s_mode_rep_tick = g_tick;
        MQTT_ReportModeStatus();
    } else {
        s_mode_rep_pending = 1;
    }
}

/* task_fast 每轮调用：挂起的上报在窗口过后补报（最终一致性兜底） */
static void CmdReportFlush(void)
{
    if (s_light_rep_pending && (g_tick - s_light_rep_tick) >= CMD_REPORT_MIN_GAP_MS) {
        s_light_rep_pending = 0;
        s_light_rep_tick = g_tick;
        USART1_Printf("[MQTT] REPORT light_state=%s (coalesced)\r\n", g_light_state);
        ESP8266_MQTTPublish(MQTT_TOPIC_STATUS, g_light_state, 1);
    }
    if (s_mode_rep_pending && (g_tick - s_mode_rep_tick) >= CMD_REPORT_MIN_GAP_MS) {
        s_mode_rep_pending = 0;
        s_mode_rep_tick = g_tick;
        MQTT_ReportModeStatus();
    }
}

/* ESP8266 下行控制指令回调：收到 home/light/cmd 的 PUBLISH 时调用 */
void MQTT_OnCmdReceived(const char *payload)
{
    /* on/off = 控制 ESP32-CAM 补光灯（板载 Flash LED，GPIO4），
       经 USART2 发 LEDON/LEDOFF 指令；CAM 确认后才更新状态并上报。
       失败时状态不变但仍上报（把网页乐观翻转拉回真实状态）。 */
    if (strcmp(payload, "on") == 0) {
#if USE_ESP32_CAM
        if (ESP32CAM_SetLed(1) == CAM_OK) {
            g_light_state = "on";
            USART1_Printf("[MQTT] CMD=on -> CAM LED ON\r\n");
        } else {
            USART1_Printf("[MQTT] CMD=on -> CAM LED FAIL\r\n");
        }
#else
        LED_ON(3);
        g_light_state = "on";
        USART1_Printf("[MQTT] CMD=on\r\n");
#endif
        ReportLightCoalesced();
    }
    else if (strcmp(payload, "off") == 0) {
#if USE_ESP32_CAM
        if (ESP32CAM_SetLed(0) == CAM_OK) {
            g_light_state = "off";
            USART1_Printf("[MQTT] CMD=off -> CAM LED OFF\r\n");
        } else {
            USART1_Printf("[MQTT] CMD=off -> CAM LED FAIL\r\n");
        }
#else
        LED_OFF(3);
        g_light_state = "off";
        USART1_Printf("[MQTT] CMD=off\r\n");
#endif
        ReportLightCoalesced();
    }
    else if (strcmp(payload, "HOME") == 0) {
        g_mode = MODE_HOME;  DecisionEngine_SetMode(g_mode);
        USART1_Printf("[MQTT] CMD=HOME\r\n");
        ReportModeCoalesced();
    }
    else if (strcmp(payload, "AWAY") == 0) {
        g_mode = MODE_AWAY;  DecisionEngine_SetMode(g_mode);
        USART1_Printf("[MQTT] CMD=AWAY\r\n");
        ReportModeCoalesced();
    }
    else if (strcmp(payload, "SLEEP") == 0) {
        g_mode = MODE_SLEEP; DecisionEngine_SetMode(g_mode);
        USART1_Printf("[MQTT] CMD=SLEEP\r\n");
        ReportModeCoalesced();
    }
    else if (strcmp(payload, "CAPTURE") == 0) {
        /* 硬限速 5s：单次拍照最坏阻塞主循环 ~4s，连发会冻结调度器 */
        if ((g_tick - s_last_capture_tick) < CMD_CAPTURE_MIN_GAP_MS) {
            USART1_SendString("[MQTT] CMD=CAPTURE dropped (5s rate limit)\r\n");
        } else {
        s_last_capture_tick = g_tick;
        /* 网页手动抓拍：不等告警，立即拍一张并推 URL（retain） */
#if USE_ESP32_CAM
        char cam_url[96];
        ESP32CAM_Status_t cam_st = ESP32CAM_TriggerCapture(cam_url, sizeof(cam_url));
        if (cam_st == CAM_OK) {
            USART1_Printf("[MQTT] CMD=CAPTURE photo: %s\r\n", cam_url);
            ESP8266_MQTTPublish(MQTT_TOPIC_CAM_SHOT, cam_url, 1);
        } else {
            USART1_Printf("[MQTT] CMD=CAPTURE failed (%d)\r\n", cam_st);
        }
#endif
        }
    }
    else if (strcmp(payload, "ALARM") == 0) {
        /* 网页告警测试：等价板载 S4 短按，走完整告警链路
           （蜂鸣器+LED+拍照+MQTT+W25Qxx 落盘），由 task_decision 统一出口消费 */
        g_manual_alert = ALERT_INTRUSION;
        g_last_alarm_tick = 0;
        USART1_SendString("[MQTT] CMD=ALARM (manual alert test)\r\n");
    }
    else {
        USART1_Printf("[MQTT] unknown CMD: %s\r\n", payload);
    }
}


/* MQTT_ReportLightStatus：broker 会话就绪后立刻主动上报当前灯状态，
   使用 retain 标志，使任何新订阅者都能立刻收到最后一条。
   esp8266.c 的 EnsureSession / Init 成功后会回调本函数。 */
void MQTT_ReportLightStatus(void)
{
    ESP8266_MQTTPublish(MQTT_TOPIC_STATUS, g_light_state, 1);
    USART1_Printf("[MQTT] REPORT light_state=%s (retain)\r\n", g_light_state);
}

/* MQTT_ReportModeStatus：broker 会话就绪后上报当前模式状态（retain）。
   固件侧 g_mode 为 enum，此处转成字符串供前端识别。
   同 MQTT_ReportLightStatus，在所有会话就绪回调里成对出现。 */
void MQTT_ReportModeStatus(void)
{
    const char *ms = g_mode == MODE_HOME ? "HOME" :
                     g_mode == MODE_AWAY ? "AWAY" : "SLEEP";
    ESP8266_MQTTPublish(MQTT_TOPIC_MODE_STATUS, ms, 1);
    USART1_Printf("[MQTT] REPORT mode=%s (retain)\r\n", ms);
}

/* ============ 硬件初始化 ============ */
static void Hardware_InitAll(void)
{
    USART1_SendString("[INIT] LED...");   LED_Init();    USART1_SendString("OK\r\n");
    USART1_SendString("[INIT] BEEP...");  Beep_Init();   USART1_SendString("OK\r\n");
    USART1_SendString("[INIT] ADC...");   ADC_Hardware_Init(); USART1_SendString("OK\r\n");
    USART1_SendString("[INIT] HCSR04...");HCSR04_Init(); USART1_SendString("OK\r\n");
    USART1_SendString("[INIT] DHT11..."); DHT11_Init();  USART1_SendString("OK\r\n");

#if USE_BOARD_KEY
    USART1_SendString("[INIT] BoardKey...");
    KEY_Init();
    USART1_SendString("OK\r\n");
#endif

#if USE_OLED
    USART1_SendString("[INIT] OLED SSD1306...");
    OLED_Init();
    USART1_SendString("OK\r\n");
    OLED_ShowBootProgress(10, "Starting...");
#endif

#if USE_ESP8266
    USART1_SendString("[INIT] ESP8266 WiFi...");
    OLED_ShowBootProgress(30, "Connecting WiFi");
    if (ESP8266_Init()) {
        USART1_SendString("OK\r\n");
        OLED_ShowBootProgress(50, "WiFi OK");
    } else {
        USART1_SendString("FAIL (no WiFi)\r\n");
        OLED_ShowBootProgress(50, "Offline mode");
    }
#endif

#if USE_ESP32_CAM
    USART1_SendString("[INIT] ESP32-CAM...");
    OLED_ShowBootProgress(55, "Camera check...");
    if (ESP32CAM_Init()) {
        USART1_SendString("OK (online)\r\n");
    } else {
        USART1_SendString("SKIP (not ready)\r\n");
    }
#endif

#if USE_RTC
    USART1_SendString("[INIT] RTC Calendar...");
    OLED_ShowBootProgress(60, "Clock sync...");
    SG_RTC_Init();
    USART1_SendString("OK\r\n");
    /* Wakeup Timer 中断不启用（硬件 INITF 异常），秒刷新由主循环轮询完成 */
#endif

#if USE_MPU6050
    USART1_SendString("[INIT] MPU6050...");
    OLED_ShowBootProgress(65, "Gyro calibrating");
    {
        uint8_t mpu_id = MPU6050_Init();
        if (mpu_id == 0) {
            USART1_SendString("OK WHO_AM_I=0x68\r\n");
            /* 静置校准陀螺仪零偏：读 50 次取平均 */
            USART1_SendString("[MPU6050] Calibrating gyro zero-bias... keep board still\r\n");
            {
                float ax, ay, az, gx, gy, gz, t;
                float sgx=0, sgy=0, sgz=0;
                int16_t rax, ray, raz, rgx, rgy, rgz, rt;
                uint8_t k;
                for (k = 0; k < 50; k++) {
                    MPU6050_GetPhysical(&ax, &ay, &az, &gx, &gy, &gz, &t);
                    sgx += gx; sgy += gy; sgz += gz;
                    if (k == 0 || k == 49) {
                        MPU6050_GetRaw(&rax, &ray, &raz, &rgx, &rgy, &rgz, &rt);
                        USART1_Printf("[MPU6050] sample[%u] raw: ax=%d ay=%d az=%d | phys: ax=%.3f ay=%.3f az=%.3f\r\n",
                                      k, rax, ray, raz, ax, ay, az);
                    }
                    if (k == 25) {
                        OLED_ShowBootProgress(70, "Gyro calibrating");
                    }
                    delay_ms(10);
                }
                s_gyro_zero_gx = sgx / 50.0f;
                s_gyro_zero_gy = sgy / 50.0f;
                s_gyro_zero_gz = sgz / 50.0f;
            }
            delay_ms(500);   /* DLPF 稳定等待 */
            USART1_Printf("[MPU6050] zero-bias: gx=%.1f gy=%.1f gz=%.1f dps\r\n",
                          s_gyro_zero_gx, s_gyro_zero_gy, s_gyro_zero_gz);
        } else {
            USART1_Printf("FAIL WHO_AM_I=0x%02X (期望0x68)\r\n", mpu_id);
        }
    }
#endif

    /* AT24C02 与 MPU6050 共用软件 IIC 总线（PB8/PB9，地址 0x50/0x68 不冲突），
       须在 MPU6050_Init 之后初始化（总线 GPIO 已就绪；AT24C02_Init 内部
       也会幂等地再调一次 IIC_Init 兜底） */
    USART1_SendString("[INIT] AT24C02 EEPROM...");
    AT24C02_Init();
    if (AT24C02_Check()) {
        USART1_SendString("OK\r\n");
    } else {
        USART1_SendString("NOT FOUND (config persist disabled)\r\n");
    }

    USART1_SendString("[INIT] W25QXX SPI Flash...");
    W25QXX_Init();
    if (EventLog_Init()) {
        USART1_SendString("OK\r\n");
        EventLog_Dump(5);   /* 开机回显最近 5 条历史告警（黑匣子） */
    } else {
        USART1_SendString("NOT FOUND (event log disabled)\r\n");
    }
}

/* ============ 采集所有传感器 ============ */

/* 3 样本中值滤波：剔除单次毛刺 */
static uint16_t median3_u16(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}
static float median3_f(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

/* ============ 防拆/震动检测（独立高频采样） ============ */
/* 主循环每 200ms 调一次，是 g_sensor.tamper_flag 的唯一写入者。
   原先与光敏/超声波共用 SampleAllSensors 的 2.1s 节拍，每 2.1s 只能观测到
   一个瞬时角速度，凑满确认窗需持续摇晃 6s 以上，实际演示无法触发。 */
static void TamperUpdate(void)
{
#if USE_MPU6050
    float ax, ay, az, gx, gy, gz, t;
    float gx_off, gy_off, gz_off;
    uint8_t hit_now;
    uint8_t hits = 0;
    uint8_t hit = 0;
    static uint16_t s_tamper_hold = 0;      /* 触发锁存：>0 期间强制 tamper_flag=1 */
    static uint8_t  s_hit_hist[8] = {0};    /* 最近 8 次采样的命中环形历史 */
    static uint8_t  s_hit_idx = 0;          /* 环形历史写入位置 */

    /* MPU6050 与 AT24C02 共用同一条软件 I2C（PB8/PB9）：防拆任务(5Hz)与
       display 任务的配置落盘可能并发。FreeRTOS 下用互斥锁串行化整条总线，
       50ms 拿不到锁说明总线被长事务占用，本轮直接放弃，等下个 200ms 节拍。
       初始化阶段（零偏校准）在调度器启动前完成，无竞争，不需要锁。 */
#if USE_FREERTOS
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return;
    }
    MPU6050_GetPhysical(&ax, &ay, &az, &gx, &gy, &gz, &t);
    xSemaphoreGive(s_i2c_mutex);
#else
    MPU6050_GetPhysical(&ax, &ay, &az, &gx, &gy, &gz, &t);
#endif

    /* 减去上电学到的零偏 */
    gx_off = gx - s_gyro_zero_gx;
    gy_off = gy - s_gyro_zero_gy;
    gz_off = gz - s_gyro_zero_gz;

    /* 滑动窗口判定（仅用陀螺仪，加速度计在部分 GY-521 模块上不稳定）：
       任一根轴偏离零偏 > 8 dps 记一次命中，写入 8 格环形历史（8 × 0.2s = 1.6s 窗口），
       窗口内命中 >= 3 次即触发。
       取代原漏积计数器：摇晃波形常在阈值上下震荡，+1/-1 相互抵消使净漂移为负，计数永远爬不到门限
       （12dps 阈值下实测 gz 峰值 23.3dps 却只有约 27% 命中率，故阈值下调到 8dps 把命中率抬到约 45%，
       P(8 次里 >= 3 次命中) 从约 37% 升到约 75%）。 */
    hit_now = ((fabsf(gx_off) > 8.0f) ||
               (fabsf(gy_off) > 8.0f) ||
               (fabsf(gz_off) > 8.0f)) ? 1 : 0;

    s_hit_hist[s_hit_idx] = hit_now;
    s_hit_idx = (s_hit_idx + 1) % 8;

    for (uint8_t i = 0; i < 8; i++) {
        hits += s_hit_hist[i];
    }

    hit = (hits >= 3) ? 1 : 0;

    /* 锁存：触发事件本身只维持零点几秒，而决策引擎 2.2~2.3s 才读一次 tamper_flag，
       纯电平信号会被漏采。展宽到 12 × 0.2s = 2.4s（大于引擎周期），保证至少被读到一次。 */
    if (hit) s_tamper_hold = 12;

    if (s_tamper_hold > 0) {
        s_tamper_hold--;
        g_sensor.tamper_flag = 1;
    } else {
        g_sensor.tamper_flag = 0;
    }
#endif
}

/* 防拆续采钩子（P3-#4 折中）：注册给 ESP8266 AT 等待循环。
   broker 宕机时 publish 路径的完整重连最坏阻塞 ~15s，期间 task_tamper
   停摆；本钩子在 AT 等待循环里以 5Hz 续采，维持滑动窗口活性。
   命中时置粘滞位：阻塞可能超过 2.4s 锁存窗，等 task_decision 恢复后消费，
   保证冻结期间的晃动一定被告警到（P3-#4 折中：不挪任务、不加 RTOS） */
static uint8_t s_tamper_sticky = 0;
static void TamperWaitHook(void)
{
    TamperUpdate();
#if USE_MPU6050
    if (g_sensor.tamper_flag) s_tamper_sticky = 1;
#endif
}

static void SampleAllSensors(void)
{
    float temp = g_sensor.temperature, humi = g_sensor.humidity;   /* 初值取上次有效值，DHT11 失败时保持不归零 */
    uint16_t light_raw[3];
    float dist_raw[3];
    int i;

    /* 光敏 + 距离各采 3 次取中值，抑制毛刺 */
    for (i = 0; i < 3; i++) {
        light_raw[i] = ADC_ReadPhotoresistor();
#if TEST_FAKE_INPUT
        /* 假数据模式：从预定义序列取值 */
        dist_raw[i] = s_fake_seq[s_fake_idx];
        s_fake_idx = (s_fake_idx + 1) % s_fake_seq_len;
        delay_ms(30);   /* 保留节奏，不需要物理 60ms 等待 */
#else
        dist_raw[i]  = HCSR04_GetDistance();
        /* 异常值剔除：< 2cm 或 > 400cm 视为无效 */
        if (dist_raw[i] < 2.0f || dist_raw[i] > 400.0f) {
            dist_raw[i] = 0.0f;
        }
        if (i < 2) delay_ms(30);   /* HC-SR04 需要 ≥60ms 测量周期 */
#endif
    }
    g_sensor.light_adc = median3_u16(light_raw[0], light_raw[1], light_raw[2]);
    {
        float dm = median3_f(dist_raw[0], dist_raw[1], dist_raw[2]);
        /* 3 次都无效 → 沿用上次值（不返回 0） */
        g_sensor.distance = (dm > 0.0f) ? dm : g_sensor.distance;
    }

    /* DHT11 读取失败时不覆盖 g_sensor，保留上一次有效值（dht11.c 失败不写输出参数）。
       决策节拍提速到 1s 后，DHT11 仍保持 2s/次（器件手册采样周期下限 1s，
       且温湿度慢变无需更高频率），用调用计数软分频。 */
    {
        static uint8_t s_dht_div = 0;
        s_dht_div = (uint8_t)(s_dht_div + 1u);
        if ((s_dht_div & 0x01u) != 0u)
        {
            if (DHT11_Read_Data(&temp, &humi) == 0) {
                g_sensor.temperature = temp;
                g_sensor.humidity    = humi;
            }
        }
    }
    g_sensor.mq2_adc     = ADC_ReadMQ2();

    /* AT 基线更新 */
    AT_Update(g_sensor.light_adc, g_sensor.mq2_adc, g_sensor.temperature, g_sensor.humidity, g_sensor.distance);
}

/* ============ 按键处理 ============ */
#if USE_BOARD_KEY
/* 配置落盘脏标志：按键只置位，由 task_display(1s) 统一执行 Config_Save。
   原因：Config_Save 含 128KB 扇区擦除（典型 ~1s，CPU 取指停顿、期间无法喂狗），
   直接在按键路径擦除有顶爆 IWDG(2s) 的风险；延迟合并且能合并连按（P3-#2） */
static volatile uint8_t s_config_dirty = 0;

#if USE_OLED
/* 按键后立即重绘主界面（等不到 display 任务的 1s 节拍）。
   告警页显示中时不覆盖；FreeRTOS 下与 display 任务用 s_oled_mutex 互斥。
   S3 短按改到松手触发后也需要复用本函数，故抽出。 */
static void RedrawHomeOled(void)
{
    if (g_alert != ALERT_NONE) return;
#if USE_FREERTOS
    if (xSemaphoreTake(s_oled_mutex, portMAX_DELAY) == pdTRUE)
    {
        OLED_ShowHomeScreen(&g_sensor, g_mode, s_time_buf,
                            s_wifi_status, s_mqtt_status);
        xSemaphoreGive(s_oled_mutex);
    }
#else
    OLED_ShowHomeScreen(&g_sensor, g_mode, s_time_buf,
                        s_wifi_status, s_mqtt_status);
#endif
}
#endif

static void HandleBoardKey(void)
{
    static uint8_t  s_last_key = 0;
    static uint32_t s_last_accept = 0;   /* 上次有效变化时刻（消抖窗起点） */
    static uint32_t s_s4_press = 0;      /* S4 按下时刻（长按计时） */
    static uint8_t  s_s4_long = 0;       /* S4 长按已触发标志 */
    static uint32_t s_s3_press = 0;      /* S3 按下时刻（长按计时） */
    static uint8_t  s_s3_long = 0;       /* S3 长按已触发标志：长按=重学基线 */
    uint8_t key = KEY_Scan();
    uint8_t prev;

    /* S4 按住 ≥1.5s：回放事件日志（黑匣子导出），松手时不再触发手动告警 */
    if (key == S4 && s_last_key == S4 && !s_s4_long && (g_tick - s_s4_press) >= 1500) {
        s_s4_long = 1;
        USART1_SendString("-> EVENT LOG DUMP\r\n");
        EventLog_Dump(20);
    }

    /* S3 按住 ≥2s：请求重新学习基线（换房间 / 纸箱演示校准）。
       实际 20s 阻塞学习由 task_decision 执行（FreeRTOS 下不饿死其他任务）。
       长按触发后本次松手不再执行短按的"切 SLEEP"动作。 */
    if (key == S3 && s_last_key == S3 && !s_s3_long && (g_tick - s_s3_press) >= 2000) {
        s_s3_long = 1;
        g_relearn_request = 1;
        USART1_SendString("-> RE-LEARN BASELINE REQUEST: keep device STILL for 20s\r\n");
    }

    if (key == s_last_key) return;   /* 无变化，跳过 */
    prev = s_last_key;
    s_last_key = key;

    /* 机械抖动过滤：50ms 内的连续变化只跟踪不触发（S4 实测一次按下双触发） */
    if ((g_tick - s_last_accept) < 50) return;
    s_last_accept = g_tick;

    if (key == S4) { s_s4_press = g_tick; s_s4_long = 0; }
    if (key == S3) { s_s3_press = g_tick; s_s3_long = 0; }

    /* S4 短按（松开且未达长按）：手动告警测试 */
    if (prev == S4 && key == 0 && !s_s4_long) {
        g_manual_alert = ALERT_INTRUSION;
        g_last_alarm_tick = 0;
        USART1_SendString("-> MANUAL ALERT TEST\r\n");
    }

    /* S3 短按（松开且未达长按 2s）：切 SLEEP。
       特意放到松手沿，避免按 S3 长按时先误切一次模式。 */
    if (prev == S3 && key == 0 && !s_s3_long) {
        g_mode = MODE_SLEEP;
        DecisionEngine_SetMode(g_mode);
        s_config_dirty = 1;
        USART1_SendString("-> MODE SLEEP\r\n");
#if USE_FREERTOS
        PostCommEvent(EVT_KEY_MODE, (uint8_t)g_mode);
#else
        MQTT_ReportModeStatus();
#endif
#if USE_OLED
        RedrawHomeOled();
#endif
    }

    if (key == 0) return;            /* 松开，不触发 */

    switch (key) {
        case S1: g_mode = MODE_HOME;  DecisionEngine_SetMode(g_mode); s_config_dirty = 1; USART1_SendString("-> MODE HOME\r\n");
#if USE_FREERTOS
                /* 上报走通信任务，避免与遥测/心跳并发抢 USART3 */
                PostCommEvent(EVT_KEY_MODE, (uint8_t)g_mode);
#else
                MQTT_ReportModeStatus();
#endif
                break;
        case S2: g_mode = MODE_AWAY;  DecisionEngine_SetMode(g_mode); s_config_dirty = 1; USART1_SendString("-> MODE AWAY\r\n");
#if USE_FREERTOS
                PostCommEvent(EVT_KEY_MODE, (uint8_t)g_mode);
#else
                MQTT_ReportModeStatus();
#endif
                break;
        /* S3 已改为：短按松手切 SLEEP，长按 2s 重新学习基线 */
    }

#if USE_OLED
    /* 按键后立即重绘主界面（Row 1 模式/时间/WiFi/MQTT），避免等 1 秒 tick。
       告警页显示中时不覆盖；代价是一次整屏重绘约 50ms，仅按键时发生。
       FreeRTOS 下该重绘发生在 fast 任务，必须与 display 任务的 OLED 刷新互斥。 */
    RedrawHomeOled();
#endif
}
#endif

#if USE_OLED
/* 基线学习进度回调：注册给 AT_LearnBaseline，每秒一次。
   进度 80→98 平滑推进 + 剩余秒数倒计时，消灭学习期画面假死。
   手动重学发生在调度器运行中（TaskDecision），与 TaskDisplay 的 1s 刷屏
   并发抢软件 SPI，必须取 s_oled_mutex；上电学习时调度器未启动、锁空闲，
   take 立即成功，两条调用路径共用本函数均安全。 */
static void BootLearnProgress(uint8_t done, uint8_t total)
{
    char st[20];
    snprintf(st, sizeof(st), "Learning %us", (unsigned)(total - done));
#if USE_FREERTOS
    xSemaphoreTake(s_oled_mutex, portMAX_DELAY);
#endif
    OLED_ShowBootProgress(80 + (uint16_t)18 * done / total, st);
#if USE_FREERTOS
    xSemaphoreGive(s_oled_mutex);
#endif
}
#endif

/* ============ 调度任务（scheduler，周期基于真实 g_tick，无时漂） ============ */

/* 裸机：0 周期每轮必跑 —— MQTT 下行处理 + 按键 + 喂狗。
   FreeRTOS：MQTT 下行与补报移入 TaskComm 串行执行（USART3 单点访问），
   本任务（TaskFast 壳，50ms 周期）只剩按键扫描与看门狗喂狗。 */
static void task_fast(void)
{
#if USE_MQTT && !USE_FREERTOS
    ESP8266_MQTTProcessRx();
    CmdReportFlush();   /* 被合并的灯/模式上报在此补报（500ms 窗口过后） */
#endif
#if USE_BOARD_KEY
    HandleBoardKey();
#endif
    IWDG_Feed();
}

/* 防拆检测：200ms（5Hz）。注册顺序在 task_decision 之前，
   同一轮到期时先刷新 tamper 标志再交给决策引擎 */
static void task_tamper(void)
{
    TamperUpdate();
}

/* 每 2s 采集 + 决策 + 告警统一出口 */
static void task_decision(void)
{
    /* ===== 长按 S3 请求的重新学习（换房间 / 进纸箱演示前校准）=====
       FreeRTOS：本任务在 AT_LearnBaseline 内阻塞 20s 期间，delay_ms 退化为
       vTaskDelay，fast 喂狗/按键、tamper 防拆、comm 网络、display 刷新全部
       照常被调度（函数内另带分段 IWDG_Feed 双保险）；
       裸机：与上电阻塞学习行为相同，协作调度器停转 20s，IWDG 由函数内喂。 */
    if (g_relearn_request) {
        g_relearn_request = 0;
        g_alert = ALERT_NONE;
        g_manual_alert = ALERT_NONE;
        USART1_SendString("\r\n[BASELINE] Manual re-learn start (20s, keep device STILL)...\r\n");
        AT_Init();                 /* is_learned=0，学习期决策静默 */
#if USE_OLED
        AT_SetProgressCallback(BootLearnProgress);
#endif
        AT_LearnBaseline();
#if USE_OLED
        AT_SetProgressCallback(0);
#endif
        /* AT24C02 软件 I2C 与 MPU6050 共 PB8/PB9：落盘必须与 TaskTamper 互斥 */
#if USE_FREERTOS
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
#endif
        if (Config_Save((const AT_Baseline_t *)AT_GetBaseline(), g_mode)) {
            USART1_SendString("[BASELINE] re-learn saved OK\r\n");
        } else {
            USART1_SendString("[BASELINE] re-learn save FAIL\r\n");
        }
#if USE_FREERTOS
        xSemaphoreGive(s_i2c_mutex);
#endif
        /* 新环境重新采集布防静息距离（给 10s 撤离窗口） */
        DecisionEngine_RestartArm();
        USART1_SendString("[BASELINE] re-learn done, next cycle resume decision\r\n");
        return;
    }

    SampleAllSensors();

    /* 基线未就绪（任何学习期）：只维持采样，不决策、不上报，
       避免学习期箱内闷热 / 距离箱壁近触发误报 */
    if (!AT_GetBaseline()->is_learned) {
        return;
    }

    /* 消费 MQTT 重连阻塞期间粘滞下来的防拆命中（见 TamperWaitHook） */
    if (s_tamper_sticky) {
        s_tamper_sticky = 0;
        g_sensor.tamper_flag = 1;
    }

    /* 决策引擎 */
    g_alert = DecisionEngine_Run(&g_sensor);

    USART1_Printf("[%lus] L=%4d D=%.1fcm T=%.0fC H=%.0f%% | %s | %s\r\n",
        g_tick / 1000,
        g_sensor.light_adc, g_sensor.distance,
        g_sensor.temperature, g_sensor.humidity,
        g_mode == MODE_HOME ? "HOME " : g_mode == MODE_AWAY ? "AWAY " : "SLEEP",
        g_alert == ALERT_NONE ? "OK" :
        g_alert == ALERT_INTRUSION ? "!!INTRUSION!!" :
        g_alert == ALERT_FIRE ? "!!FIRE!!" :
        g_alert == ALERT_TAMPER ? "!!TAMPER!!" : "!!ENV!!");

    /* 告警处理 —— 统一出口（决策引擎 + 手动测试） */
    {
        Alert_Type_t effective = (g_manual_alert != ALERT_NONE) ? g_manual_alert : g_alert;
        static Alert_Type_t s_last_effective = ALERT_NONE;
        uint32_t now = g_tick / 1000;   /* 真实秒 */
        uint8_t state_changed = (effective != s_last_effective);

        if (effective != ALERT_NONE) {
            if (state_changed || now - g_last_alarm_tick >= 5) {
                LED_Toggle(1);

                /* 新告警事件落盘 W25Qxx 历史日志（只在状态翻转时记一条，
                   5s 重复告警不重复记，省 FLASH 寿命也防日志刷屏） */
                if (state_changed) {
                    EventLog_Add((uint8_t)effective, &g_sensor, (uint8_t)g_mode);

                    /* 分级告警音（P2-5）：比赛演示期间 BEEP_ENABLE=0 整体静音，
                       恢复时把 beep.h 的 BEEP_ENABLE 改 1 即可（阻塞最长 1.2s
                       < IWDG 2s 窗口，仅在状态翻转时播放） */
#if BEEP_ENABLE
                    switch (effective) {
                        case ALERT_INTRUSION:   /* 入侵：急促三连音 */
                            Beep_PlayNote(NOTE_LA_HI, 100); delay_ms(80);
                            Beep_PlayNote(NOTE_LA_HI, 100); delay_ms(80);
                            Beep_PlayNote(NOTE_LA_HI, 100);
                            break;
                        case ALERT_FIRE:        /* 火灾：连续双音交替 */
                            Beep_PlayNote(NOTE_MI, 300);
                            Beep_PlayNote(NOTE_LA, 300);
                            Beep_PlayNote(NOTE_MI, 300);
                            Beep_PlayNote(NOTE_LA, 300);
                            break;
                        case ALERT_TAMPER:      /* 防拆：低沉单长音 */
                            Beep_PlayNote(BEEP_ERR, 800);
                            break;
                        default:                /* 环境异常：单短音 */
                            Beep_PlayNote(NOTE_SOL, 200);
                            break;
                    }
#endif
                }

#if USE_FREERTOS
                /* 抢占式调度：MQTT 上报与入侵抓拍全部投递到通信任务执行，
                   保证 USART2/USART3 任何时刻只有一个任务在访问（见 TaskComm）。
                   告警音/LED/事件日志等本地执行部分仍留在本任务。 */
                PostCommEvent(EVT_ALERT, (uint8_t)effective);
#else
#if USE_ESP8266
    #if USE_MQTT
                if (effective == ALERT_INTRUSION)
                    ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Intrusion!", 0);
                else if (effective == ALERT_FIRE)
                    ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Fire!", 0);
                else if (effective == ALERT_TAMPER)
                    ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Tamper!", 0);
                else
                    ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:EnvAbnormal!", 0);
    #else
                if (effective == ALERT_INTRUSION)
                    ESP8266_SendAlert("Intrusion!");
                else if (effective == ALERT_FIRE)
                    ESP8266_SendAlert("Fire!");
                else if (effective == ALERT_TAMPER)
                    ESP8266_SendAlert("Tamper!");
                else
                    ESP8266_SendAlert("EnvAbnormal!");
    #endif
#endif

#if USE_ESP32_CAM
                if (effective == ALERT_INTRUSION) {
                    char cam_url[96];
                    ESP32CAM_Status_t cam_st = ESP32CAM_TriggerCapture(cam_url, sizeof(cam_url));
#if USE_OLED
                    s_cam_ok = (cam_st == CAM_OK) ? 1 : 0;
#endif
                    if (cam_st == CAM_OK) {
                        USART1_Printf("[ESP32-CAM] photo: %s\r\n", cam_url);
                        /* 拍照成功：图片 URL 推 MQTT（retain，Web 端刷新也能看到最后一张） */
                        ESP8266_MQTTPublish(MQTT_TOPIC_CAM_SHOT, cam_url, 1);
                    } else if (cam_st == CAM_OFFLINE) {
                        USART1_SendString("[ESP32-CAM] offline\r\n");
                    } else {
                        USART1_Printf("[ESP32-CAM] failed (%d)\r\n", cam_st);
                    }
                }
#endif
#endif /* !USE_FREERTOS */

                g_last_alarm_tick = now;
                g_manual_alert = ALERT_NONE;  /* 手动告警发完即清（comm/按键写、本任务清） */
            }
        } else {
            LED_OFF(1);
        }
        s_last_effective = effective;
    }
}

/* 每 1s：配置落盘 + RTC 秒刷新 + OLED 刷新（含 WiFi/MQTT 状态采样） */
static void task_display(void)
{
    /* 延迟落盘：按键置脏 / AT 自愈置脏后在此统一擦写（见 HandleBoardKey 上方注释）。
       FreeRTOS 下落盘走 AT24C02（软件 I2C，与 MPU6050 共线），取总线互斥锁。 */
    if (s_config_dirty || AT_TakeConfigDirty()) {
        s_config_dirty = 0;
#if USE_FREERTOS
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
        Config_Save(AT_GetBaseline(), g_mode);
        xSemaphoreGive(s_i2c_mutex);
#else
        Config_Save(AT_GetBaseline(), g_mode);
#endif
    }

#if USE_RTC
    SG_RTC_FormatTime(s_time_buf, sizeof(s_time_buf));
    /* 显示节拍提速到 500ms 后，RTC 秒打印仍保持 1s 一行（隔次打印），
       避免同一秒重复刷屏日志 */
    {
        static uint8_t s_rtc_div = 0;
        s_rtc_div = (uint8_t)(s_rtc_div + 1u);
        if (s_rtc_div & 0x01u)
            USART1_Printf("[RTC] %s\r\n", s_time_buf);
    }
#endif

#if USE_OLED
#if USE_ESP8266
    s_wifi_status = ESP8266_IsWiFiOk() ? "W+" : "W-";
    #if USE_MQTT
    s_mqtt_status = ESP8266_IsMQTTOk() ? "M+" : "M-";
    #endif
#endif
    /* OLED 软件 SPI 另一处使用点是 fast 任务的按键即时重绘，必须互斥 */
#if USE_FREERTOS
    xSemaphoreTake(s_oled_mutex, portMAX_DELAY);
#endif
    if (g_alert != ALERT_NONE) {
        OLED_ShowAlertPage(&g_sensor, g_alert, s_time_buf, s_cam_ok);
    } else {
        OLED_ShowHomeScreen(&g_sensor, g_mode, s_time_buf,
                            s_wifi_status, s_mqtt_status);
    }
#if USE_FREERTOS
    xSemaphoreGive(s_oled_mutex);
#endif
#endif
}

#if USE_MQTT
/* 每 10s：遥测上报 + MQ2 阈值调试行（原决策块 %100 调试行，同 10s 节拍） */
static void task_telemetry(void)
{
    char mqtt_msg[64];
    sprintf(mqtt_msg, "T=%.0f,H=%.0f,D=%.0f,L=%d,M=%d",
            g_sensor.temperature, g_sensor.humidity,
            g_sensor.distance, g_sensor.light_adc,
            (int)g_sensor.mq2_adc);
    ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, mqtt_msg, 0);

    /* 运行期阈值真值：阈值靠 EWMA(alpha=0.01) 漂移，是慢变量，10s 粒度足够 */
    USART1_Printf("[DBG] MQ2=%d th=%d T=%.1fC dist_base=%.1f\r\n",
                  g_sensor.mq2_adc, AT_GetBaseline()->mq2_fire_threshold,
                  g_sensor.temperature, AT_GetBaseline()->dist_ewma);
}

/* MQTT 心跳 PINGREQ：每 MQTT_KEEPALIVE_SEC/2 秒（60/2 = 30s） */
static void task_ping(void)
{
    ESP8266_MQTTPing();
}
#endif

#if USE_ESP8266
static uint8_t s_ntp_done = 0;   /* NTP 是否已成功同步过（开机或恢复后一次即可） */

/* NTP 对时 + 对时后 MQTT 重连（GetNTPTime 内部会 CIPCLOSE 旧 socket）。
   开机与 WiFi 恢复后共用；成功置 s_ntp_done。 */
static void SyncNtpOnce(void)
{
    USART1_SendString("[RTC] Calling NTP...\r\n");
    {
        RTC_DateTime_t ntp_dt;
        if (ESP8266_GetNTPTime(&ntp_dt)) {
            SG_RTC_SetDateTime(ntp_dt.Year, ntp_dt.Month, ntp_dt.Date,
                               ntp_dt.Hours, ntp_dt.Minutes, ntp_dt.Seconds);
            USART1_Printf("[RTC] NTP synced: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                          ntp_dt.Year, ntp_dt.Month, ntp_dt.Date,
                          ntp_dt.Hours, ntp_dt.Minutes, ntp_dt.Seconds);
            s_ntp_done = 1;
        } else {
            USART1_SendString("[RTC] NTP sync failed, use compile time\r\n");
        }
    }

#if USE_MQTT
    /* GetNTPTime 内部 CIPCLOSE 后 MQTT TCP 必须重建 */
    if (!ESP8266_MQTTConnect())
        USART1_Printf("[MQTT] Reconnect after NTP failed\r\n");
    else if (!ESP8266_MQTTSubscribe())
        USART1_Printf("[MQTT] Resubscribe after NTP failed\r\n");
    else {
        MQTT_ReportLightStatus();
        MQTT_ReportModeStatus();
    }
#endif
}

/* 每 5 分钟：ESP8266 网络健康检查 */
static void task_health(void)
{
    ESP8266_CheckHealth();

    /* 开机时 ESP8266 未就绪会跳过 NTP（RTC 停在编译时间）；
       WiFi 恢复后在此补一次对时（P1 冷启动容错）。 */
    if (!s_ntp_done && ESP8266_IsWiFiOk()) {
        SyncNtpOnce();
    }
}
#endif

#if ML_DATA_COLLECT
/* 每 2s：ML 数据采集 CSV 行（串口 + UDP）。
   FreeRTOS 下不单独建任务，由 TaskComm 在 2s 节拍调用（ESP8266_SendUDP
   访问 USART3，必须与其他网络操作同在通信任务内串行）。 */
static void task_ml_collect(void)
{
    uint32_t t = g_tick / 1000;
    char csv_buf[128];
    const char *mode_str = g_mode == MODE_HOME ? "HOME" :
                           g_mode == MODE_AWAY ? "AWAY" : "SLEEP";
    const char *alert_str = g_alert == ALERT_NONE ? "0" :
                            g_alert == ALERT_INTRUSION ? "1" :
                            g_alert == ALERT_FIRE ? "2" :
                            g_alert == ALERT_TAMPER ? "3" : "4";
    int n = snprintf(csv_buf, sizeof(csv_buf),
        "%lu,%u,%.1f,%.1f,%.1f,%s,%s\r\n",
        t, g_sensor.light_adc, g_sensor.distance,
        g_sensor.temperature, g_sensor.humidity,
        mode_str, alert_str);
    USART1_SendString(csv_buf);
#if USE_ESP8266
    /* 直接发原始 CSV 行到 UDP（绕过 SendData 的 JSON 包装） */
    extern uint8_t ESP8266_SendUDP(uint8_t *data, uint16_t len);
    ESP8266_SendUDP((uint8_t *)csv_buf, (uint16_t)n);
#endif
}
#endif

#if USE_FREERTOS
/* ==========================================================================
 * FreeRTOS 任务层
 *
 * 旧的 task_* 周期函数原样保留为"单次执行体"，下面的 TaskXxx 壳函数只负责
 * vTaskDelay 周期节拍，业务代码零改动。调度节拍对照：
 *   TaskFast      50ms   按键 + IWDG（旧 0 周期每轮，50ms 远小于消抖/喂狗窗口）
 *   TaskTamper    200ms  防拆 5Hz
 *   TaskDecision  1000ms 采集 + 决策 + 本地告警（DHT11 内部 2s 软分频）
 *   TaskDisplay   500ms  RTC + OLED + 配置落盘（RTC 打印内部 1s 分频）
 *   TaskComm      内部节拍：MQTT 收包 50ms / 遥测 10s / 心跳 30s / 健康 5min
 * ========================================================================== */

static void TaskFast(void *arg)
{
    (void)arg;
    for (;;)
    {
        task_fast();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void TaskTamper(void *arg)
{
    (void)arg;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(200));
        task_tamper();
    }
}

static void TaskDecision(void *arg)
{
    (void)arg;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        task_decision();
    }
}

static void TaskDisplay(void *arg)
{
    (void)arg;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        task_display();
    }
}

#if USE_ESP8266
/* 告警事件的网络侧处理：MQTT 告警上报 + 入侵抓拍（代码语义与裸机版一致，
   只是执行位置从 decision 任务移到本任务，串行化 USART2/USART3 访问） */
static void CommHandleAlert(uint8_t alert)
{
    Alert_Type_t effective = (Alert_Type_t)alert;

#if USE_MQTT
    if (effective == ALERT_INTRUSION)
        ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Intrusion!", 0);
    else if (effective == ALERT_FIRE)
        ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Fire!", 0);
    else if (effective == ALERT_TAMPER)
        ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Tamper!", 0);
    else
        ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:EnvAbnormal!", 0);
#else
    if (effective == ALERT_INTRUSION)
        ESP8266_SendAlert("Intrusion!");
    else if (effective == ALERT_FIRE)
        ESP8266_SendAlert("Fire!");
    else if (effective == ALERT_TAMPER)
        ESP8266_SendAlert("Tamper!");
    else
        ESP8266_SendAlert("EnvAbnormal!");
#endif

#if USE_ESP32_CAM
    if (effective == ALERT_INTRUSION)
    {
        char cam_url[96];
        ESP32CAM_Status_t cam_st = ESP32CAM_TriggerCapture(cam_url, sizeof(cam_url));
#if USE_OLED
        s_cam_ok = (cam_st == CAM_OK) ? 1 : 0;
#endif
        if (cam_st == CAM_OK) {
            USART1_Printf("[ESP32-CAM] photo: %s\r\n", cam_url);
            ESP8266_MQTTPublish(MQTT_TOPIC_CAM_SHOT, cam_url, 1);
        } else if (cam_st == CAM_OFFLINE) {
            USART1_SendString("[ESP32-CAM] offline\r\n");
        } else {
            USART1_Printf("[ESP32-CAM] failed (%d)\r\n", cam_st);
        }
    }
#endif
}

/* 通信任务：全工程唯一访问 ESP8266/ESP32-CAM 串口的任务。
   - 50ms 节拍处理 MQTT 下行帧 + 合并上报补报（CmdReportFlush）
   - 事件队列：按键模式补报、告警上报/抓拍
   - 内部维护遥测/心跳/健康检查/ML 采集的软周期（原协作式调度同一套时序）
   阻塞点（AT 长等待、CAPTURE 最坏 ~4s）只冻结本任务；看门狗由 TaskFast 喂，
   防拆由 TaskTamper + TamperWaitHook 续采，与裸机版行为一致。 */
static void TaskComm(void *arg)
{
    TickType_t last_tele;
    TickType_t last_ping;
    TickType_t last_health;
#if ML_DATA_COLLECT
    TickType_t last_ml;
#endif
    TickType_t now;
    comm_evt_t evt;

    (void)arg;
    last_tele = last_ping = last_health = xTaskGetTickCount();
#if ML_DATA_COLLECT
    last_ml = last_tele;
#endif

    for (;;)
    {
#if USE_MQTT
        ESP8266_MQTTProcessRx();
        CmdReportFlush();   /* 被合并的灯/模式上报在此补报（500ms 窗口过后） */
#endif

        /* 事件队列一次性抽空（长度 8，正常不会积压） */
        while (xQueueReceive(s_comm_queue, &evt, 0) == pdPASS)
        {
            if (evt.type == (uint8_t)EVT_KEY_MODE)
            {
                MQTT_ReportModeStatus();   /* g_mode 已由按键任务更新，retain 补报 */
            }
            else if (evt.type == (uint8_t)EVT_ALERT)
            {
                CommHandleAlert(evt.value);
            }
            else
            {
                /* 未知事件丢弃 */
            }
        }

        now = xTaskGetTickCount();
#if USE_MQTT
        if ((TickType_t)(now - last_tele) >= pdMS_TO_TICKS(10000))
        {
            last_tele = now;
            task_telemetry();
        }
        if ((TickType_t)(now - last_ping) >= pdMS_TO_TICKS(MQTT_KEEPALIVE_SEC * 1000 / 2))
        {
            last_ping = now;
            task_ping();
        }
#endif
        if ((TickType_t)(now - last_health) >= pdMS_TO_TICKS(300000))
        {
            last_health = now;
            task_health();
        }
#if ML_DATA_COLLECT
        if ((TickType_t)(now - last_ml) >= pdMS_TO_TICKS(2000))
        {
            last_ml = now;
            task_ml_collect();
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif /* USE_ESP8266 */

/* ============ FreeRTOS 钩子函数（FreeRTOSConfig.h 中使能） ============ */

/* 空闲钩子：没有任务就绪时 WFI 休眠，任意中断（USART RX/SysTick）唤醒，
   保留旧协作式调度器 __WFI 的低功耗行为。 */
void vApplicationIdleHook(void)
{
    __WFI();
}

/* 栈溢出（方法 2）/堆分配失败：打故障标记并停机。
   注意这两个钩子可能在 PendSV 上下文中被调用，不能取互斥锁/走阻塞串口，
   只写全局标记，J-Link 暂停后看 s_rtos_fault 即可定位。 */
static volatile uint32_t s_rtos_fault = 0;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    s_rtos_fault = 1U;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    s_rtos_fault = 2U;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
#endif /* USE_FREERTOS */

/* ============ 主函数 ============ */
int main(void)
{
#if USE_FREERTOS
    /* FreeRTOS 官方要求：所有优先级位都用作抢占优先级（STM32 4 位 → group 4）。
       USART1/2/3 的 ISR 抢占优先级保持 1（高于 configMAX_SYSCALL=5，不调用
       任何 FromISR API，仅填环形缓冲），满足该约束。 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
#else
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
#endif
    Delay_Init();
    USART1_Init(115200);
#if USE_FREERTOS
    /* heap_4 在调度器启动前即可用，先建同步对象，后续初始化打印自动绕过锁 */
    USART1_LockInit();
    s_comm_queue = xQueueCreate(8, sizeof(comm_evt_t));
    s_i2c_mutex  = xSemaphoreCreateMutex();
    s_oled_mutex = xSemaphoreCreateMutex();
    if ((s_comm_queue == (QueueHandle_t)0) ||
        (s_i2c_mutex == (SemaphoreHandle_t)0) ||
        (s_oled_mutex == (SemaphoreHandle_t)0))
    {
        USART1_SendString("[RTOS] kernel object create FAIL\r\n");
        for (;;) { }
    }
#endif

    USART1_SendString("\r\n====================================\r\n");
    USART1_SendString("  SmartGuard v0.7.2 - Space Guard Agent (FreeRTOS)\r\n");
    USART1_SendString("====================================\r\n");

    Hardware_InitAll();
    delay_ms(1000);

    /* 检测 ESP8266 是否连通外网 */
#if USE_ESP8266
    if (ESP8266_IsWiFiOk())
    {
    ESP8266_CheckInternet();

    /* NTP/UDP 时间同步（阿里云 NTP 服务器） */
    SyncNtpOnce();
    }
    else
    {
        /* WiFi 未连上时跳过外网检测/NTP/MQTT 握手（各阶段 AT 超时串行叠加，
           最坏浪费 ~40-60s 启动时间）。运行期由 CheckHealth/EnsureSession
           负责恢复（P2 断网启动优化）。 */
        USART1_SendString("[NET] WiFi offline, skip internet/NTP/MQTT boot steps\r\n");
    }
#endif

    /* 同步初始模式到决策引擎 */
    DecisionEngine_Init();
    DecisionEngine_SetMode(g_mode);

#if USE_OLED
    OLED_ShowBootProgress(75, "Network sync...");
#endif

    /* ========== 基线学习（优先从 FLASH 恢复，失败才重新学 20s） ========== */
    AT_Init();

    if (Config_Load((AT_Baseline_t *)AT_GetBaseline(), &g_mode)) {
        /* FLASH 有有效配置，直接恢复，跳过 20s 学习 */
        USART1_SendString("[BASELINE] Restored from FLASH (skip 20s learn)\r\n");
        DecisionEngine_SetMode(g_mode);
#if USE_OLED
        OLED_ShowBootProgress(98, "Restored!");
#endif
    } else {
        /* FLASH 无有效数据，正常学习 20s */
        USART1_SendString("\r\n[BASELINE] Learning 20s baseline...\r\n");
#if USE_OLED
        AT_SetProgressCallback(BootLearnProgress);
#endif
        AT_LearnBaseline();

        /* 学习完存 FLASH */
        if (Config_Save((const AT_Baseline_t *)AT_GetBaseline(), g_mode)) {
            USART1_SendString("[BASELINE] Saved to FLASH OK\r\n");
        } else {
            USART1_SendString("[BASELINE] FLASH save FAIL\r\n");
        }
    }

    const AT_Baseline_t *bl = AT_GetBaseline();
    USART1_Printf("[BASELINE] light_bl=%d temp_bl=%.1f humi_bl=%.1f mode=%d\r\n",
        bl->light_baseline, bl->temp_baseline, bl->humi_baseline, g_mode);

#if USE_OLED
    OLED_ShowBootProgress(100, "Ready!");
    delay_ms(800);
#endif

    /* ========== 启动独立看门狗（基线学习完成后再启动，避免 20s 阻塞超时） ========== */
    USART1_SendString("\r\n[INIT] IWDG 2s watchdog...");
    IWDG_Init();
    USART1_SendString("OK\r\n");

    USART1_SendString("\r\n[MAIN] Entering SmartGuard main loop...\r\n\r\n");

#if ML_DATA_COLLECT
    USART1_SendString("CSV_HEADER:tick,light,distance_cm,temp_c,humi_pct,mode,alert\r\n");
#endif

#if USE_ESP8266
    /* P3-#4 折中：MQTT 重连长阻塞期间由 AT 等待循环回调续采防拆 */
    ESP8266_SetWaitHook(TamperWaitHook);
#endif

#if USE_FREERTOS
    /* ========== FreeRTOS 任务创建并启动调度器（永不返回） ==========
       优先级：fast(4) > tamper(3) > decision/comm(2) > display(1)。
       高优先级任务全部阻塞在 vTaskDelay 上时，display 才执行 OLED 慢刷新；
       comm 的 AT 长阻塞不再冻结按键/喂狗/防拆（对比旧协作式调度的关键改进）。 */
    #define SG_CREATE_TASK(func, name, stack, prio)                              \
        do {                                                                     \
            TaskHandle_t _h = (TaskHandle_t)0;                                   \
            if (xTaskCreate((func), (name), (stack), (void *)0,                 \
                            (prio), &_h) != pdPASS)                              \
            {                                                                    \
                USART1_Printf("[RTOS] create task %s FAIL\r\n", (name));        \
            }                                                                    \
        } while (0)

    SG_CREATE_TASK(TaskFast,     "fast",     STACK_FAST,     PRIO_FAST);
    SG_CREATE_TASK(TaskTamper,   "tamper",   STACK_TAMPER,   PRIO_TAMPER);
    SG_CREATE_TASK(TaskDecision, "decision", STACK_DECISION, PRIO_DECISION);
    SG_CREATE_TASK(TaskDisplay,  "display",  STACK_DISPLAY,  PRIO_DISPLAY);
#if USE_ESP8266
    SG_CREATE_TASK(TaskComm,     "comm",     STACK_COMM,     PRIO_COMM);
#endif

    USART1_SendString("[RTOS] scheduler starting (FreeRTOS V10.5.1)\r\n\r\n");
    vTaskStartScheduler();   /* 永不返回：port 重配 SysTick，首任务由 SVC 拉起 */

    /* 正常不会走到：只有堆不足导致 idle 任务创建失败才返回 */
    for (;;)
    {
    }
#else
    /* ========== 协作式调度器启动（注册表批量注册，风格对齐课程模板 task_list；
       任务实现见上方 task_* 函数。注册顺序有意义：tamper 须在 decision 之前，
       同一轮到期时先刷新 tamper 标志再交给决策引擎） ========== */
    {
        static const sched_task_reg_t s_task_table[] =
        {
            {task_fast,      0,                             SCHED_TASK_ENABLE, "fast"},      /* MQTT 下行 + 按键 + 喂狗（每轮必跑） */
            {task_tamper,    200,                           SCHED_TASK_ENABLE, "tamper"},    /* 防拆 5Hz */
            {task_decision,  2000,                          SCHED_TASK_ENABLE, "decision"},  /* 采样 + 决策 + 告警统一出口 */
            {task_display,   1000,                          SCHED_TASK_ENABLE, "display"},   /* RTC 秒刷 + OLED 刷新 */
#if USE_MQTT
            {task_telemetry, 10000,                         SCHED_TASK_ENABLE, "telemetry"}, /* 遥测 + MQ2 阈值调试行 */
            {task_ping,      MQTT_KEEPALIVE_SEC * 1000 / 2, SCHED_TASK_ENABLE, "ping"},      /* 30s 心跳 */
#endif
#if ML_DATA_COLLECT
            {task_ml_collect, 2000,                         SCHED_TASK_ENABLE, "ml_collect"},/* ML 数据采集 CSV */
#endif
#if USE_ESP8266
            {task_health,    300000,                        SCHED_TASK_ENABLE, "health"},    /* 5min 网络健康检查 */
#endif
        };

        scheduler_init();
        if (scheduler_register_table(s_task_table,
                (uint8_t)(sizeof(s_task_table) / sizeof(s_task_table[0])))
            != (uint8_t)(sizeof(s_task_table) / sizeof(s_task_table[0])))
        {
            USART1_SendString("[SCH] WARN: task table overflow, some tasks NOT registered\r\n");
        }
        scheduler_run();   /* 永不返回：空闲 __WFI 休眠，SysTick 1ms 唤醒，周期零漂移 */
    }
#endif
}
