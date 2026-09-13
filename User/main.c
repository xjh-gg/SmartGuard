/**
  * @file    main.c
  * @brief   SmartGuard v0.4 — 智能空间守护 Agent
  *
  * 硬件清单 (已全部验证):
  *   ADC         — 光敏电阻 PF7/ADC3_IN5, 电位器 PA5/ADC1_IN5
  *   HCSR04      — Trig=PC8, Echo=PC6
  *   DHT11       — PG9 (一次读回 temp+humi)
  *   BoardKey    — S1=PA0, S2=PE2, S3=PE3, S4=PE4 (板载按键)
  *   LED         — PF9/PF10/PE13/PE14 (低电平亮), BEEP: PF8
  *   OLED SSD1306 — PB0=CLK PF11=MOSI PB1=DC PC13=CS1, CS2接VCC
  *   ESP8266     — USART3 PB10/PB11, 115200bps
  */

/* ============ 编译开关（必须在 include 之前定义）============ */
#define USE_MATRIX_KEY  0
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
#if USE_MATRIX_KEY
#include "matrix_key.h"
#endif
#if USE_BOARD_KEY
#include "key.h"
#endif
#include "sensor_hub.h"
#include "decision_engine.h"   /* 已经 include USE_OLED/USE_ESP8266 定义 */
#include "adaptive_threshold.h"
#include "oled.h"
#include "iwdg.h"
#include "config_store.h"
#include <math.h>          /* fabsf：TamperUpdate 滑动窗口判定用 */

#if USE_RTC
#include "rtc.h"
#endif

#if USE_ESP8266
#include "esp8266.h"
#include <string.h>      /* strcmp for MQTT_OnCmdReceived */
#endif

#if USE_ESP32_CAM
#include "esp32_cam.h"
#endif

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
static Alert_Type_t  g_manual_alert = ALERT_NONE;  /* 按键手动触发的测试告警 */
static uint32_t      g_last_alarm_tick = 0;

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
static uint32_t s_mqtt_last_ping_tick = 0;

/* ESP8266 下行控制指令回调：收到 home/light/cmd 的 PUBLISH 时调用 */
void MQTT_OnCmdReceived(const char *payload)
{
    /* led.c 的 case 编号比 led.h 的宏名大 1（API 是 1-based，宏是 0-based）：
       case 1→LED0=PF9  case 2→LED1=PF10  case 3→LED2=PE13  case 4→LED3=PE14
       这里用 3 → PE13（GEC-M4 原理图 Page 2 的 FSMC_D10，板载 D3），
       全项目仅此处驱动，不会被告警灯逻辑（用 case 1 = PF9）覆盖。 */
    if (strcmp(payload, "on") == 0) {
        LED_ON(3);
        USART1_Printf("[MQTT] CMD=on\r\n");
    }
    else if (strcmp(payload, "off") == 0) {
        LED_OFF(3);
        USART1_Printf("[MQTT] CMD=off\r\n");
    }
    else if (strcmp(payload, "HOME") == 0) {
        g_mode = MODE_HOME;  DecisionEngine_SetMode(g_mode);
        USART1_Printf("[MQTT] CMD=HOME\r\n");
    }
    else if (strcmp(payload, "AWAY") == 0) {
        g_mode = MODE_AWAY;  DecisionEngine_SetMode(g_mode);
        USART1_Printf("[MQTT] CMD=AWAY\r\n");
    }
    else if (strcmp(payload, "SLEEP") == 0) {
        g_mode = MODE_SLEEP; DecisionEngine_SetMode(g_mode);
        USART1_Printf("[MQTT] CMD=SLEEP\r\n");
    }
    else {
        USART1_Printf("[MQTT] unknown CMD: %s\r\n", payload);
    }
    /* 状态改变后上报 */
    ESP8266_MQTTPublish(MQTT_TOPIC_STATUS, payload);
}

/* ============ 硬件初始化 ============ */
static void Hardware_InitAll(void)
{
    USART1_SendString("[INIT] LED...");   LED_Init();    USART1_SendString("OK\r\n");
    USART1_SendString("[INIT] BEEP...");  Beep_Init();   USART1_SendString("OK\r\n");
    USART1_SendString("[INIT] ADC...");   ADC_Hardware_Init(); USART1_SendString("OK\r\n");
    USART1_SendString("[INIT] HCSR04...");HCSR04_Init(); USART1_SendString("OK\r\n");
    USART1_SendString("[INIT] DHT11..."); DHT11_Init();  USART1_SendString("OK\r\n");

#if USE_MATRIX_KEY
    USART1_SendString("[INIT] MatrixKey...");
    Matrix_Key_Init();
    USART1_SendString("OK\r\n");
#endif

#if USE_BOARD_KEY
    USART1_SendString("[INIT] BoardKey...");
    KEY_Init();
    USART1_SendString("OK\r\n");
#endif

#if USE_OLED
    USART1_SendString("[INIT] OLED SSD1306...");
    OLED_Init();
    USART1_SendString("OK\r\n");
    OLED_ShowBootProgress(10, "Loading...");
#endif

#if USE_ESP8266
    USART1_SendString("[INIT] ESP8266 WiFi...");
    OLED_ShowBootProgress(30, "WiFi Connecting...");
    if (ESP8266_Init()) {
        USART1_SendString("OK\r\n");
        OLED_ShowBootProgress(50, "WiFi OK");
    } else {
        USART1_SendString("FAIL (no WiFi)\r\n");
        OLED_ShowBootProgress(50, "WiFi OFFLINE");
    }
#endif

#if USE_ESP32_CAM
    USART1_SendString("[INIT] ESP32-CAM...");
    OLED_ShowBootProgress(55, "ESP32-CAM...");
    if (ESP32CAM_Init()) {
        USART1_SendString("OK (online)\r\n");
    } else {
        USART1_SendString("SKIP (not ready)\r\n");
    }
#endif

#if USE_RTC
    USART1_SendString("[INIT] RTC Calendar...");
    OLED_ShowBootProgress(60, "RTC Init...");
    SG_RTC_Init();
    USART1_SendString("OK\r\n");
    /* Wakeup Timer 中断不启用（硬件 INITF 异常），秒刷新由主循环轮询完成 */
#endif

#if USE_MPU6050
    USART1_SendString("[INIT] MPU6050...");
    OLED_ShowBootProgress(65, "Calibrating...");
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
                        OLED_ShowBootProgress(75, "Calibrating...");
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

#if USE_OLED
    OLED_ShowBootProgress(100, "System Ready");
    delay_ms(800);
#endif
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

    MPU6050_GetPhysical(&ax, &ay, &az, &gx, &gy, &gz, &t);

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

static void SampleAllSensors(void)
{
    float temp = 0, humi = 0;
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

    DHT11_Read_Data(&temp, &humi);
    g_sensor.temperature = temp;
    g_sensor.humidity    = humi;
    g_sensor.mq2_adc     = ADC_ReadMQ2();

    /* AT 基线更新 */
    AT_Update(g_sensor.light_adc, g_sensor.mq2_adc, temp, humi);
}

/* ============ 按键处理 ============ */
#if USE_MATRIX_KEY
static void HandleMatrixKey(void)
{
    char key = Key_Scan();
    if (key == 0) return;

    switch (key) {
        case 'A': g_mode = MODE_HOME;  DecisionEngine_SetMode(g_mode); Config_Save(AT_GetBaseline(), g_mode); USART1_SendString("-> MODE HOME\r\n");  break;
        case 'B': g_mode = MODE_AWAY;  DecisionEngine_SetMode(g_mode); Config_Save(AT_GetBaseline(), g_mode); USART1_SendString("-> MODE AWAY\r\n");  break;
        case 'C': g_mode = MODE_SLEEP; DecisionEngine_SetMode(g_mode); Config_Save(AT_GetBaseline(), g_mode); USART1_SendString("-> MODE SLEEP\r\n"); break;
        case 'D':
            g_manual_alert = ALERT_INTRUSION;
            g_last_alarm_tick = 0;   /* 强制立即发送 */
            USART1_SendString("-> MANUAL ALERT TEST\r\n");
            break;
    }
}
#endif

#if USE_BOARD_KEY
static void HandleBoardKey(void)
{
    static uint8_t s_last_key = 0;
    uint8_t key = KEY_Scan();

    if (key == s_last_key) return;   /* 无变化，跳过 */
    s_last_key = key;
    if (key == 0) return;            /* 松开，不触发 */

    switch (key) {
        case S1: g_mode = MODE_HOME;  DecisionEngine_SetMode(g_mode); Config_Save(AT_GetBaseline(), g_mode); USART1_SendString("-> MODE HOME\r\n");  break;
        case S2: g_mode = MODE_AWAY;  DecisionEngine_SetMode(g_mode); Config_Save(AT_GetBaseline(), g_mode); USART1_SendString("-> MODE AWAY\r\n");  break;
        case S3: g_mode = MODE_SLEEP; DecisionEngine_SetMode(g_mode); Config_Save(AT_GetBaseline(), g_mode); USART1_SendString("-> MODE SLEEP\r\n"); break;
        case S4:
            g_manual_alert = ALERT_INTRUSION;
            g_last_alarm_tick = 0;
            USART1_SendString("-> MANUAL ALERT TEST\r\n");
            break;
    }

#if USE_OLED
    /* 按键后立即重绘主界面（Row 1 模式/时间/WiFi/MQTT），避免等 1 秒 tick。
       告警页显示中时不覆盖；代价是一次整屏重绘约 50ms，仅按键时发生 */
    if (g_alert == ALERT_NONE) {
        OLED_ShowHomeScreen(&g_sensor, g_mode, s_time_buf,
                            s_wifi_status, s_mqtt_status);
    }
#endif
}
#endif

/* ============ 主函数 ============ */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART1_Init(115200);

    USART1_SendString("\r\n====================================\r\n");
    USART1_SendString("  SmartGuard v0.4 - Space Guard Agent\r\n");
    USART1_SendString("====================================\r\n");

    Hardware_InitAll();
    delay_ms(1000);

    /* 检测 ESP8266 是否连通外网 */
#if USE_ESP8266
    ESP8266_CheckInternet();

    /* NTP/UDP 时间同步（阿里云 NTP 服务器） */
    USART1_SendString("[RTC] Calling NTP...\r\n");
    {
        RTC_DateTime_t ntp_dt;
        if (ESP8266_GetNTPTime(&ntp_dt)) {
            SG_RTC_SetDateTime(ntp_dt.Year, ntp_dt.Month, ntp_dt.Date,
                               ntp_dt.Hours, ntp_dt.Minutes, ntp_dt.Seconds);
            USART1_Printf("[RTC] NTP synced: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                          ntp_dt.Year, ntp_dt.Month, ntp_dt.Date,
                          ntp_dt.Hours, ntp_dt.Minutes, ntp_dt.Seconds);
            {
                RTC_DateTime_t verify;
                SG_RTC_GetTime(&verify);
                USART1_Printf("[RTC] read-back: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                              verify.Year, verify.Month, verify.Date,
                              verify.Hours, verify.Minutes, verify.Seconds);
            }
        } else {
            USART1_SendString("[RTC] NTP sync failed, use compile time\r\n");
        }
    }
    USART1_SendString("[RTC] NTP call returned\r\n");

    #if USE_MQTT
        /* GetNTPTime internally does AT+CIPCLOSE and reopens UDP.
           Reconnect MQTT TCP link, otherwise data goes nowhere. */
        if (!ESP8266_MQTTConnect())
            USART1_Printf("[MQTT] Reconnect after NTP failed\r\n");
        else if (!ESP8266_MQTTSubscribe())
            USART1_Printf("[MQTT] Resubscribe after NTP failed\r\n");
        else
            s_mqtt_last_ping_tick = 0;   /* 重连成功，心跳计时从零开始 */
    #endif
#endif

    /* 同步初始模式到决策引擎 */
    DecisionEngine_Init();
    DecisionEngine_SetMode(g_mode);

    /* ========== 基线学习（优先从 FLASH 恢复，失败才重新学 20s） ========== */
    AT_Init();

    if (Config_Load((AT_Baseline_t *)AT_GetBaseline(), &g_mode)) {
        /* FLASH 有有效配置，直接恢复，跳过 20s 学习 */
        USART1_SendString("[BASELINE] Restored from FLASH (skip 20s learn)\r\n");
        DecisionEngine_SetMode(g_mode);
    } else {
        /* FLASH 无有效数据，正常学习 20s */
        USART1_SendString("\r\n[BASELINE] Learning 20s baseline...\r\n");
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
    OLED_Clear();
    OLED_ShowString(0, 0, "Baseline Done!");
    OLED_ShowString(0, 2, "Entering main loop...");
#endif

    /* ========== 启动独立看门狗（基线学习完成后再启动，避免 20s 阻塞超时） ========== */
    USART1_SendString("\r\n[INIT] IWDG 2s watchdog...");
    IWDG_Init();
    USART1_SendString("OK\r\n");

    USART1_SendString("\r\n[MAIN] Entering SmartGuard main loop...\r\n\r\n");

#if ML_DATA_COLLECT
    USART1_SendString("CSV_HEADER:tick,light,distance_cm,temp_c,humi_pct,mode,alert\r\n");
#endif

    /* ========== 主循环 ========== */
    uint32_t loop_tick = 0;

    while (1) {
#if USE_MQTT
        ESP8266_MQTTProcessRx();   /* 主循环每次都调，处理下行 PUBLISH */
#endif
        delay_ms(100);
        loop_tick++;

    #if USE_MQTT
        if (loop_tick % 100 == 0) {   /* 100 x 100ms = 10 seconds */
            char mqtt_msg[64];
            sprintf(mqtt_msg, "T=%.0f,H=%.0f,D=%.0f,L=%d,M=%d",
                    g_sensor.temperature, g_sensor.humidity,
                    g_sensor.distance, g_sensor.light_adc,
                    (int)g_sensor.mq2_adc);
            ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, mqtt_msg);
        }

        /* MQTT 心跳 PINGREQ：每 MQTT_KEEPALIVE_SEC / 2 秒发一次 */
        if (loop_tick - s_mqtt_last_ping_tick >= MQTT_KEEPALIVE_SEC * 10 / 2) {
            ESP8266_MQTTPing();
            s_mqtt_last_ping_tick = loop_tick;
        }
    #endif

        /* 防拆检测独立高频采样：200ms 一次（5Hz）。
           必须在下面的 2s 决策块之前：tick 20 同时满足 %2，先刷新标志再交给决策引擎 */
        if (loop_tick % 2 == 0) {
            TamperUpdate();
        }

        /* 每 2s 采集 + 决策 */
        if (loop_tick % 20 == 0) {
            SampleAllSensors();

            /* 决策引擎 */
            g_alert = DecisionEngine_Run(&g_sensor);

            USART1_Printf("[%lus] L=%4d D=%.1fcm T=%.0fC H=%.0f%% | %s | %s\r\n",
                loop_tick / 10,
                g_sensor.light_adc, g_sensor.distance,
                g_sensor.temperature, g_sensor.humidity,
                g_mode == MODE_HOME ? "HOME " : g_mode == MODE_AWAY ? "AWAY " : "SLEEP",
                g_alert == ALERT_NONE ? "OK" :
                g_alert == ALERT_INTRUSION ? "!!INTRUSION!!" :
                g_alert == ALERT_FIRE ? "!!FIRE!!" :
                g_alert == ALERT_TAMPER ? "!!TAMPER!!" : "!!ENV!!");

            /* 运行期阈值真值：学习期那条 "[AT] ... fire_th=" 是一次性快照，
               此后 AT_Update 每 2s 重算阈值，只有这里能看到实时值和 MQ2 余量。
               降频到 10s：阈值靠 EWMA(alpha=0.01) 漂移，是慢变量，2s 粒度只是噪音。
               外层已是 %20（2s），100 是 20 的整数倍，故与 10s 遥测同拍不错位。 */
            if (loop_tick % 100 == 0) {
                USART1_Printf("[DBG] MQ2=%d th=%d T=%.1fC\r\n",
                              g_sensor.mq2_adc, AT_GetBaseline()->mq2_fire_threshold,
                              g_sensor.temperature);
            }

            /* 告警处理 —— 统一出口（决策引擎 + 手动测试） */
            {
                Alert_Type_t effective = (g_manual_alert != ALERT_NONE) ? g_manual_alert : g_alert;
                static Alert_Type_t s_last_effective = ALERT_NONE;
                uint32_t now = loop_tick / 10;
                uint8_t state_changed = (effective != s_last_effective);

                if (effective != ALERT_NONE) {
                    if (state_changed || now - g_last_alarm_tick >= 5) {
                        LED_Toggle(1);

#if USE_ESP8266
    #if USE_MQTT
                        if (effective == ALERT_INTRUSION)
                            ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Intrusion!");
                        else if (effective == ALERT_FIRE)
                            ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Fire!");
                        else if (effective == ALERT_TAMPER)
                            ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:Tamper!");
                        else
                            ESP8266_MQTTPublish(MQTT_TOPIC_SENSOR, "ALERT:EnvAbnormal!");
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
                            } else if (cam_st == CAM_OFFLINE) {
                                USART1_SendString("[ESP32-CAM] offline\r\n");
                            } else {
                                USART1_Printf("[ESP32-CAM] failed (%d)\r\n", cam_st);
                            }
                        }
#endif

                        g_last_alarm_tick = now;
                        g_manual_alert = ALERT_NONE;  /* 手动告警发完即清 */
                    }
                } else {
                    LED_OFF(1);
                }
                s_last_effective = effective;
            }
        }

        /* ============ ESP8266 网络健康检查：每 5 分钟 ============ */
#if USE_ESP8266
        if (loop_tick % 3000 == 0) {
            ESP8266_CheckHealth();
        }
#endif

        /* ============ ML 数据采集：每 2s 输出 CSV 行 ============ */
#if ML_DATA_COLLECT
        if (loop_tick % 20 == 0) {
            uint32_t t = loop_tick / 10;
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

        /* ★ RTC 秒刷新 + OLED 每秒刷新 (每 10 tick = 1 秒)
         * 不再依赖 RTC Wakeup 中断，改为直接读 RTC 日历时间 */
#if USE_RTC
        if (loop_tick % 10 == 0) {
            SG_RTC_FormatTime(s_time_buf, sizeof(s_time_buf));
            USART1_Printf("[RTC] %s\r\n", s_time_buf);
        }
#endif

#if USE_OLED
        if (loop_tick % 10 == 0) {
#if USE_ESP8266
            s_wifi_status = ESP8266_IsWiFiOk() ? "W+" : "W-";
    #if USE_MQTT
            s_mqtt_status = ESP8266_IsMQTTOk() ? "M+" : "M-";
    #endif
#endif
            if (g_alert != ALERT_NONE) {
                OLED_ShowAlertPage(&g_sensor, g_alert, s_time_buf, s_cam_ok);
            } else {
                OLED_ShowHomeScreen(&g_sensor, g_mode, s_time_buf,
                                    s_wifi_status, s_mqtt_status);
            }
        }
#endif

#if USE_MATRIX_KEY
        HandleMatrixKey();
#endif
#if USE_BOARD_KEY
        HandleBoardKey();
#endif

        /* 喂独立看门狗（2s 超时，主循环 100ms tick 足够） */
        IWDG_Feed();
    }
}

