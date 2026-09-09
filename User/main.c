/**
  * @file    main.c
  * @brief   SmartGuard v0.4 — 智能空间守护 Agent
  *
  * 硬件清单 (已全部验证):
  *   ADC         — 光敏电阻 PF7/ADC3_IN5, 电位器 PA5/ADC1_IN5
  *   HCSR04      — Trig=PC8, Echo=PC6
  *   DHT11       — PG9 (一次读回 temp+humi)
  *   MatrixKey   — COL: PB7/PA4/PG15/PC7, ROW: PC9/PB6/PE6/PA8
  *   LED         — PF9/PF10/PE13/PE14 (低电平亮), BEEP: PF8
  *   OLED SSD1306 — PB0=CLK PF11=MOSI PB1=DC PC13=CS1, CS2接VCC
  *   ESP8266     — USART3 PB10/PB11, 115200bps
  */

/* ============ 编译开关（必须在 include 之前定义）============ */
#define USE_MATRIX_KEY  1
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
#include "matrix_key.h"
#include "sensor_hub.h"
#include "decision_engine.h"   /* 已经 include USE_OLED/USE_ESP8266 定义 */
#include "adaptive_threshold.h"
#include "oled.h"
#include "iwdg.h"
#include "config_store.h"

#if USE_RTC
#include "rtc.h"
#endif

#if USE_ESP8266
#include "esp8266.h"
#endif

#if USE_ESP32_CAM
#include "esp32_cam.h"
#endif

#if USE_MPU6050
#include "mpu6050.h"
#endif

/* ============ 全局状态 ============ */
static Sensor_Data_t g_sensor;
static Guard_Mode_t  g_mode    = MODE_AWAY;   /* 默认离家模式：入侵检测开启 */
static Alert_Type_t  g_alert   = ALERT_NONE;
static uint32_t      g_last_alarm_tick = 0;

/* MPU6050 零偏校准值（初始化时学习） */
static float   s_gyro_zero_gx = 0.0f;
static float   s_gyro_zero_gy = 0.0f;
static float   s_gyro_zero_gz = 0.0f;
static uint8_t s_tamper_hit_count = 0;  /* 连续触发计数，≥2 才告警 */

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

#if USE_OLED
    USART1_SendString("[INIT] OLED SSD1306...");
    OLED_Init();
    USART1_SendString("OK\r\n");
    OLED_ShowString(0, 0, "SmartGuard v0.4");
    OLED_ShowString(0, 2, "Booting...");
#endif

#if USE_ESP8266
    USART1_SendString("[INIT] ESP8266 WiFi...");
    if (ESP8266_Init()) {
        USART1_SendString("OK\r\n");
    } else {
        USART1_SendString("FAIL (no WiFi)\r\n");
    }
#endif

#if USE_ESP32_CAM
    USART1_SendString("[INIT] ESP32-CAM...");
    if (ESP32CAM_Init()) {
        USART1_SendString("OK (online)\r\n");
    } else {
        USART1_SendString("SKIP (not ready)\r\n");
    }
#endif

#if USE_RTC
    USART1_SendString("[INIT] RTC Calendar...");
    SG_RTC_Init();
    USART1_SendString("OK\r\n");
    /* Wakeup Timer 中断不启用（硬件 INITF 异常），秒刷新由主循环轮询完成 */
#endif

#if USE_MPU6050
    USART1_SendString("[INIT] MPU6050...");
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
}

/* ============ 采集所有传感器 ============ */
static void SampleAllSensors(void)
{
    float temp = 0, humi = 0;

    g_sensor.light_adc = ADC_ReadPhotoresistor();   /* PF7/ADC3_IN5 */
    g_sensor.distance  = HCSR04_GetDistance();
    DHT11_Read_Data(&temp, &humi);
    g_sensor.temperature = temp;
    g_sensor.humidity    = humi;
    g_sensor.tamper_flag = 0;

#if USE_MPU6050
    {
        float ax, ay, az, gx, gy, gz, t;
        float gx_off, gy_off, gz_off;
        uint8_t hit = 0;

        MPU6050_GetPhysical(&ax, &ay, &az, &gx, &gy, &gz, &t);

        /* 减去零偏 */
        gx_off = gx - s_gyro_zero_gx;
        gy_off = gy - s_gyro_zero_gy;
        gz_off = gz - s_gyro_zero_gz;

        /* 防拆/震动判断：仅用陀螺仪（加速度计在部分 GY-521 模块上不稳定）
           偏离零偏 > ±25 dps → 有人碰板子
           必须连续 3 次触发才告警（防抖，约 6 秒） */
        if ((gx_off >  25.0f || gx_off < -25.0f) ||
            (gy_off >  25.0f || gy_off < -25.0f) ||
            (gz_off >  25.0f || gz_off < -25.0f))
        {
            if (s_tamper_hit_count < 4) s_tamper_hit_count++;
            if (s_tamper_hit_count >= 3) hit = 1;
        }
        else
        {
            if (s_tamper_hit_count > 0) s_tamper_hit_count--;
        }

        if (hit) {
            g_sensor.tamper_flag = 1;
            USART1_Printf("[TAMPER_DBG] ax=%.2f ay=%.2f az=%.2f gx=%.1f gy=%.1f gz=%.1f | off: gx=%.1f gy=%.1f gz=%.1f cnt=%d\r\n",
                          ax, ay, az, gx, gy, gz, gx_off, gy_off, gz_off, s_tamper_hit_count);
        }
    }
#endif

    /* AT 基线更新 */
    AT_Update(g_sensor.light_adc, 0, temp, humi);
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
            g_alert = ALERT_INTRUSION;
            /* BEEP OFF */
            USART1_SendString("-> MANUAL ALERT TEST\r\n");
            break;
    }
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
        } else {
            USART1_SendString("[RTC] NTP sync failed, use compile time\r\n");
        }
    }
    USART1_SendString("[RTC] NTP call returned\r\n");
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
        delay_ms(100);
        loop_tick++;

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

            /* 告警处理 */
            if (g_alert != ALERT_NONE) {
                uint32_t now = loop_tick / 10;
                if (now - g_last_alarm_tick >= 5) {
                    /* BEEP OFF */
                    LED_Toggle(1);

#if USE_ESP8266
                    if (g_alert == ALERT_INTRUSION)
                        ESP8266_SendAlert("Intrusion!");
                    else if (g_alert == ALERT_FIRE)
                        ESP8266_SendAlert("Fire!");
                    else if (g_alert == ALERT_TAMPER)
                        ESP8266_SendAlert("Tamper!");
                    else
                        ESP8266_SendAlert("EnvAbnormal!");
#endif

#if USE_ESP32_CAM
                    /* 入侵告警时触发 ESP32-CAM 拍照 */
                    if (g_alert == ALERT_INTRUSION) {
                        char cam_url[96];
                        ESP32CAM_Status_t cam_st = ESP32CAM_TriggerCapture(cam_url, sizeof(cam_url));
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
                }
            } else {
                LED_OFF(1);
            }
        }

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

        /* OLED 每秒刷新 + 告警闪烁 */
#if USE_OLED
        if (loop_tick % 10 == 0) {
            if (g_alert != ALERT_NONE && (loop_tick / 10) % 2 == 1) {
                /* 告警闪烁：同时清 page 6 和 page 7 (8x16 字体占 2 页) */
                OLED_ClearArea(0, 6, 128);
                OLED_ClearArea(0, 7, 128);
            } else {
                OLED_ShowHomeScreen(&g_sensor, g_mode, g_alert);
            }
        }
#endif

        /* ★ RTC 秒刷新 —— 主循环轮询 (每 10 tick = 1 秒)
         * 不再依赖 RTC Wakeup 中断（硬件 INITF 反复进入 Init Mode 导致中断不触发）
         * 改为直接读 RTC 日历时间，稳定可靠 */
#if USE_RTC
        if (loop_tick % 10 == 0) {
            char time_buf[10];
            SG_RTC_FormatTime(time_buf, sizeof(time_buf));
            USART1_Printf("[RTC] %s\r\n", time_buf);
#if USE_OLED
            OLED_ClearArea(40, 0, 72);
            OLED_ShowString(40, 0, time_buf);
#endif
        }
#endif

#if USE_MATRIX_KEY
        HandleMatrixKey();
#endif

        /* 喂独立看门狗（2s 超时，主循环 100ms tick 足够） */
        IWDG_Feed();
    }
}

