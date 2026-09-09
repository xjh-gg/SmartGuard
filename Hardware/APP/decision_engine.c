/**
  ******************************************************************************
  * @file    decision_engine.c
  * @brief   决策引擎 v0.7 —— 规则优先 + AI 辅助（TinyML 可开关）
  *
  * 架构：规则层守门（80% 场景） → 边界地带交给 AI（模糊分流）
  *
  * v0.7 变化：
  *   - 新增 TinyML 集成点（USE_ML 宏开关）
  *   - CheckIntrusion 边界地带（40-80cm + 光照异动）交给 ML
  *   - 需要先跑 20s 基线学习 → 采数据 → 训练 → 设 USE_ML=1
  *
  * v0.5 遗留：
  *   - 条件编译包裹 OLED/ESP8266 依赖（可独立编译）
  *   - 入侵检测优先 AT_CheckIntrusion，fallback 固定阈值
  ******************************************************************************
  */
#include "decision_engine.h"
#include "sensor_hub.h"
#include "adaptive_threshold.h"
#include "led.h"
#include "beep.h"
#include "pwm.h"
#include "usart.h"
#include "string.h"

#if defined(USE_ESP8266) && USE_ESP8266
  #include "esp8266.h"
#endif

#if defined(USE_ML) && USE_ML
  #include "ml_inference.h"
  #include "scaler_params.h"
#endif

static Guard_Mode_t s_mode        = MODE_HOME;
static Alert_Type_t s_alert_last  = ALERT_NONE;
static float        s_dist_prev    = 0.0f;

/* ========== 固定阈值兜底 ========== */
#define INTRUSION_DIST_HARD    40.0f    /* 硬阈值：<40cm 直接告警，不经过 AI */
#define INTRUSION_DIST_SOFT    80.0f    /* 软阈值：40-80cm 交给 AI 辅助判断 */
#define INTRUSION_DELTA_CM     20.0f    /* 距离突变兜底 */

void DecisionEngine_Init(void)
{
    s_mode        = MODE_HOME;
    s_alert_last  = ALERT_NONE;
    s_dist_prev   = 0.0f;

#if defined(USE_ML) && USE_ML
    USART1_SendString("[AI] TinyML MLP (10->16->1) 已启用\r\n");
#endif
}

void DecisionEngine_SetMode(Guard_Mode_t mode)
{
    if (mode != s_mode)
    {
        s_mode = mode;
        switch (mode)
        {
            case MODE_HOME:  USART1_SendString("[MODE] HOME 在家模式 - 入侵检测关\n");   break;
            case MODE_AWAY:  USART1_SendString("[MODE] AWAY 离家模式 - 入侵检测开\n");   break;
            case MODE_SLEEP: USART1_SendString("[MODE] SLEEP 睡眠模式 - 入侵检测开\n"); break;
        }
        Beep_Off();
        LED_OFF(1);
        LED_OFF(2);
        s_alert_last  = ALERT_NONE;
        s_dist_prev   = 0.0f;
    }
}

Guard_Mode_t DecisionEngine_GetMode(void)
{
    return s_mode;
}

/**
  * @brief  入侵判断 —— 规则优先 + AI 辅助
  *
  *   硬阈值（距离 < 40cm）  → 直接 ALERT_INTRUSION，跳过 AI
  *   AT 自适应触发          → 直接 ALERT_INTRUSION
  *   边界地带（40-80cm）    → 调 ML 辅助判断（如果启用了）
  *   兜底（距离突变 >20cm） → ALERT_INTRUSION
  */
static Alert_Type_t CheckIntrusion(const Sensor_Data_t *data)
{
    if (s_mode == MODE_HOME) return ALERT_NONE;
    if (data->distance <= 0.0f) return ALERT_NONE;

    /* ===== 1. AT 自适应阈值（硬触发） ===== */
    const AT_Baseline_t *bl = AT_GetBaseline();
    if (bl && bl->is_learned)
    {
        if (AT_CheckIntrusion(data, s_dist_prev, data->distance))
        {
            return ALERT_INTRUSION;
        }
    }

    float delta = s_dist_prev - data->distance;

    /* ===== 2. 硬阈值（直接过，不调 AI） ===== */
    if (data->distance < INTRUSION_DIST_HARD)
    {
        return ALERT_INTRUSION;
    }

    /* ===== 3. 边界地带（40-80cm）→ AI 辅助 ===== */
    if (data->distance < INTRUSION_DIST_SOFT)
    {
#if defined(USE_ML) && USE_ML
        {
            float x_raw[4]  = { (float)data->light_adc, data->distance, data->temperature, data->humidity };
            float baseline[4];
            const AT_Baseline_t *bl = AT_GetBaseline();
            baseline[0] = bl && bl->is_learned ? bl->light_ewma  : 1200.0f;
            baseline[1] = bl && bl->is_learned ? 0.0f            : 208.0f;  /* distance ewma 没存，用中位数 */
            baseline[2] = bl && bl->is_learned ? bl->temp_ewma   : 29.0f;
            baseline[3] = bl && bl->is_learned ? bl->humi_ewma   : 44.0f;
            uint8_t pred = ml_predict(x_raw, baseline, SCALER_MEAN, SCALER_SCALE);
            if (pred) {
                USART1_Printf("[AI] ML=INTRUSION dist=%.1fcm\r\n", data->distance);
                return ALERT_INTRUSION;
            }
        }
#endif
    }

    /* ===== 4. 兜底：距离突变 ===== */
    if (delta > INTRUSION_DELTA_CM && data->distance < 80.0f)
    {
        return ALERT_INTRUSION;
    }

    return ALERT_NONE;
}

/**
  * @brief  火灾判断（MQ-2 + DHT11）
  */
static Alert_Type_t CheckFire(const Sensor_Data_t *data)
{
    if (data->mq2_adc == 0 || data->temperature <= 0.0f) return ALERT_NONE;

    const AT_Baseline_t *bl = AT_GetBaseline();
    if (bl && bl->is_learned)
    {
        if (AT_CheckFire(data)) return ALERT_FIRE;
    }
    else
    {
        if (data->mq2_adc > 400 && data->temperature > 30.0f)
        {
            return ALERT_FIRE;
        }
    }

    return ALERT_NONE;
}

/**
  * @brief  防拆/震动检测（MPU6050）
  *         仅在离家/睡眠模式下启用；在家模式不触发
  */
static Alert_Type_t CheckTamper(const Sensor_Data_t *data)
{
    if (s_mode == MODE_HOME) return ALERT_NONE;
    if (data->tamper_flag) {
        USART1_SendString("[ALERT] 防拆告警: 检测到震动或移动\r\n");
        return ALERT_TAMPER;
    }
    return ALERT_NONE;
}

/**
  * @brief  光敏自动照明
  */
static void AutoLighting(const Sensor_Data_t *data)
{
    /* 板载光敏方向（adc.c 已核实）：光照越强 ADC 值越小、越暗值越大。
       PWM 低电平点亮、CCR 越大越亮，因此直接用 ADC 值映射占空比：
       白天 ADC 小 -> 占空比小 -> 灯暗；夜晚 ADC 大 -> 占空比大 -> 灯亮。
       若实测方向相反（手遮挡光敏灯反而变暗），把 data->light_adc 换成 4095-data->light_adc。 */
    uint16_t duty = (uint16_t)((uint32_t)data->light_adc * PWM_PERIOD / 4095);
    PWM_Home_SetDuty(duty);
}

Alert_Type_t DecisionEngine_Run(const Sensor_Data_t *data)
{
    Alert_Type_t alert = ALERT_NONE;

    /* 1. 自动照明 */
    AutoLighting(data);

    /* 2. 火灾检测 */
    alert = CheckFire(data);

    /* 3. 入侵检测 */
    if (alert == ALERT_NONE)
    {
        alert = CheckIntrusion(data);
    }

    /* 4. 防拆/震动检测 */
    if (alert == ALERT_NONE)
    {
        alert = CheckTamper(data);
    }

    /* 5. 告警动作（状态变化才触发） */
    if (alert != s_alert_last)
    {
        switch (alert)
        {
            case ALERT_INTRUSION:
                /* Beep_On(); */     /* 蜂鸣器已关闭 */
                LED_ON(1);
                LED_OFF(2);
                /* 注意：源文件为无 BOM UTF-8，ARMCC 按 GBK 扫描，全角标点须保持
                   字节配对合法，故此处用半角 '!'（改回全角'！'会触发 #870-D） */
                USART1_Printf("[ALERT] 入侵告警! 距离=%.1fcm\r\n", data->distance);
#if defined(USE_ESP8266) && USE_ESP8266
                ESP8266_SendAlert("intrusion");
#endif
                break;

            case ALERT_FIRE:
                /* Beep_On(); */     /* 蜂鸣器已关闭 */
                LED_ON(1);
                LED_ON(2);
                USART1_Printf("[ALERT] 火灾告警！MQ2=%d T=%.1fC\r\n",
                              data->mq2_adc, data->temperature);
#if defined(USE_ESP8266) && USE_ESP8266
                ESP8266_SendAlert("fire");
#endif
                break;

            case ALERT_TAMPER:
                /* Beep_On(); */     /* 蜂鸣器已关闭 */
                LED_ON(1);
                LED_OFF(2);
                USART1_SendString("[ALERT] 防拆告警！\r\n");
#if defined(USE_ESP8266) && USE_ESP8266
                ESP8266_SendAlert("tamper");
#endif
                break;

            default:
                Beep_Off();
                LED_OFF(1);
                LED_OFF(2);
                USART1_SendString("[OK] 环境正常\n");
                break;
        }
        s_alert_last = alert;
    }

    s_dist_prev = data->distance;
    return alert;
}

const char *DecisionEngine_GetAlertString(Alert_Type_t alert)
{
    switch (alert)
    {
        case ALERT_FIRE:          return "FIRE";
        case ALERT_INTRUSION:     return "INTRUSION";
        case ALERT_ENV_ABNORMAL:  return "ENV";
        case ALERT_TAMPER:        return "TAMPER";
        default:                  return "OK";
    }
}



