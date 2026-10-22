#ifndef __DECISION_ENGINE_H
#define __DECISION_ENGINE_H

#include "stm32f4xx.h"
#include "sensor_hub.h"

/* ============ 硬件开关（全局有效，所有 .c 文件可见） ============ */
#define USE_OLED        1   /* 0=没接OLED  1=接了OLED（SSD1306 软件 SPI，引脚以 oled.h 为准） */
#define USE_ESP8266     1   /* 0=没接WiFi  1=接了ESP8266 (USART3)   */
#define USE_DHT11       1   /* 0=板载没接  1=接了DHT11温湿度 (PG9)  */
#define USE_ML          1   /* 0=纯规则  1=TinyML MLP 辅助 (v1 二分类: 正常/入侵) */
#define USE_RTC         1   /* 0=关闭RTC  1=启用（已修复 Wakeup Timer Bug：EXTI Line22 + 计数器0 + WUTWF等待） */
/* MQ-2 烟雾开关在 Hardware/ADC/adc.h 的 USE_MQ2（驱动就近自持） */
/* ============================================================== */

typedef enum {
    MODE_HOME = 0,
    MODE_AWAY,
    MODE_SLEEP
} Guard_Mode_t;

typedef enum {
    ALERT_NONE = 0,
    ALERT_INTRUSION,
    ALERT_FIRE,
    ALERT_ENV_ABNORMAL,
    ALERT_TAMPER
} Alert_Type_t;

void DecisionEngine_Init(void);
void DecisionEngine_SetMode(Guard_Mode_t mode);
Guard_Mode_t DecisionEngine_GetMode(void);
Alert_Type_t DecisionEngine_Run(const Sensor_Data_t *data);
const char *DecisionEngine_GetAlertString(Alert_Type_t alert);

/* 重新进入布防静息采样（手动重新学习基线完成后调用）：
   清空静息距离，重新给约 10s 布防缓冲采集"无人时的距离" */
void DecisionEngine_RestartArm(void);

#endif

