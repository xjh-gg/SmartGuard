/**
  ******************************************************************************
  * @file    esp8266.h
  * @brief   ESP8266 WiFi 模块驱动头文件（UDP 数据上报）
  *
  * 硬件连接（STM32F407ZE）：
  *   STM32 PB10 (USART3_TX) -> ESP8266 RXD
  *   STM32 PB11 (USART3_RX) <- ESP8266 TXD
  *   GND 共地
  *   ESP8266 供电：3.3V（部分模块可 5V，请按模块规格）
  *
  * 说明：
  *   本驱动使用 ESP8266 的 AT 指令，通过 UDP 把传感器数据和告警信息
  *   发送到指定服务器。如需 MQTT，可在 UDP 接收端做协议转换，或升级
  *   ESP8266 为支持 MQTT AT 指令的固件。
  ******************************************************************************
  */
#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "stm32f4xx.h"
#include "rtc.h"
#include "wifi_config.h"
#include "bemfa_config.h"

/* ========== WiFi 配置 ==========
 * SSID 与密码不在本文件（本文件入 git）。它们外置在同目录的 wifi_config.h，
 * 该文件已列入 .gitignore，只存在于本地工作区。
 * 新克隆后需复制 wifi_config.h.example 为 wifi_config.h 并填真实值，否则编译不过。
 * 代码里使用的外置宏：WIFI_SSID / WIFI_PASSWORD
 * ====================================== */

/* ========== UDP 目标服务器（用于数据上报和告警） ========== */
#define ESP8266_UDP_SERVER_IP   "192.168.137.1"   /* 电脑热点网关 IP */
#define ESP8266_UDP_SERVER_PORT 12345             /* 电脑 Python 接收端口 */

/* ========== 时间服务器配置 ========== */
#define ESP8266_TIMEZONE_HOUR   8                     /* 东八区 Beijing/Shanghai */

/* ========== NTP 时间同步配置 ========== */
#define ESP8266_NTP_SERVER_IP   "203.107.6.88"        /* ntp.aliyun.com */
#define ESP8266_NTP_SERVER_PORT 123                    /* NTP 标准端口 */

/* ========== 外网检测配置 ========== */
#define ESP8266_PING_TARGET     "223.5.5.5"           /* 阿里公共 DNS */

/* ========== 接口函数 ========== */
uint8_t ESP8266_Init(void);
uint8_t ESP8266_SendAT(char *cmd, char *ack, uint16_t timeout_ms);
void    ESP8266_Reset(void);
uint8_t ESP8266_SetMode(uint8_t mode);
uint8_t ESP8266_ConnectAP(char *ssid, char *password);
uint8_t ESP8266_ConnectUDP(char *ip, uint16_t port);
uint8_t ESP8266_ConnectTCP(char *ip, uint16_t port);   /* TCP 连接 */
uint8_t ESP8266_SendUDP(uint8_t *data, uint16_t len);
uint8_t ESP8266_SendTCP(uint8_t *data, uint16_t len); /* 发送 TCP 数据 */
int16_t ESP8266_Receive(uint8_t *buf, uint16_t len, uint16_t timeout_ms);

void    ESP8266_SendAlert(const char *msg);
void    ESP8266_SendData(const char *topic, const char *payload);
uint8_t ESP8266_GetNTPTime(RTC_DateTime_t *dt);        /* NTP/UDP 方式获取网络时间（UTC+8） */
uint8_t ESP8266_CheckInternet(void);
uint8_t ESP8266_CheckHealth(void);
uint8_t ESP8266_IsWiFiOk(void);        /* 返回 WiFi 连接状态，供 OLED 状态栏使用 */
uint8_t ESP8266_IsMQTTOk(void);       /* 返回 MQTT 会话状态，供 OLED 状态行使用 */

/* ========== MQTT Broker 配置（替代原巴法云） ========== */
#define USE_MQTT              1
#define MQTT_SERVER           "192.168.137.1"
#define MQTT_PORT             1883
#define MQTT_CLIENT_ID        "stm32_light_01"
#define MQTT_TOPIC_CMD        "home/light/cmd"
#define MQTT_TOPIC_STATUS     "home/light/status"     /* 灯状态 on/off */
#define MQTT_TOPIC_SENSOR     "home/sensor/status"    /* 遥测 + 告警 */
#define MQTT_TOPIC_MODE_STATUS "home/mode/status"       /* 模式状态 HOME/AWAY/SLEEP，retain */
#define MQTT_TOPIC_CAM_SHOT   "home/cam/shot"           /* 入侵拍照图片 URL，retain */
#define MQTT_KEEPALIVE_SEC    60

/* ---------- 原巴法云配置（已下线，保留供参考） ----------
 * UID/TOPIC 已外置到同目录 bemfa_config.h（该文件不入库，模板见 bemfa_config.h.example）。
#define USE_BEMFA             1
#define BEMFA_SERVER          "tcp.bemfa.com"
#define BEMFA_PORT            8344
uint8_t ESP8266_BemfaConnect(void);
uint8_t ESP8266_BemfaSubscribe(void);
uint8_t ESP8266_BemfaPublish(const char *msg);
uint8_t ESP8266_BemfaHeartbeat(void);
------------------------------------------------------- */

uint8_t ESP8266_MQTTConnect(void);       /* TCP 连接 + MQTT CONNECT */
uint8_t ESP8266_MQTTSubscribe(void);     /* SUBSCRIBE 主题 */
uint8_t ESP8266_MQTTPublish(const char *topic, const char *payload, uint8_t retain);
uint8_t ESP8266_MQTTPing(void);          /* PINGREQ 心跳 */
void    ESP8266_MQTTProcessRx(void);     /* 处理 +IPD 帧中的 MQTT 下行包 */

/* 外部回调：收到 home/light/cmd 的 PUBLISH 时由 main.c 实现 */
extern void MQTT_OnCmdReceived(const char *payload);
extern void MQTT_ReportLightStatus(void);   /* 会话就绪时由 esp8266.c 回调，内部发 retained */
extern void MQTT_ReportModeStatus(void);    /* 同上报模式状态到 home/mode/status (retain) */

#endif /* __ESP8266_H__ */
