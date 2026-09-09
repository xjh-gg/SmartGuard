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

/* ========== WiFi 配置（使用前按实际环境修改） ========== */
#define ESP8266_WIFI_SSID       "LAPTOP-KNLKBLMS 0997"
#define ESP8266_WIFI_PASSWORD   "6Z5{58n8"

/* ========== UDP 目标服务器（用于数据上报和告警） ========== */
#define ESP8266_UDP_SERVER_IP   "192.168.137.1"   /* 电脑热点网关 IP */
#define ESP8266_UDP_SERVER_PORT 12345             /* 电脑 Python 接收端口 */

/* ========== 时间服务器配置 ========== */
#define ESP8266_HTTP_TIME_IP     "20.49.104.6"        /* worldclockapi.com 固定 IP */
#define ESP8266_HTTP_TIME_HOST   "worldclockapi.com"  /* HTTP Host 头 */
#define ESP8266_HTTP_TIME_PATH   "/api/json/utc/now"  /* 返回 UTC 时间 */
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
uint8_t ESP8266_GetHTTPTime(RTC_DateTime_t *dt);       /* HTTP 方式获取网络时间（UTC+8） */
uint8_t ESP8266_GetNTPTime(RTC_DateTime_t *dt);        /* NTP/UDP 方式获取网络时间（UTC+8） */
uint8_t ESP8266_CheckInternet(void);
uint8_t ESP8266_CheckHealth(void);

#endif /* __ESP8266_H__ */
