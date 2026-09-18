/**
  ******************************************************************************
  * @file    esp32_cam.h
  * @brief   ESP32-CAM 联动模块头文件
  *
 * 硬件连接（STM32F407ZE ↔ ESP32-CAM）：
 *   STM32 USART2_TX (PA2) ──→ ESP32-CAM U0R (GPIO3)
 *   STM32 USART2_RX (PA3) ◄── ESP32-CAM U0T (GPIO1)
 *   STM32 GND            ──→ ESP32-CAM GND（必须是真 GND：上排5V隔壁 或
 *                             下排IO0与VCC之间那根！下排角落"GND"实为
 *                             GND/R复位脚，接上CAM会被按住复位；
 *                             VCC针是3.3V输出，也不能当地用）
 *   STM32 5V             ──→ ESP32-CAM 5V（注意供电要足！≥500mA）
  *
  * 串口协议（115200, 8N1）：
  *   STM32 → ESP32-CAM: "CAPTURE\r\n"         // 触发拍照
  *   STM32 → ESP32-CAM: "STATUS\r\n"          // 查询在线状态
  *   ESP32-CAM → STM32: "OK http://IP/shot.jpg\r\n"  // 拍照成功，返回图片URL
  *   ESP32-CAM → STM32: "READY\r\n"           // 初始化完成 / STATUS 响应
  ******************************************************************************
  */
#ifndef __ESP32_CAM_H__
#define __ESP32_CAM_H__

#include "stm32f4xx.h"

/* ========== 编译开关 ========== */
#define USE_ESP32_CAM   1     /* 1=启用 ESP32-CAM 联动 */

/* ========== 状态码 ========== */
typedef enum {
    CAM_OK = 0,
    CAM_TIMEOUT = -1,
    CAM_OFFLINE = -2,
    CAM_BUSY = -3
} ESP32CAM_Status_t;

/* ========== 函数声明 ========== */
#if USE_ESP32_CAM
    uint8_t            ESP32CAM_Init(void);           /* 初始化 USART2 + 等 ESP32-CAM READY */
    ESP32CAM_Status_t  ESP32CAM_TriggerCapture(char *url_buf, uint16_t buf_len);  /* 触发拍照，返回图片URL */
    uint8_t            ESP32CAM_IsOnline(void);       /* 检查在线状态 */
#else
    #define ESP32CAM_Init()                0
    #define ESP32CAM_TriggerCapture(url,l) CAM_OFFLINE
    #define ESP32CAM_IsOnline()            0
#endif

#endif /* __ESP32_CAM_H__ */
