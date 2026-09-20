/**
  ******************************************************************************
  * @file    esp32_cam.c
  * @brief   ESP32-CAM 联动模块实现
  *
  * 工作流程：
  *   1. Init() — 初始化 USART2（PA2/PA3, 115200），等待 ESP32-CAM 上电后回 "READY"
  *   2. TriggerCapture() — 发 "CAPTURE\r\n"，等 ESP32-CAM 回 URL
  *   3. IsOnline() — 发 "STATUS\r\n"，检查是否回 "READY"
  *
  * 容错设计：
  *   - ESP32-CAM 可能离线（未上电/启动中），所有函数有超时保护
  *   - 不阻塞主循环，用 delay_ms 轮询接收
  *   - 离线时返回 CAM_OFFLINE，不影响 SmartGuard 正常运行
  ******************************************************************************
  */
#include "esp32_cam.h"
#include "usart.h"
#include "delay.h"
#include "iwdg.h"
#include <string.h>

#if USE_ESP32_CAM

/* ========== 内部状态 ========== */
static uint8_t cam_online = 0;   /* ESP32-CAM 是否就绪 */

/* ========== 接收一行字符串（直到 '\r' 或超时） ========== */
static uint8_t ReadLine(char *buf, uint16_t buf_len, uint32_t timeout_ms)
{
    uint16_t idx = 0;
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms && idx < buf_len - 1)
    {
        uint8_t ch;
        if (USART2_GetByte(&ch))
        {
            if (ch == '\r') continue;     /* 跳过 \r */
            if (ch == '\n') break;         /* \n 结束一行 */
            buf[idx++] = ch;
        }
        else
        {
            /* 轮询间隙喂狗：TriggerCapture 的等待最长 3000ms > IWDG 2s，
               不喂狗会在 CAM 慢响应时误复位。timeout_ms 是硬上界，
               CAM 无响应仍按时超时退出，不会因喂狗变成永不复位。 */
            IWDG_Feed();
            delay_ms(1);
            elapsed++;
        }
    }
    buf[idx] = '\0';
    return (idx > 0);   /* 1=收到了数据，0=超时 */
}

/**
  * @brief  初始化 ESP32-CAM 模块
  *         - 初始化 USART2 (PA2/PA3, 115200)
  *         - 发 STATUS 等 READY，最多等 5s（ESP32-CAM WiFi 连接需要时间）
  * @retval 1=成功，0=ESP32-CAM 未就绪
  */
uint8_t ESP32CAM_Init(void)
{
    /* 初始化 USART2 */
    USART2_Init(115200);
    delay_ms(100);
    USART2_FlushRx();

    /* 最多等 5s，期间每秒发一次 STATUS */
    char rx_buf[64];
    for (int retry = 0; retry < 5; retry++)
    {
        USART2_FlushRx();
        USART2_SendString("STATUS\r\n");
        if (ReadLine(rx_buf, sizeof(rx_buf), 1000))
        {
            if (strstr(rx_buf, "READY") != NULL)
            {
                cam_online = 1;
                return 1;
            }
        }
        delay_ms(1000);
    }

    cam_online = 0;
    return 0;   /* ESP32-CAM 未就绪（可能没上电），不影响主流程 */
}

/**
  * @brief  触发 ESP32-CAM 拍照
  * @param  url_buf: 输出缓冲区，接收图片 URL（如 "http://192.168.1.100/shot.jpg"）
  * @param  buf_len: url_buf 长度
  * @retval ESP32CAM_Status_t
  */
ESP32CAM_Status_t ESP32CAM_TriggerCapture(char *url_buf, uint16_t buf_len)
{
    if (!cam_online)
    {
        /* 离线时尝试重新探测一次 */
        char rx[32];
        USART2_SendString("STATUS\r\n");
        if (ReadLine(rx, sizeof(rx), 1000) && strstr(rx, "READY") != NULL)
        {
            cam_online = 1;
        }
        else
        {
            return CAM_OFFLINE;
        }
    }

    /* 发拍照命令 */
    USART2_FlushRx();
    USART2_SendString("CAPTURE\r\n");

    /* 等待响应（最多 3s，ESP32-CAM 拍照+存 SD 需要时间） */
    char rx_buf[128];
    if (!ReadLine(rx_buf, sizeof(rx_buf), 3000))
    {
        /* 超时说明 CAM 大概率已死机/掉线：清零在线标志，
           让下次走 1s 快速重探测分支，而不是每次 3s 满超时（P3-#7） */
        cam_online = 0;
        return CAM_TIMEOUT;
    }

    /* 解析响应："OK http://xxx/shot.jpg" 或 "OK" */
    if (strncmp(rx_buf, "OK", 2) == 0)
    {
        /* 找 URL 部分 */
        char *url_start = strstr(rx_buf, "http");
        if (url_start && url_buf)
        {
            strncpy(url_buf, url_start, buf_len - 1);
            url_buf[buf_len - 1] = '\0';
        }
        return CAM_OK;
    }

    return CAM_BUSY;
}

/**
  * @brief  检查 ESP32-CAM 在线状态
  */
uint8_t ESP32CAM_IsOnline(void)
{
    char rx[32];
    USART2_SendString("STATUS\r\n");
    if (ReadLine(rx, sizeof(rx), 500) && strstr(rx, "READY") != NULL)
    {
        cam_online = 1;
        return 1;
    }
    cam_online = 0;
    return 0;
}

#endif /* USE_ESP32_CAM */
