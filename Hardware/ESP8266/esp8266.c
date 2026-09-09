/**
  ******************************************************************************
  * @file    esp8266.c
  * @brief   ESP8266 WiFi 模块驱动实现（UDP 数据上报）
  *
  * 说明：
  *   所有 AT 指令均通过 USART3 发送，ESP8266 应答也经 USART3 接收。
  *   接收采用中断 + 环形缓冲区，主循环通过 USART3_GetByte 读取。
  ******************************************************************************
  */
#include "esp8266.h"
#include "usart.h"
#include "delay.h"
#include "iwdg.h"
#include <string.h>
#include <stdio.h>

#define U1_PRINTF  USART1_Printf   /* 方便切换到 USART2/其他串口调试 */

static uint8_t  s_send_fail_count = 0;  /* 连续发送失败计数，用于健康检查 */

/**
  * @brief  初始化 ESP8266 通信接口并连接 WiFi
  * @note   固定使用 USART3 115200 波特率；先测 AT、再连 WiFi、最后可选连 UDP
  * @retval 1 = WiFi 连接成功，0 = 失败
  */
uint8_t ESP8266_Init(void)
{
    USART3_Init(115200);

    /* 1. 发送 AT 检查 ESP8266 是否活着 */
    if (!ESP8266_SendAT("AT\r\n", "OK", 1000))
    {
        /* 没响应就复位一次再试 */
        ESP8266_Reset();
        if (!ESP8266_SendAT("AT\r\n", "OK", 1000))
        {
            return 0;   /* ESP8266 彻底没反应 */
        }
    }

    /* 2. 连接 WiFi */
    if (!ESP8266_ConnectAP(ESP8266_WIFI_SSID, ESP8266_WIFI_PASSWORD))
    {
        return 0;   /* WiFi 连接失败 */
    }

    /* 3. 建立 UDP 连接到电脑（用于推告警） */
    U1_PRINTF("[...] ESP8266 connecting UDP to %s:%d...\r\n",
              ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT);
    if (ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT))
    {
        U1_PRINTF("[OK] ESP8266 UDP connected\r\n");
    }
    else
    {
        U1_PRINTF("[FAIL] ESP8266 UDP connect failed\r\n");
    }

    return 1;   /* WiFi 已连上，UDP 失败不阻塞主流程 */
}

/**
  * @brief  发送 AT 指令并等待指定应答
  * @param  cmd:    要发送的 AT 指令字符串（需已包含 \r\n）
  * @param  ack:    期望收到的应答字符串，NULL 表示不等待应答
  * @param  timeout_ms: 超时时间（毫秒）
  * @retval 1 = 收到应答，0 = 超时
  */
uint8_t ESP8266_SendAT(char *cmd, char *ack, uint16_t timeout_ms)
{
    uint8_t rx_buf[256];
    uint16_t rx_idx = 0;
    uint8_t ch;

    USART3_FlushRx();
    memset(rx_buf, 0, sizeof(rx_buf));

    if (cmd != NULL)
    {
        USART3_SendString(cmd);
    }

    if (ack == NULL)
    {
        return 1;
    }

    while (timeout_ms--)
    {
        while (USART3_GetByte(&ch))
        {
            if (rx_idx < sizeof(rx_buf) - 1)
            {
                rx_buf[rx_idx++] = ch;
                rx_buf[rx_idx] = '\0';
            }
        }

        if (strstr((char *)rx_buf, ack) != NULL)
        {
            return 1;
        }

        IWDG_Feed();
        delay_ms(1);
    }

    return 0;
}

/**
  * @brief  软件复位 ESP8266
  */
void ESP8266_Reset(void)
{
    ESP8266_SendAT("AT+RST\r\n", "ready", 3000);
    delay_ms(2000);   /* 复位后需要等待启动完成 */
}

/**
  * @brief  设置 ESP8266 工作模式
  * @param  mode: 1=STA  Station 模式
  *              2=AP   Access Point 模式
  *              3=AP+STA 混合模式
  */
uint8_t ESP8266_SetMode(uint8_t mode)
{
    char cmd[32];
    sprintf(cmd, "AT+CWMODE_CUR=%d\r\n", mode);
    return ESP8266_SendAT(cmd, "OK", 1000);
}

/**
  * @brief  STA 模式连接指定 WiFi
  */
uint8_t ESP8266_ConnectAP(char *ssid, char *password)
{
    char cmd[128];

    if (!ESP8266_SetMode(1))
    {
        return 0;
    }

    sprintf(cmd, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", ssid, password);
    return ESP8266_SendAT(cmd, "OK", 15000);   /* 连接热点可能需要较长时间 */
}

/**
  * @brief  UDP 连接（单连接模式）
  * @param  ip:   目标 IP 地址字符串
  * @param  port: 目标端口号
  */
uint8_t ESP8266_ConnectUDP(char *ip, uint16_t port)
{
    char cmd[128];
    uint8_t ok;

    /* UDP 使用单连接模式 */
    U1_PRINTF("[...] ESP8266 AT+CIPMUX=0\r\n");
    ok = ESP8266_SendAT("AT+CIPMUX=0\r\n", "OK", 2000);
    U1_PRINTF("  -> CIPMUX=0 %s\r\n", ok ? "OK" : "FAIL");
    if (!ok) {
        /* 可能已经是 0 了，继续 */
    }
    delay_ms(200);

    sprintf(cmd, "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n", ip, port);
    U1_PRINTF("[...] ESP8266 %s", cmd);
    ok = ESP8266_SendAT(cmd, "OK", 5000);
    U1_PRINTF("  -> CIPSTART UDP %s\r\n", ok ? "OK" : "FAIL (may be firewall, try anyway)");

    /* 即使 CIPSTART 失败也返回 1 — UDP 无连接，ESP8266 可能仍能发包 */
    return 1;
}

/**
  * @brief  发送 UDP 数据
  * @param  data: 数据缓冲区
  * @param  len:  数据长度
  */
uint8_t ESP8266_SendUDP(uint8_t *data, uint16_t len)
{
    char cmd[32];
    uint16_t i;

    sprintf(cmd, "AT+CIPSEND=%d\r\n", len);
    if (!ESP8266_SendAT(cmd, ">", 1000))
    {
        s_send_fail_count++;
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        USART3_SendByte(data[i]);
    }

    s_send_fail_count = 0;
    return 1;
}

/**
  * @brief  从 ESP8266 接收网络数据（解析 +IPD 帧）
  * @param  buf:    接收数据缓冲区
  * @param  len:    缓冲区最大长度
  * @param  timeout_ms: 等待超时时间（毫秒）
  * @retval 实际接收到的字节数，0 表示超时或未收到数据
  */
int16_t ESP8266_Receive(uint8_t *buf, uint16_t len, uint16_t timeout_ms)
{
    uint8_t ch;
    uint16_t rx_idx = 0;
    uint16_t first_num = 0;
    uint16_t pkt_len = 0;
    uint8_t state = 0;

    while (timeout_ms--)
    {
        while (USART3_GetByte(&ch))
        {
            switch (state)
            {
                case 0:   /* 等待 '+' */
                    if (ch == '+') state = 1;
                    break;
                case 1:   /* '+' 后接 'I' */
                    if (ch == 'I') state = 2;
                    else if (ch == '+') state = 1;
                    else state = 0;
                    break;
                case 2:   /* 'IP' */
                    if (ch == 'P') state = 3;
                    else state = 0;
                    break;
                case 3:   /* 'IPD' */
                    if (ch == 'D') state = 4;
                    else state = 0;
                    break;
                case 4:   /* 'IPD,' 后读取长度 */
                    if (ch == ':')
                    {
                        pkt_len = first_num;
                        state = 5;
                    }
                    else if (ch >= '0' && ch <= '9')
                    {
                        first_num = first_num * 10 + (ch - '0');
                    }
                    else
                    {
                        state = 0;
                    }
                    break;
                case 5:   /* 读取实际数据 */
                    if (rx_idx < len)
                    {
                        buf[rx_idx++] = ch;
                    }
                    pkt_len--;
                    if (pkt_len == 0 || rx_idx >= len)
                    {
                        return rx_idx;
                    }
                    break;
                default:
                    state = 0;
                    break;
            }
        }

        IWDG_Feed();
        delay_ms(1);
    }

    return rx_idx;
}

/**
  * @brief  使用 AT+PING 检测 ESP8266 是否已连通外网
  * @retval 1=能 ping 通公网地址，0=不能
  */
uint8_t ESP8266_CheckInternet(void)
{
    char cmd[64];
    uint8_t ok;
    sprintf(cmd, "AT+PING=\"%s\"\r\n", ESP8266_PING_TARGET);
    U1_PRINTF("[NET] PING %s...\r\n", ESP8266_PING_TARGET);
    ok = ESP8266_SendAT(cmd, "OK", 5000);
    U1_PRINTF("[NET] %s\r\n", ok ? "Internet OK" : "No Internet");
    return ok;
}

/**
  * @brief  TCP 连接（单连接模式）
  */
uint8_t ESP8266_ConnectTCP(char *ip, uint16_t port)
{
    char cmd[128];
    uint8_t ok;
    U1_PRINTF("[...] ESP8266 AT+CIPMUX=0\r\n");
    ESP8266_SendAT("AT+CIPMUX=0\r\n", "OK", 2000);
    delay_ms(200);
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip, port);
    U1_PRINTF("[...] ESP8266 %s", cmd);
    ok = ESP8266_SendAT(cmd, "OK", 10000);
    U1_PRINTF("  -> CIPSTART TCP %s\r\n", ok ? "OK" : "FAIL");
    return ok;
}

/**
  * @brief  发送 TCP 数据
  */
uint8_t ESP8266_SendTCP(uint8_t *data, uint16_t len)
{
    char cmd[32];
    uint16_t i;
    uint8_t ch;

    sprintf(cmd, "AT+CIPSEND=%d\r\n", len);
    while (USART3_GetByte(&ch)) { }   /* 清掉连接状态残留 */

    if (!ESP8266_SendAT(cmd, ">", 3000))
    {
        U1_PRINTF("[ESP8266] no > prompt\r\n");
        return 0;
    }

    delay_ms(100);   /* 等 > 提示稳定 */

    for (i = 0; i < len; i++) USART3_SendByte(data[i]);
    U1_PRINTF("[ESP8266] sent %d bytes\r\n", len);

    /* 给 ESP8266 时间把数据发出去，并等 SEND OK 一闪而过 */
    delay_ms(800);
    return 1;
}

/**
  * @brief  在 HTTP 响应里搜索 JSON 字段值
  */
static uint16_t HTTP_FindField(const uint8_t *buf, uint16_t buf_len,
                               const char *field, char *out, uint16_t max_len)
{
    uint16_t flen = strlen(field);
    uint16_t i, j;
    uint16_t out_i = 0;

    for (i = 0; i + flen < buf_len; i++)
    {
        if (memcmp(buf + i, field, flen) != 0) continue;

        j = i + flen;

        /* 支持 JSON 中 "datetime": 和 datetime: 两种写法 */
        if (buf[j] == '"') {
            if (j + 1 >= buf_len || buf[j + 1] != ':') continue;
            j += 2;
        } else if (buf[j] == ':') {
            j++;
        } else {
            continue;
        }

        while (j < buf_len && (buf[j] == ' ' || buf[j] == '"')) j++;
        out_i = 0;
        while (j < buf_len && out_i < max_len - 1 &&
               buf[j] != '"' && buf[j] != ',' && buf[j] != '}')
        {
            out[out_i++] = buf[j];
            j++;
        }
        out[out_i] = '\0';
        return out_i;
    }
    return 0;
}

#if 0  /* HTTP 时间同步已被 NTP 取代，保留代码供参考 */
/**
  * @brief  通过 HTTP API 获取网络时间（UTC+8）
  * @retval 1=成功，0=失败
  */
uint8_t ESP8266_GetHTTPTime(RTC_DateTime_t *dt)
{
    uint8_t http_req[256];
    uint8_t http_resp[600];
    int16_t resp_len;
    char datetime[32];
    int y, mo, d, h, mi, s;
    uint16_t req_len;
    uint16_t dbg_len;
    uint32_t dbg_to;
    uint8_t success = 0;
    uint8_t retry;
    uint8_t ch;

    if (!dt) return 0;
    memset(dt, 0, sizeof(RTC_DateTime_t));

    U1_PRINTF("[HTTP] Getting time...\r\n");

    /* 1. 彻底关闭所有连接；无连接时 ESP 返回 ERROR 也接受 */
    ESP8266_SendAT("AT+CIPCLOSE\r\n", NULL, 0);
    delay_ms(400);
    ESP8266_SendAT("AT+CIPCLOSE\r\n", NULL, 0);
    delay_ms(400);
    while (USART3_GetByte(&ch)) { }

    /* 2. 确保 CIPMUX=0 单连接模式，重试 3 次 */
    for (retry = 0; retry < 3; retry++) {
        if (ESP8266_SendAT("AT+CIPMUX=0\r\n", "OK", 2000)) {
            U1_PRINTF("[HTTP] CIPMUX=0 OK (retry=%d)\r\n", retry);
            break;
        }
        U1_PRINTF("[HTTP] CIPMUX=0 retry %d/3\r\n", retry + 1);
        delay_ms(500);
        while (USART3_GetByte(&ch)) { }
    }
    if (retry >= 3) {
        U1_PRINTF("[HTTP] CIPMUX=0 failed after retries\r\n");
        goto restore_udp;
    }
    delay_ms(200);

    /* 3. 用 IP 直接连 TCP 80，绕过 ESP8266 不稳定的内置 DNS */
    U1_PRINTF("[HTTP] connecting TCP %s:80 ...\r\n", ESP8266_HTTP_TIME_IP);
    {
        char cmd[128];
        sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",80\r\n", ESP8266_HTTP_TIME_IP);
        if (!ESP8266_SendAT(cmd, "OK", 15000))
        {
            U1_PRINTF("[HTTP] TCP connect FAIL (timeout or busy)\r\n");
            goto restore_udp;
        }
    }
    U1_PRINTF("[HTTP] TCP connected\r\n");
    delay_ms(1000);

    /* 4. 构造 HTTP GET，Host 头用域名 */
    req_len = snprintf((char *)http_req, sizeof(http_req),
        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Connection: close\r\n"
        "\r\n",
        ESP8266_HTTP_TIME_PATH, ESP8266_HTTP_TIME_HOST);
    U1_PRINTF("[HTTP] sending GET %s (%d bytes)\r\n", ESP8266_HTTP_TIME_PATH, req_len);
    if (!ESP8266_SendTCP(http_req, req_len))
    {
        U1_PRINTF("[HTTP] send failed\r\n");
        goto restore_udp;
    }

    /* 5. 接收响应 */
    U1_PRINTF("[HTTP] waiting response...\r\n");
    resp_len = ESP8266_Receive(http_resp, sizeof(http_resp), 10000);
    U1_PRINTF("[HTTP] got %d bytes\r\n", resp_len);
    if (resp_len < 80)
    {
        dbg_len = 0;
        dbg_to = 3000;

        U1_PRINTF("[HTTP] response too short, dump raw 3s...\r\n");
        while (dbg_to-- && dbg_len < sizeof(http_resp) - 1)
        {
            if (USART3_GetByte(&http_resp[dbg_len]))
            {
                dbg_len++;
            }
            else
            {
                delay_ms(1);
            }
        }
        http_resp[dbg_len] = 0;
        U1_PRINTF("[HTTP] raw %d bytes: %.300s\r\n", dbg_len, http_resp);
        goto restore_udp;
    }

    /* 6. 搜索 currentDateTime 字段（自动兼容 JSON 引号） */
    if (HTTP_FindField(http_resp, resp_len, "currentDateTime", datetime, sizeof(datetime)) == 0)
    {
        U1_PRINTF("[HTTP] currentDateTime field not found\r\n");
        http_resp[resp_len > 200 ? 200 : resp_len] = 0;
        U1_PRINTF("[HTTP] head: %.200s\r\n", http_resp);
        goto restore_udp;
    }

    U1_PRINTF("[HTTP] currentDateTime=%s\r\n", datetime);

    /* 7. 解析 ISO 8601 UTC 时间，转 UTC+8
       worldclockapi 格式示例：2026-09-09T01:54Z（无秒）
       也兼容带秒格式：2026-09-09T01:54:30Z */
    s = 0;
    if (sscanf(datetime, "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &s) < 5)
    {
        U1_PRINTF("[HTTP] parse failed\r\n");
        goto restore_udp;
    }
    h += ESP8266_TIMEZONE_HOUR;
    if (h >= 24) { h -= 24; d++; }

    dt->Year   = (uint8_t)(y - 2000);
    dt->Month  = (uint8_t)mo;
    dt->Date   = (uint8_t)d;
    dt->Hours  = (uint8_t)h;
    dt->Minutes= (uint8_t)mi;
    dt->Seconds= (uint8_t)s;
    dt->WeekDay= 1;
    success = 1;

    U1_PRINTF("[HTTP] OK: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
              dt->Year, dt->Month, dt->Date,
              dt->Hours, dt->Minutes, dt->Seconds);

restore_udp:
    while (USART3_GetByte(&ch)) { }
    ESP8266_SendAT("AT+CIPCLOSE\r\n", NULL, 0);
    delay_ms(300);
    ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT);
    return success;
}
#endif

/**
  * @brief  Unix 时间戳转日期时间（纯整数运算，不依赖 localtime）
  * @param  ts: Unix 时间戳（秒，自 1970-01-01 00:00:00 UTC）
  * @param  dt: 输出 RTC_DateTime_t，Year 存 0-99（相对 2000 年）
  */
static void unix_to_datetime(uint32_t ts, RTC_DateTime_t *dt)
{
    uint32_t sec, min, hour, day;
    uint32_t y, m, d;
    static const uint8_t days_in_month[] = {31,28,31,30,31,30,31,31,30,31,30,31};

    sec  = ts % 60;  ts /= 60;
    min  = ts % 60;  ts /= 60;
    hour = ts % 24;
    day  = ts / 24;

    /* 从 1970-01-01 起算天数，逐年逐月扣除 */
    y = 1970;
    while (1) {
        uint16_t yday = (((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0)) ? 366 : 365;
        if (day < yday) break;
        day -= yday;
        y++;
    }

    m = 1;
    while (m <= 12) {
        uint8_t md = days_in_month[m - 1];
        if (m == 2 && (((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0)))
            md = 29;
        if (day < md) break;
        day -= md;
        m++;
    }
    d = day + 1;

    dt->Year    = (uint8_t)(y >= 2000 ? y - 2000 : 0);
    dt->Month   = (uint8_t)m;
    dt->Date    = (uint8_t)d;
    dt->Hours   = (uint8_t)hour;
    dt->Minutes = (uint8_t)min;
    dt->Seconds = (uint8_t)sec;
    dt->WeekDay = 1;
}

/**
  * @brief  通过 NTP/UDP 获取网络时间（UTC+8 北京时间）
  * @param  dt: 输出日期时间结构体
  * @retval 1=成功，0=失败
  */
uint8_t ESP8266_GetNTPTime(RTC_DateTime_t *dt)
{
    uint8_t ntp_req[48];
    uint32_t ntp_ts, unix_ts;
    uint8_t retry;
    uint8_t ch;
    uint8_t close_ok;
    char cmd[128];

    if (!dt) return 0;
    memset(dt, 0, sizeof(RTC_DateTime_t));

    U1_PRINTF("[NTP] Getting time from %s:%d...\r\n",
              ESP8266_NTP_SERVER_IP, ESP8266_NTP_SERVER_PORT);

    /* 1. 关闭当前连接，循环检测直到 OK 或 ERROR */
    for (retry = 0; retry < 3; retry++) {
        close_ok = ESP8266_SendAT("AT+CIPCLOSE\r\n", "OK", 2000);
        if (!close_ok) {
            close_ok = ESP8266_SendAT("AT+CIPCLOSE\r\n", "ERROR", 2000);
        }
        if (close_ok) break;
        delay_ms(300);
    }
    delay_ms(200);

    /* 2. 清空 USART3 残留数据 */
    while (USART3_GetByte(&ch)) { }

    /* 3. 确保 CIPMUX=0 单连接模式 */
    for (retry = 0; retry < 3; retry++) {
        if (ESP8266_SendAT("AT+CIPMUX=0\r\n", "OK", 2000)) {
            U1_PRINTF("[NTP] CIPMUX=0 OK\r\n");
            break;
        }
        delay_ms(500);
        while (USART3_GetByte(&ch)) { }
    }
    if (retry >= 3) {
        U1_PRINTF("[NTP] CIPMUX=0 failed\r\n");
        goto restore_udp;
    }
    delay_ms(200);

    /* 4. 建立 UDP 连接到 NTP 服务器 */
    sprintf(cmd, "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n",
            ESP8266_NTP_SERVER_IP, ESP8266_NTP_SERVER_PORT);
    if (!ESP8266_SendAT(cmd, "OK", 10000)) {
        U1_PRINTF("[NTP] UDP connect FAIL\r\n");
        goto restore_udp;
    }
    U1_PRINTF("[NTP] UDP connected to NTP server\r\n");
    delay_ms(500);

    /* 5. 构造 48 字节 NTP 请求包 */
    memset(ntp_req, 0, sizeof(ntp_req));
    ntp_req[0] = 0x23;   /* LI=0, VN=4, Mode=3 (client) */

    /* 6. 发送 NTP 请求 */
    if (!ESP8266_SendUDP(ntp_req, 48)) {
        U1_PRINTF("[NTP] send failed\r\n");
        goto restore_udp;
    }
    U1_PRINTF("[NTP] request sent, waiting response...\r\n");

    /* 7. 接收 NTP 响应（原始读取，绕过 +IPD 解析器）
     *    阿里云 NTP 服务器实际发送 56 字节（含 8 字节扩展零填充），
     *    但 ESP8266 +IPD 头报告 48 字节，导致 ESP8266_Receive() 截断。
     *    改用原始 USART3 读取，捕获全部数据后在缓冲区中解析。 */
    {
        uint8_t raw[160];
        uint16_t raw_len = 0;
        uint16_t ntp_offset = 0;
        uint16_t i;
        int16_t raw_timeout = 10000;

        U1_PRINTF("[NTP] waiting raw response...\r\n");

        while (raw_timeout--) {
            while (USART3_GetByte(&ch) && raw_len < sizeof(raw)) {
                raw[raw_len++] = ch;
            }
            if (raw_len >= 80) {
                delay_ms(50);
                while (USART3_GetByte(&ch) && raw_len < sizeof(raw))
                    raw[raw_len++] = ch;
                break;
            }
            IWDG_Feed();
            delay_ms(1);
        }

        U1_PRINTF("[NTP] raw %u bytes\r\n", raw_len);

        /* 调试：打印原始数据十六进制 */
#ifdef DEBUG_NTP
        {
            uint16_t di;
            U1_PRINTF("[NTP] HEX: ");
            for (di = 0; di < raw_len; di++) {
                U1_PRINTF("%02X ", raw[di]);
            }
            U1_PRINTF("\r\n");
        }
#endif

        if (raw_len < 48) {
            U1_PRINTF("[NTP] response too short\r\n");
            goto restore_udp;
        }

        /* 定位 +IPD 后的 NTP 载荷起始 */
        for (i = 0; i + 4 < raw_len; i++) {
            if (raw[i] == '+' && raw[i+1] == 'I' &&
                raw[i+2] == 'P' && raw[i+3] == 'D') {
                i += 4;
                while (i < raw_len && raw[i] != ':') i++;
                if (i < raw_len) ntp_offset = i + 1;
                break;
            }
        }

        if (ntp_offset == 0 || raw_len - ntp_offset < 48) {
            U1_PRINTF("[NTP] +IPD not found or payload too short\r\n");
            goto restore_udp;
        }

        U1_PRINTF("[NTP] payload offset=%u avail=%u\r\n",
                  ntp_offset, raw_len - ntp_offset);

        /* 8. 提取 Transmit Timestamp（标准 NTP 偏移 40-43，大端序） */
        ntp_ts = ((uint32_t)raw[ntp_offset + 40] << 24) |
                 ((uint32_t)raw[ntp_offset + 41] << 16) |
                 ((uint32_t)raw[ntp_offset + 42] <<  8) |
                 ((uint32_t)raw[ntp_offset + 43]);

        U1_PRINTF("[NTP] tx ts[40]: %02X %02X %02X %02X\r\n",
                  raw[ntp_offset + 40], raw[ntp_offset + 41],
                  raw[ntp_offset + 42], raw[ntp_offset + 43]);

    }

    if (ntp_ts == 0) {
        U1_PRINTF("[NTP] timestamp is zero\r\n");
        goto restore_udp;
    }

    /* 9. NTP 时间戳 → Unix 时间戳（减去 70 年偏移） */
    unix_ts = ntp_ts - 2208988800UL;

    /* 10. UTC → 北京时间（UTC+8） */
    unix_ts += (uint32_t)ESP8266_TIMEZONE_HOUR * 3600;

    U1_PRINTF("[NTP] ntp_ts=%lu unix_ts=%lu\r\n", ntp_ts, unix_ts);

    /* 11. 转换为日期时间 */
    unix_to_datetime(unix_ts, dt);

    U1_PRINTF("[NTP] OK: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
              dt->Year, dt->Month, dt->Date,
              dt->Hours, dt->Minutes, dt->Seconds);

restore_udp:
    /* 关闭 NTP UDP 连接 */
    while (USART3_GetByte(&ch)) { }
    ESP8266_SendAT("AT+CIPCLOSE\r\n", NULL, 0);
    delay_ms(300);

    /* 恢复 UDP 连接到电脑 */
    U1_PRINTF("[NTP] Restoring UDP to PC...\r\n");
    if (!ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT)) {
        U1_PRINTF("[NTP] UDP restore FAIL, retrying...\r\n");
        delay_ms(500);
        ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT);
    }

    return (dt->Year > 0 || dt->Month > 0) ? 1 : 0;
}

/**
  * @brief  发送告警信息（JSON 格式）+ 5 秒冷却防刷屏
  */
static uint32_t s_last_send_tick = 0;   /* 上次发送的系统节拍 */
#define SEND_COOLDOWN_MS  5000          /* 5 秒内不重复发 */

void ESP8266_SendAlert(const char *msg)
{
    /* 冷却检查：用 g_tick（delay.h 里的 1ms 全局节拍） */
    uint32_t now = g_tick;
    if ((now - s_last_send_tick) < SEND_COOLDOWN_MS)
    {
        return;   /* 冷却中，不重复发 */
    }
    s_last_send_tick = now;

    char json_buf[256];
    int len;

    len = snprintf(json_buf, sizeof(json_buf),
                   "{\"type\":\"alert\",\"msg\":\"%s\"}", msg);
    if (len > 0 && len < (int)sizeof(json_buf))
    {
        if (ESP8266_SendUDP((uint8_t *)json_buf, (uint16_t)len))
        {
            U1_PRINTF("[ESP8266] UDP sent: %s\r\n", json_buf);
        }
        else
        {
            U1_PRINTF("[ESP8266] UDP send FAIL: %s\r\n", json_buf);
        }
    }
}

/**
  * @brief  发送数据（JSON 格式）
  * @param  topic:   数据主题/名称
  * @param  payload: 数据内容字符串
  */
void ESP8266_SendData(const char *topic, const char *payload)
{
    char json_buf[256];
    int len;

    len = snprintf(json_buf, sizeof(json_buf),
                   "{\"type\":\"data\",\"topic\":\"%s\",\"payload\":\"%s\"}",
                   topic, payload);
    if (len > 0 && len < (int)sizeof(json_buf))
    {
        ESP8266_SendUDP((uint8_t *)json_buf, (uint16_t)len);
    }
}

/**
  * @brief  ESP8266 健康检查与自动恢复
  * @note   主循环每 5 分钟调用一次。
  *         1) AT 无响应 → 复位 + 完整重连（WiFi + UDP）
  *         2) AT 有响应但连续发送失败 → 仅重建 UDP 连接
  *         3) 一切正常 → 不做任何操作
  */
uint8_t ESP8266_CheckHealth(void)
{
    /* 1. 轻量探测：发 AT，1s 内无响应认为模块挂死 */
    if (!ESP8266_SendAT("AT\r\n", "OK", 1000))
    {
        U1_PRINTF("[ESP8266] no response, resetting...\r\n");
        ESP8266_Reset();
        if (!ESP8266_Init())
        {
            U1_PRINTF("[ESP8266] re-init FAIL\r\n");
            return 0;
        }
        U1_PRINTF("[ESP8266] re-init OK\r\n");
        s_send_fail_count = 0;
        return 1;
    }

    /* 2. 模块活着但连续发送失败 → 重建 UDP 连接 */
    if (s_send_fail_count >= 3)
    {
        U1_PRINTF("[ESP8266] %d send fails, reconnecting UDP...\r\n", s_send_fail_count);
        ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT);
        s_send_fail_count = 0;
    }

    return 1;
}
