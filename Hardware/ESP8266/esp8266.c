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

#define U1_PRINTF  USART1_Printf   /* easy to switch to USART2/other UART for debugging */

/* 抑制 #236-D: controlling expression is constant
   原因：MQTT_OnCmdReceived 函数指针是链接期常量，编译器保守判断 */
#pragma diag_suppress 236

static uint8_t  s_send_fail_count = 0;  /* 连续发送失败计数，用于健康检查 */
static volatile uint8_t s_wifi_ok = 0;  /* WiFi 连接状态，供 OLED 状态栏读取 */
static uint8_t  s_mqtt_ok = 0;          /* MQTT 握手完成标志（CONNECT+SUBSCRIBE 都成功） */

/* 下行 +IPD 帧解析器的字节入口，本体在文件末尾的 MQTT 区。
   提前声明是因为 ESP8266_SendTCP() 发送前也要调它，把已到达的下行帧
   喂进解析器而不是直接丢弃。 */
static void MQTT_FeedByte(uint8_t ch);

/* AT 等待间隙回调（P3-#4 折中方案）：broker 宕机时 publish 路径的完整重连
   最坏阻塞 ~15s（CIPSTART 10s 主导），协作式调度器整体冻结，防拆 5Hz
   采样会停。在等待循环里每 200ms 调一次本钩子，main 注册它续采防拆，
   冻结期间的晃动检测不丢。狗本来就喂（SendAT 每 1ms IWDG_Feed）。 */
static void (*s_wait_hook)(void) = 0;
static uint32_t s_hook_last = 0;

void ESP8266_SetWaitHook(void (*cb)(void))
{
    s_wait_hook = cb;
}

static void WaitHook_Tick(void)
{
    if (s_wait_hook != 0 && (g_tick - s_hook_last) >= 200)
    {
        s_hook_last = g_tick;
        s_wait_hook();
    }
}

/**
  * @brief  初始化 ESP8266 通信接口并连接 WiFi
  * @note   先 115200 波特率探测，失败则降级 9600；再测 AT、连 WiFi、可选连 UDP
  * @retval 1 = WiFi 连接成功，0 = 失败
  */
uint8_t ESP8266_Init(void)
{
    uint32_t baud;
    uint8_t  at_ok = 0;

    U1_PRINTF("[ESP] --- ESP8266_Init start ---\r\n");

    /* 1. 波特率探测：先试 115200，不行再降级 9600 */
    for (baud = 115200; baud >= 9600; baud = (baud == 115200) ? 9600 : 0)
    {
        if (baud == 0) break;
        U1_PRINTF("[ESP] trying USART3 @ %lu baud...\r\n", baud);
        USART3_Init(baud);
        delay_ms(200);   /* 等 USART 稳定 */
        USART3_FlushRx();

        if (ESP8266_SendAT("AT\r\n", "OK", 3000))
        {
            at_ok = 1;
            U1_PRINTF("[ESP] AT OK at %lu baud\r\n", baud);
            break;
        }
        U1_PRINTF("[ESP] AT no response at %lu baud, trying next...\r\n", baud);
    }

    if (!at_ok)
    {
        /* 模块上电启动可能比 STM32 慢（ESP8266 自身 boot 需 ~1-2s）：
           等 2s 补一轮探测再宣判，避免开机即进入离线模式（实测复现）。
           必须分段喂狗：本函数在运行期 CheckHealth→re-init 路径也可达，
           整段 delay_ms(2000) 恰好等于 IWDG 窗口，必咬（同 Reset 的教训）。 */
        U1_PRINTF("[ESP] no response, module may still be booting, retry in 2s...\r\n");
        for (uint8_t i = 0; i < 20; i++)
        {
            delay_ms(100);
            IWDG_Feed();
        }
        for (baud = 115200; baud >= 9600; baud = (baud == 115200) ? 9600 : 0)
        {
            if (baud == 0) break;
            USART3_Init(baud);
            delay_ms(200);
            USART3_FlushRx();
            if (ESP8266_SendAT("AT\r\n", "OK", 3000))
            {
                at_ok = 1;
                U1_PRINTF("[ESP] AT OK at %lu baud (retry)\r\n", baud);
                break;
            }
        }
    }

    if (!at_ok)
    {
        U1_PRINTF("[ESP] AT no response at 115200 and 9600, hardware issue!\r\n");
        U1_PRINTF("[ESP] check: 3.3V power? TX->RXD? RX->TXD? GND connected?\r\n");
        s_wifi_ok = 0;
        return 0;
    }

    /* 2. 软件复位一次，确保干净状态，再验一次 AT */
    U1_PRINTF("[ESP] resetting module...\r\n");
    ESP8266_Reset();
    delay_ms(500);

    if (!ESP8266_SendAT("AT\r\n", "OK", 3000))
    {
        U1_PRINTF("[ESP] no AT after reset, giving up\r\n");
        s_wifi_ok = 0;
        return 0;
    }

    /* 3. 连接 WiFi */
    U1_PRINTF("[ESP] connecting WiFi SSID=\"%s\"...\r\n", WIFI_SSID);
    if (!ESP8266_ConnectAP(WIFI_SSID, WIFI_PASSWORD))
    {
        U1_PRINTF("[ESP] WiFi connect FAIL\r\n");
        return 0;
    }
    U1_PRINTF("[ESP] WiFi connected!\r\n");

#if USE_MQTT
    if (!ESP8266_MQTTConnect())
        U1_PRINTF("[MQTT] Init FAIL, CheckHealth will retry\r\n");
    else if (!ESP8266_MQTTSubscribe())
        U1_PRINTF("[MQTT] Subscribe FAIL\r\n");
    else {
        s_mqtt_ok = 1;   /* CONNECT + SUBSCRIBE 都成功，MQTT 会话就绪 */
        U1_PRINTF("[MQTT] Init OK (session ready)\r\n");
        MQTT_ReportLightStatus();    /* 首次会话就绪后主动上报灯状态（broker retain） */
        MQTT_ReportModeStatus();   /* 首次会话就绪后主动上报模式状态（broker retain） */
    }
#else
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
#endif

    return 1;   /* WiFi 已连上，UDP 失败不阻塞主流程 */
}

/**
  * @brief  发送 AT 指令并等待指定应答（带完整调试日志）
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
    uint16_t start_remain = timeout_ms;

    USART3_FlushRx();
    memset(rx_buf, 0, sizeof(rx_buf));

    if (cmd != NULL)
    {
        U1_PRINTF("[ESP] >>> %s", cmd);
        USART3_SendString(cmd);
    }

    if (ack == NULL)
    {
        U1_PRINTF("[ESP] (no ack expected, fire-and-forget)\r\n");
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
            U1_PRINTF("[ESP] <<< OK (found \"%s\", raw=%.128s)\r\n", ack, rx_buf);
            return 1;
        }

        WaitHook_Tick();
        IWDG_Feed();
        delay_ms(1);
    }

    U1_PRINTF("[ESP] <<< TIMEOUT %ums, looking for \"%s\", got (%u bytes): %.128s\r\n",
              start_remain, ack, rx_idx, rx_buf);
    return 0;
}

/**
  * @brief  软件复位 ESP8266
  */
void ESP8266_Reset(void)
{
    uint8_t i;

    ESP8266_SendAT("AT+RST\r\n", "ready", 3000);

    /* 复位后需要等待启动完成。必须分段喂狗：IWDG 超时正好是 2000ms，
       整段 delay_ms(2000) 零余量零喂狗。冷启动时无害（IWDG_Init 在
       基线学习之后才执行），但 IWDG 一旦启动只能靠掉电关闭，热复位
       （RESET 键 / 重新烧录不断电）后它仍在跑，这 2 秒必然被咬死，
       于是每次热复位都变成开机死循环（实测：+0.41/+2.40/+4.38s 三条 banner）。 */
    for (i = 0; i < 20; i++)
    {
        delay_ms(100);
        IWDG_Feed();
    }
}

/**
  * @brief  设置 ESP8266 工作模式（带固件版本 fallback）
  * @param  mode: 1=STA  Station 模式
  *              2=AP   Access Point 模式
  *              3=AP+STA 混合模式
  * @note   新版固件用 CWMODE_CUR，老固件只认 CWMODE（无后缀）
  */
uint8_t ESP8266_SetMode(uint8_t mode)
{
    char cmd[32];
    uint8_t ok;

    snprintf(cmd, sizeof(cmd), "AT+CWMODE_CUR=%d\r\n", mode);
    ok = ESP8266_SendAT(cmd, "OK", 1000);
    if (ok) return 1;

    U1_PRINTF("[ESP] CWMODE_CUR failed, trying CWMODE fallback...\r\n");
    snprintf(cmd, sizeof(cmd), "AT+CWMODE=%d\r\n", mode);
    ok = ESP8266_SendAT(cmd, "OK", 1000);
    if (ok) return 1;

    U1_PRINTF("[ESP] both CWMODE_CUR and CWMODE failed, module in wrong mode\r\n");
    return 0;
}

/**
  * @brief  STA 模式连接指定 WiFi（带固件版本 fallback）
  * @note   新版固件用 CWJAP_CUR，老固件只认 CWJAP（无后缀）
  */
uint8_t ESP8266_ConnectAP(char *ssid, char *password)
{
    char cmd[128];
    uint8_t ok;

    /* 1. 设置 STA 模式（带 fallback） */
    U1_PRINTF("[ESP] setting STA mode...\r\n");
    if (!ESP8266_SetMode(1))
    {
        U1_PRINTF("[ESP] SetMode FAIL, cannot continue\r\n");
        s_wifi_ok = 0;
        return 0;
    }

    /* 2. 先断开上次连接（防 WIFI DISCONNECT 残留） */
    ESP8266_SendAT("AT+CWQAP\r\n", "OK", 1000);
    delay_ms(200);

    /* 3. 先试 CWJAP_CUR（新固件），失败 fallback CWJAP（老固件） */
    snprintf(cmd, sizeof(cmd), "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", ssid, password);
    U1_PRINTF("[ESP] trying CWJAP_CUR...\r\n");
    ok = ESP8266_SendAT(cmd, "OK", 15000);
    if (!ok)
    {
        U1_PRINTF("[ESP] CWJAP_CUR failed, trying CWJAP fallback...\r\n");
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);
        ok = ESP8266_SendAT(cmd, "OK", 15000);
    }

    if (ok)
    {
        U1_PRINTF("[ESP] WiFi connected OK!\r\n");
        s_wifi_ok = 1;
        return 1;
    }

    U1_PRINTF("[ESP] WiFi connect FAIL (SSID=\"%s\")\r\n", ssid);
    U1_PRINTF("[ESP] check: SSID/password correct? hotspot on?\r\n");
    s_wifi_ok = 0;
    return 0;
}

/**
  * @brief  查询 WiFi 是否在线（供 OLED 状态栏等只读使用）
  * @retval 1=在线，0=掉线
  */
uint8_t ESP8266_IsWiFiOk(void)
{
    return s_wifi_ok;
}

/**
  * @brief  查询 MQTT 会话是否就绪（供 OLED 状态行等只读使用）
  * @retval 1=会话就绪（CONNECT+SUBSCRIBE 成功且未断），0=未就绪
  */
uint8_t ESP8266_IsMQTTOk(void)
{
    return s_mqtt_ok;
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

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n", ip, port);
    U1_PRINTF("[...] ESP8266 %s", cmd);
    ok = ESP8266_SendAT(cmd, "OK", 5000);
    U1_PRINTF("  -> CIPSTART UDP %s\r\n", ok ? "OK" : "FAIL (may be firewall, try anyway)");

    /* 返回 CIPSTART 的真实结果（原实现恒返回 1，失败被掩盖）。
       UDP 虽无连接语义、失败后仍可能发出包，但 AT 失败通常意味着
       模块或参数异常，调用方（esp8266.c:107/1250 等）均有 0/1 分支处理 */
    return ok;
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

    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", len);
    if (!ESP8266_SendAT(cmd, ">", 1000))
    {
        s_send_fail_count++;
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        USART3_SendByte(data[i]);
    }

    /* 等 SEND OK：发完字节不等于模块接受/发出（与 SendTCP 对齐，P3-#8）。
       原实现发完直接报成功，模块拒收时告警静默丢失 */
    {
        uint8_t  ch;
        uint8_t  ok_buf[48] = {0};
        uint16_t ok_idx = 0;
        uint16_t ok_timeout = 1500;
        while (ok_timeout--) {
            while (USART3_GetByte(&ch)) {
                if (ok_idx < sizeof(ok_buf) - 1) { ok_buf[ok_idx++] = ch; ok_buf[ok_idx] = '\0'; }
            }
            if (strstr((char *)ok_buf, "SEND OK")) {
                s_send_fail_count = 0;
                return 1;
            }
            if (strstr((char *)ok_buf, "ERROR") ||
                strstr((char *)ok_buf, "SEND FAIL")) break;
            WaitHook_Tick();
            IWDG_Feed();
            delay_ms(1);
        }
        U1_PRINTF("[ESP8266] UDP no SEND OK, got: %s\r\n", ok_buf);
    }

    s_send_fail_count++;
    return 0;
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
    snprintf(cmd, sizeof(cmd), "AT+PING=\"%s\"\r\n", ESP8266_PING_TARGET);
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
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip, port);
    U1_PRINTF("[...] ESP8266 %s", cmd);
    ok = ESP8266_SendAT(cmd, "OK", 10000);
    U1_PRINTF("  -> CIPSTART TCP %s\r\n", ok ? "OK" : "FAIL");
    return ok;
}

/* TCP 失败统一登记：清会话标志（不许假装在线）+ 累计失败计数（供健康检查）。
   此前 SendTCP 各失败路径静默 return 0，s_mqtt_ok 恒 1、s_send_fail_count
   只在 MQTT 模式永不被调用的 SendUDP 里递增，导致 TCP 半死永远无恢复。 */
static void TCP_NoteFail(void)
{
    s_mqtt_ok = 0;
    if (s_send_fail_count < 250) s_send_fail_count++;
}

/**
  * @brief  发送 TCP 数据（带 link 自动重连）
  */
uint8_t ESP8266_SendTCP(uint8_t *data, uint16_t len)
{
    char cmd[32];
    uint16_t i;
    uint8_t ch;
    uint8_t need_reconnect = 0;

    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", len);
    /* 发送前清空环形缓冲。不能直接丢：遥测每 10s 发一次，若下行命令正好
       落在这个窗口里就会被吞掉，连日志都不留。改为喂给 +IPD 分帧器，
       FeedByte 只暂存命令、不回调，因此不会与本次发送形成重入。 */
    while (USART3_GetByte(&ch)) MQTT_FeedByte(ch);

    if (!ESP8266_SendAT(cmd, ">", 3000))
    {
        /* 等不到 ">" 即判定链路异常，无条件尝试一次 TCP 重连+重发。
           原实现靠 err_buf 匹配 "link is not valid"/"ERROR" 决定是否重连，
           但 SendAT 在等 ">" 的 3s 窗口内已把模块响应吞进自己的 rx_buf
           （超时返回时随栈帧丢弃），这里再读环形缓冲常见时序下恒为空，
           按内容匹配的断链检测实际不可达，近乎死代码。 */
        uint8_t  err_buf[64] = {0};
        uint16_t err_idx = 0;
        while (USART3_GetByte(&ch) && err_idx < sizeof(err_buf) - 1)
            err_buf[err_idx++] = ch;
        err_buf[err_idx] = 0;

        U1_PRINTF("[ESP8266] no > prompt (got: %.48s), try TCP reconnect...\r\n", err_buf);
        need_reconnect = 1;
    }

    if (need_reconnect)
    {
        s_mqtt_ok = 0;   /* TCP 重建了，MQTT 会话丢了，必须重握手 */
        if (!ESP8266_ConnectTCP(MQTT_SERVER, MQTT_PORT))
        {
            U1_PRINTF("[ESP8266] TCP reconnect FAIL\r\n");
            TCP_NoteFail();
            return 0;
        }
        /* 新 socket 上没有 MQTT CONNECT 会话：裸重发 PUBLISH 报文会被
           broker 当协议违规直接丢弃，函数却返回"成功"——告警静默丢失。
           不装：如实返回失败；s_mqtt_ok 已清，下一次发布会走
           EnsureSession 完成 CONNECT+SUBSCRIBE 重握手后再发（P3-#8） */
        U1_PRINTF("[ESP8266] TCP reconnected, MQTT session lost, msg dropped honestly\r\n");
        return 0;
    }

    delay_ms(100);   /* 等 > 提示稳定 */

    for (i = 0; i < len; i++) USART3_SendByte(data[i]);
    U1_PRINTF("[ESP8266] sent %d bytes\r\n", len);

    /* Read SEND OK from ESP8266, else next AT+CIPSEND will be rejected */
    {
        uint8_t  ok_buf[64] = {0};
        uint16_t ok_idx = 0;
        uint16_t ok_timeout = 2000;
        while (ok_timeout--) {
            while (USART3_GetByte(&ch)) {
                /* 这个等待窗口有 50~200ms，期间到达的下行 +IPD 帧若只进 ok_buf
                   就会随 return 一起丢掉。字节一到就喂解析器，覆盖三种情况：
                   SEND OK 之前到达的帧、超出 ok_buf 63 字节上限被截断的帧、
                   以及超时 return 0 路径上的帧。不可在 return 前再回灌 ok_buf，
                   否则同一批字节会被喂两次，破坏 +IPD 状态机的同步。 */
                MQTT_FeedByte(ch);
                if (ok_idx < sizeof(ok_buf) - 1) {
                    ok_buf[ok_idx++] = ch;
                    ok_buf[ok_idx]   = '\0';
                }
            }
            if (strstr((char *)ok_buf, "SEND OK")) {
                s_send_fail_count = 0;   /* 发送成功，清连续失败计数 */
                return 1;
            }
            if (strstr((char *)ok_buf, "ERROR") ||
                strstr((char *)ok_buf, "SEND FAIL")) {
                U1_PRINTF("[ESP8266] CIPSEND rejected: %s\r\n", ok_buf);
                TCP_NoteFail();
                return 0;
            }
            WaitHook_Tick();
            IWDG_Feed();
            delay_ms(1);
        }
        U1_PRINTF("[ESP8266] no SEND OK, got: %s\r\n", ok_buf);
        TCP_NoteFail();
        return 0;
    }
}

/* =====================================================================
 * MQTT 3.1.1 精简实现（替代原巴法云 TCP 协议）
 * 底层 TCP 收发直接复用 ESP8266_ConnectTCP / ESP8266_SendTCP
 * ===================================================================== */

/* 写 MQTT UTF-8 字符串（2 字节大端长度前缀 + payload） */
static uint16_t MQTT_WriteStr(uint8_t *dst, const char *s)
{
    uint16_t len = (uint16_t)strlen(s);
    dst[0] = (len >> 8) & 0xFF;
    dst[1] =  len       & 0xFF;
    memcpy(dst + 2, s, len);
    return len + 2;
}

/**
  * @brief  连接 MQTT Broker：TCP 连接 + MQTT CONNECT
  * @retval 1=OK, 0=FAIL
  */
uint8_t ESP8266_MQTTConnect(void)
{
    uint8_t pkt[128];
    uint16_t pos;

    U1_PRINTF("[MQTT] Closing old socket...\r\n");
    ESP8266_SendAT("AT+CIPCLOSE\r\n", "OK", 2000);
    delay_ms(300);

    U1_PRINTF("[MQTT] Connecting %s:%d\r\n", MQTT_SERVER, MQTT_PORT);
    if (!ESP8266_ConnectTCP(MQTT_SERVER, MQTT_PORT)) {
        U1_PRINTF("[MQTT] TCP FAIL\r\n");
        return 0;
    }

    /* --- 构造 MQTT CONNECT 报文 ---
       固定头: 0x10 | 剩余长度 (单字节，<=127)
       可变头: 协议名 "MQTT"(6) + 级别 0x04 + 标志 0x02(Clean) + KeepAlive(2)
       Payload: ClientID
    */
    pos = 2;  /* 预留固定头 2 字节 */
    pos += MQTT_WriteStr(pkt + pos, "MQTT");    /* 协议名 */
    pkt[pos++] = 0x04;                           /* 协议级别 4 = 3.1.1 */
    pkt[pos++] = 0x02;                           /* Connect Flags: CleanSession=1 */
    pkt[pos++] = (MQTT_KEEPALIVE_SEC >> 8) & 0xFF;
    pkt[pos++] =  MQTT_KEEPALIVE_SEC       & 0xFF;
    pos += MQTT_WriteStr(pkt + pos, MQTT_CLIENT_ID);

    pkt[0] = 0x10;                               /* CONNECT */
    pkt[1] = (uint8_t)(pos - 2);                 /* 剩余长度 */

    U1_PRINTF("[MQTT] Sending CONNECT (%d bytes)\r\n", pos);
    if (!ESP8266_SendTCP(pkt, pos)) {
        U1_PRINTF("[MQTT] CONNECT send FAIL\r\n");
        return 0;
    }

    /* 等 CONNACK：固定头 0x20 0x02 XX 0x00(返回码=0)
       ESP8266 的下行数据通常带 +IPD,N: 前缀，且 CIPSEND 的 SEND OK
       读缓冲可能把前缀吞成残缺的 PD,/D,/、: 等，因此不能假设 CONNACK
       从字节 0 开始，必须对整段缓冲扫描 0x20 0x02 XX 0x00 签名。 */
    {
        uint8_t  rx[64] = {0};
        uint16_t rx_idx = 0;
        uint16_t i;
        uint8_t  ch;
        uint16_t t = 2000;
        while (t--) {
            while (USART3_GetByte(&ch)) {
                if (rx_idx < sizeof(rx) - 1) rx[rx_idx++] = ch;
            }
            for (i = 0; i + 3 < rx_idx; i++) {
                if (rx[i] == 0x20 && rx[i+1] == 0x02 && rx[i+3] == 0x00) {
                    U1_PRINTF("[MQTT] CONNACK OK (off=%u)\r\n", i);
                    return 1;
                }
            }
            WaitHook_Tick();
            IWDG_Feed();
            delay_ms(1);
        }
        U1_PRINTF("[MQTT] no CONNACK, raw: %.32s\r\n", rx);
        return 0;
    }
}

/**
  * @brief  订阅控制指令主题 MQTT_TOPIC_CMD
  * @retval 1=OK, 0=FAIL
  */
uint8_t ESP8266_MQTTSubscribe(void)
{
    uint8_t pkt[64];
    uint16_t pos;
    uint16_t pkt_id = 1;

    pos = 2;   /* 固定头 2 字节 */
    pkt[pos++] = (pkt_id >> 8) & 0xFF;
    pkt[pos++] =  pkt_id       & 0xFF;
    pos += MQTT_WriteStr(pkt + pos, MQTT_TOPIC_CMD);
    pkt[pos++] = 0x00;   /* QoS 0 */

    pkt[0] = 0x82;       /* SUBSCRIBE (fixed=0x80 | flags=0x02) */
    pkt[1] = (uint8_t)(pos - 2);

    U1_PRINTF("[MQTT] Subscribing %s (%d bytes)\r\n", MQTT_TOPIC_CMD, pos);
    if (!ESP8266_SendTCP(pkt, pos)) {
        U1_PRINTF("[MQTT] SUBSCRIBE send FAIL\r\n");
        return 0;
    }

    /* 等 SUBACK: 0x90 <剩余长度> <pkt_id高> <pkt_id低> <0x00>
       与 CONNACK 同理：SUBACK 也带 +IPD,N: 前缀且可能被 SEND OK
       缓冲切碎，故对整个缓冲扫描 0x90 ... 0x00 签名。 */
    {
        uint8_t  rx[32] = {0};
        uint16_t rx_idx = 0;
        uint16_t i;
        uint8_t  ch;
        uint16_t t = 2000;
        while (t--) {
            while (USART3_GetByte(&ch)) {
                if (rx_idx < sizeof(rx) - 1) rx[rx_idx++] = ch;
            }
            for (i = 0; i + 4 < rx_idx; i++) {
                if (rx[i] == 0x90 && rx[i+4] == 0x00) {
                    U1_PRINTF("[MQTT] SUBACK OK\r\n");
                    return 1;
                }
            }
            WaitHook_Tick();
            IWDG_Feed();
            delay_ms(1);
        }
        U1_PRINTF("[MQTT] no SUBACK\r\n");
        return 0;
    }
}

/**
  * @brief  确保 MQTT 会话有效（TCP + CONNECT + SUBSCRIBE 都 OK）
  * @retval 1=会话就绪，0=重连失败
  */
static uint8_t MQTT_EnsureSession(void)
{
    /* 重连退避：broker 不可用时遥测(10s)/心跳(30s)每次 publish 都会走到这里。
       无退避时每个周期都做一次完整握手（CIPSTART 10s 超时主导，最坏 ~15s），
       协作式调度器被反复长阻塞，决策/显示/采样全部挨饿。
       限制完整重连最多每 60s 一次，期间快速失败；首次调用必放行。 */
    #define MQTT_RECONNECT_BACKOFF_MS  60000U
    static uint32_t s_last_attempt = 0;

    if (s_mqtt_ok) return 1;   /* 会话还活着，直接用 */

    if (s_last_attempt != 0 &&
        (g_tick - s_last_attempt) < MQTT_RECONNECT_BACKOFF_MS)
        return 0;
    s_last_attempt = g_tick;

    U1_PRINTF("[MQTT] session lost, full reconnect...\r\n");
    if (!ESP8266_MQTTConnect())
    {
        U1_PRINTF("[MQTT] CONNECT FAIL\r\n");
        return 0;
    }
    if (!ESP8266_MQTTSubscribe())
    {
        U1_PRINTF("[MQTT] SUBSCRIBE FAIL\r\n");
        return 0;
    }
    s_mqtt_ok = 1;
    s_send_fail_count = 0;   /* 会话恢复，清连续失败计数 */
    U1_PRINTF("[MQTT] session restored OK\r\n");
    MQTT_ReportLightStatus();    /* 会话就绪后立刻主动上报灯状态（broker retain） */
    MQTT_ReportModeStatus();       /* 会话就绪后立刻主动上报模式状态（broker retain） */
    return 1;
}

/**
  * @brief  向指定主题发布消息（QoS 0，不等待 PUBACK）
  * @param  retain: 1=broker 保留最后一条消息，新客户端订阅时立刻送达；0=不保留
  */
uint8_t ESP8266_MQTTPublish(const char *topic, const char *payload, uint8_t retain)
{
    uint8_t pkt[256];
    uint16_t pos;
    uint16_t tlen, plen;

    if (!topic || !payload) return 0;

    /* 确保 MQTT 会话有效（TCP link 存在 + CONNECT + SUBSCRIBE 已完成） */
    if (!MQTT_EnsureSession())
    {
        return 0;
    }

    tlen = (uint16_t)strlen(topic);
    plen = (uint16_t)strlen(payload);
    if (2 + 2 + tlen + plen > sizeof(pkt) - 2) {
        U1_PRINTF("[MQTT] PUBLISH pkt too big\r\n");
        return 0;
    }

    pos = 2;   /* 固定头 2 字节 */
    pos += MQTT_WriteStr(pkt + pos, topic);
    memcpy(pkt + pos, payload, plen);
    pos += plen;

    pkt[0] = 0x30 | (retain ? 0x01 : 0x00);  /* PUBLISH QoS 0, retain=bit0 */
    pkt[1] = (uint8_t)(pos - 2);

    return ESP8266_SendTCP(pkt, pos);
}

/**
  * @brief  MQTT PINGREQ 心跳（确保会话有效后再发）
  */
uint8_t ESP8266_MQTTPing(void)
{
    static uint8_t ping[] = {0xC0, 0x00};   /* PINGREQ + 剩余长度 0 */

    if (!MQTT_EnsureSession()) return 0;
    return ESP8266_SendTCP(ping, 2);
}

/**
  * @brief  处理 USART3 环形缓冲中的 MQTT 下行包
  *         调用时机：主循环每 tick 调一次（非阻塞，不引入任何延时）
  *
  * 两级状态机：
  *   第一级 IPD_* —— 剥掉 ESP8266 AT 固件的 "+IPD,<len>:" 前缀
  *                   （多连接固件是 "+IPD,<link id>,<len>:"，两种都支持）。
  *                   只有进入载荷态之后，字节才会被当成 MQTT 固定头。
  *   第二级 MQTT_ParseFrame() —— 在完整载荷上按 MQTT 3.1.1 解包。
  *
  * 为什么必须分两级：ASCII '0'-'9' 是 0x30-0x39，':' 是 0x3A、'=' 是 0x3D，
  * 它们 & 0xF0 全都等于 0x30。旧实现直接在裸 UART 流上判 (ch & 0xF0)==0x30，
  * 于是把 "+IPD,2:" 里的 '2' 当成 PUBLISH 固定头、':' 当成剩余长度 58，
  * 真包被当作包体吞掉，topic/payload 全解析成垃圾。
  *
  * 重入说明：MQTT_FeedByte() 只解析、只把命令暂存到 s_pending_cmd，
  * 绝不在这里回调。因为 ESP8266_SendTCP() 发送前也会调 FeedByte 清缓冲，
  * 而 MQTT_OnCmdReceived() 里又要 MQTTPublish 上报状态，若在 FeedByte 内
  * 直接回调就会形成 SendTCP → 回调 → MQTTPublish → SendTCP 的重入。
  */

/* MQTT 固定头高 4 位 */
#define MQTT_PKT_PUBLISH    0x30
#define MQTT_PKT_PINGRESP   0xD0

#define IPD_BUF_SIZE        128   /* 单帧 +IPD 载荷上限，超长的帧丢弃但仍计数字节以保持同步 */
#define IPD_LEN_MAX         2048  /* +IPD 长度字段的溢出保护上限 */
#define CMD_BUF_SIZE        32    /* 暂存下行命令 payload（"on"/"off" 之类） */
#define TOPIC_BUF_SIZE      40    /* topic 本地副本，够放 MQTT_TOPIC_CMD 并留 '\0' */

typedef enum {
    IPD_IDLE = 0,   /* 等 '+' */
    IPD_I,          /* 已收 '+'，等 'I' */
    IPD_P,          /* 等 'P' */
    IPD_D,          /* 等 'D' */
    IPD_NUM1,       /* 读第一段数字：单连接=载荷长度，多连接=link id */
    IPD_NUM2,       /* 多连接：读第二段数字=载荷长度 */
    IPD_PAYLOAD,    /* 收载荷，存进 s_ipd_buf */
    IPD_SKIP        /* 载荷超过 IPD_BUF_SIZE：只计数不存，收满后丢弃 */
} IPD_State_t;

static IPD_State_t s_ipd_state = IPD_IDLE;
static uint16_t    s_ipd_len   = 0;     /* +IPD 声明的载荷长度 */
static uint16_t    s_ipd_got   = 0;     /* 已收载荷字节数 */
static uint8_t     s_ipd_buf[IPD_BUF_SIZE];

/* 跨 +IPD 帧的半包暂存：TCP 会把一个 MQTT 包拆进两帧（broker 推 retained 时高发），
   解析器在帧尾遇到不完整包时把残段存这里，下一帧开头先拼上再解析 */
static uint8_t     s_carry[IPD_BUF_SIZE];
static uint16_t    s_carry_len = 0;
static uint8_t     s_combined[IPD_BUF_SIZE];

static char        s_pending_cmd[CMD_BUF_SIZE];
static uint8_t     s_pending_valid = 0;

/**
  * @brief  在一帧完整的 +IPD 载荷上解析 MQTT 包（一帧可能含多个包）
  * @param  buf: 载荷首地址，len: 载荷字节数
  */
static void MQTT_ParseFrame(const uint8_t *buf, uint16_t len)
{
    /* topic / payload 的本地副本：包内字段不带 '\0'，必须拷出来再终止，
       不能在 buf 上就地写终止符（旧实现的越界写就是这么来的）。
       用 static 是为了不占主循环栈（Stack_Size 只有 0x400）。 */
    static char topic_s[TOPIC_BUF_SIZE];
    static char payload_s[CMD_BUF_SIZE];
    uint16_t off = 0;

    while (off + 2 <= len)
    {
        uint8_t  type = buf[off] & 0xF0;
        uint16_t rl   = buf[off + 1];    /* 剩余长度：本项目 QoS 0 小包，单字节够用 */
        const uint8_t *body;
        uint16_t tlen, plen, i;

        if (rl > 127) {
            U1_PRINTF("[MQTT] rx: remaining length >127 unsupported, frame dropped\r\n");
            return;
        }
        /* 卫兵：包体必须完整落在这一帧里。
           不完整 = TCP 拆包：残段存入 carry，与下一帧拼接后再解析（不再丢弃） */
        if (off + 2 + rl > len) {
            if (len > off && (uint16_t)(len - off) <= sizeof(s_carry)) {
                uint16_t k;
                for (k = 0; k < (uint16_t)(len - off); k++) s_carry[k] = buf[off + k];
                s_carry_len = (uint16_t)(len - off);
                U1_PRINTF("[MQTT] rx: partial pkt %uB carried to next frame\r\n",
                          (unsigned)s_carry_len);
            } else {
                s_carry_len = 0;
            }
            return;
        }

        if (type == MQTT_PKT_PINGRESP) {
            U1_PRINTF("[MQTT] PINGRESP\r\n");
            off += 2 + rl;
            continue;
        }

        if (type != MQTT_PKT_PUBLISH) {
            /* CONNACK / SUBACK 由各自的握手路径用签名扫描处理，这里只跳过 */
            off += 2 + rl;
            continue;
        }

        /* ---- PUBLISH：body = topic_len(2) + topic + payload ---- */
        body = buf + off + 2;
        tlen = ((uint16_t)body[0] << 8) | body[1];

        /* 卫兵：topic 长度不能超出剩余长度 */
        if (tlen + 2 > rl) {
            U1_PRINTF("[MQTT] rx: bad topic len %u (rl=%u), frame dropped\r\n",
                      (unsigned)tlen, (unsigned)rl);
            return;
        }
        plen = rl - 2 - tlen;   /* QoS 0 无 packet id，剩下的全是 payload */

        /* 卫兵：两个字段都要装得下本地副本，且留得出 '\0' */
        if (tlen >= sizeof(topic_s) || plen >= sizeof(payload_s)) {
            U1_PRINTF("[MQTT] rx: topic/payload too long (%u/%u), frame dropped\r\n",
                      (unsigned)tlen, (unsigned)plen);
            return;
        }

        for (i = 0; i < tlen; i++) topic_s[i]   = (char)body[2 + i];
        topic_s[tlen] = '\0';
        for (i = 0; i < plen; i++) payload_s[i] = (char)body[2 + tlen + i];
        payload_s[plen] = '\0';

        U1_PRINTF("[MQTT] rx PUBLISH topic=%s payload=%.32s\r\n", topic_s, payload_s);

        if (strcmp(topic_s, MQTT_TOPIC_CMD) == 0) {
            strncpy(s_pending_cmd, payload_s, sizeof(s_pending_cmd) - 1);
            s_pending_cmd[sizeof(s_pending_cmd) - 1] = '\0';
            s_pending_valid = 1;
        }

        off += 2 + rl;
    }
}

/**
  * @brief  把一个 UART 字节喂给 +IPD 分帧器
  * @note   非阻塞、无延时、不回调；可安全地从发送路径调用
  */
static void MQTT_FeedByte(uint8_t ch)
{
    switch (s_ipd_state)
    {
        case IPD_IDLE:
            if (ch == '+') s_ipd_state = IPD_I;
            break;

        case IPD_I:
            if (ch == 'I')      s_ipd_state = IPD_P;
            else if (ch == '+') s_ipd_state = IPD_I;   /* 连续 '+'，停留在起始态 */
            else                s_ipd_state = IPD_IDLE;
            break;

        case IPD_P:
            s_ipd_state = (ch == 'P') ? IPD_D    : IPD_IDLE;
            break;

        case IPD_D:
            if (ch == 'D') { s_ipd_len = 0; s_ipd_state = IPD_NUM1; }
            else             s_ipd_state = IPD_IDLE;
            break;

        case IPD_NUM1:
            if (ch >= '0' && ch <= '9') {
                s_ipd_len = s_ipd_len * 10 + (uint16_t)(ch - '0');
                if (s_ipd_len > IPD_LEN_MAX) { s_ipd_len = 0; s_ipd_state = IPD_IDLE; }
            }
            else if (ch == ',') {                 /* 多连接：第一段是 link id */
                s_ipd_len   = 0;
                s_ipd_state = IPD_NUM2;
            }
            else if (ch == ':') {                 /* 单连接：第一段就是长度 */
                s_ipd_got   = 0;
                if (s_ipd_len == 0)                s_ipd_state = IPD_IDLE;
                else if (s_ipd_len > IPD_BUF_SIZE) s_ipd_state = IPD_SKIP;
                else                               s_ipd_state = IPD_PAYLOAD;
            }
            else s_ipd_state = IPD_IDLE;
            break;

        case IPD_NUM2:
            if (ch >= '0' && ch <= '9') {
                s_ipd_len = s_ipd_len * 10 + (uint16_t)(ch - '0');
                if (s_ipd_len > IPD_LEN_MAX) { s_ipd_len = 0; s_ipd_state = IPD_IDLE; }
            }
            else if (ch == ':') {
                s_ipd_got   = 0;
                if (s_ipd_len == 0)                s_ipd_state = IPD_IDLE;
                else if (s_ipd_len > IPD_BUF_SIZE) s_ipd_state = IPD_SKIP;
                else                               s_ipd_state = IPD_PAYLOAD;
            }
            else s_ipd_state = IPD_IDLE;
            break;

        case IPD_PAYLOAD:
            if (s_ipd_got < IPD_BUF_SIZE) s_ipd_buf[s_ipd_got] = ch;
            s_ipd_got++;
            if (s_ipd_got >= s_ipd_len) {
                if (s_carry_len > 0) {
                    /* 帧头拼上上一帧遗留的半包残段 */
                    uint16_t total = (uint16_t)(s_carry_len + s_ipd_got);
                    if (total <= IPD_BUF_SIZE) {
                        uint16_t k;
                        for (k = 0; k < s_carry_len; k++) s_combined[k] = s_carry[k];
                        for (k = 0; k < s_ipd_got;  k++) s_combined[s_carry_len + k] = s_ipd_buf[k];
                        s_carry_len = 0;
                        MQTT_ParseFrame(s_combined, total);
                    } else {
                        U1_PRINTF("[MQTT] rx: carry+frame overflow %uB, dropped\r\n",
                                  (unsigned)total);
                        s_carry_len = 0;
                    }
                } else {
                    MQTT_ParseFrame(s_ipd_buf, s_ipd_got);
                }
                s_ipd_len   = 0;
                s_ipd_got   = 0;
                s_ipd_state = IPD_IDLE;
            }
            break;

        case IPD_SKIP:
            /* 帧太长装不下：继续计数直到帧尾，保证后续字节同步不被打乱 */
            s_ipd_got++;
            if (s_ipd_got >= s_ipd_len) {
                U1_PRINTF("[MQTT] rx: +IPD frame %u bytes > buf %u, dropped\r\n",
                          (unsigned)s_ipd_len, (unsigned)IPD_BUF_SIZE);
                s_ipd_len   = 0;
                s_ipd_got   = 0;
                s_ipd_state = IPD_IDLE;
            }
            break;

        default:
            s_ipd_state = IPD_IDLE;
            break;
    }
}

void ESP8266_MQTTProcessRx(void)
{
    uint8_t ch;

    while (USART3_GetByte(&ch))
        MQTT_FeedByte(ch);

    /* 回调统一在这里派发（主循环上下文），FeedByte 内部只暂存 */
    if (s_pending_valid) {
        s_pending_valid = 0;
        MQTT_OnCmdReceived(s_pending_cmd);
    }
}

// 原巴法云实现（已下线，保留供参考）
#if 0  /* 巴法云已下线，保留供参考 */

uint8_t ESP8266_BemfaConnect(void)
{
    U1_PRINTF("[BEMFA] Closing old socket...\r\n");
    ESP8266_SendAT("AT+CIPCLOSE\r\n", "OK", 2000);
    delay_ms(500);

    U1_PRINTF("[BEMFA] Connecting %s:%d\r\n", BEMFA_SERVER, BEMFA_PORT);
    if (!ESP8266_ConnectTCP(BEMFA_SERVER, BEMFA_PORT)) {
        U1_PRINTF("[BEMFA] Connect FAIL\r\n");
        return 0;
    }
    U1_PRINTF("[BEMFA] TCP OK, subscribing...\r\n");
    delay_ms(500);
    return ESP8266_BemfaSubscribe();
}

uint8_t ESP8266_BemfaSubscribe(void)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "cmd=1&uid=%s&topic=%s\r\n", BEMFA_UID, BEMFA_TOPIC);
    U1_PRINTF("[BEMFA] Sub topic=%s\r\n", BEMFA_TOPIC);
    return ESP8266_SendTCP((uint8_t *)cmd, (uint16_t)strlen(cmd));
}

uint8_t ESP8266_BemfaPublish(const char *msg)
{
    char cmd[192];
    int len;
    if (msg == 0) return 0;
    len = snprintf(cmd, sizeof(cmd), "cmd=2&uid=%s&topic=%s&msg=%s\r\n",
                  BEMFA_UID, BEMFA_TOPIC, msg);
    if (len <= 0 || len >= (int)sizeof(cmd)) return 0;
    return ESP8266_SendTCP((uint8_t *)cmd, (uint16_t)len);
}

uint8_t ESP8266_BemfaHeartbeat(void)
{
    const char *cmd = "ping\r\n";
    return ESP8266_SendTCP((uint8_t *)cmd, (uint16_t)strlen(cmd));
}

#endif  /* 巴法云代码块结束 */


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
    while (USART3_GetByte(&ch)) MQTT_FeedByte(ch);

    /* 3. 确保 CIPMUX=0 单连接模式 */
    for (retry = 0; retry < 3; retry++) {
        if (ESP8266_SendAT("AT+CIPMUX=0\r\n", "OK", 2000)) {
            U1_PRINTF("[NTP] CIPMUX=0 OK\r\n");
            break;
        }
        delay_ms(500);
        while (USART3_GetByte(&ch)) MQTT_FeedByte(ch);
    }
    if (retry >= 3) {
        U1_PRINTF("[NTP] CIPMUX=0 failed\r\n");
        goto restore_udp;
    }
    delay_ms(200);

    /* 4. 建立 UDP 连接到 NTP 服务器 */
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n",
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
    while (USART3_GetByte(&ch)) MQTT_FeedByte(ch);
    ESP8266_SendAT("AT+CIPCLOSE\r\n", NULL, 0);
    delay_ms(300);

#if !USE_MQTT
    /* 恢复 UDP 连接到电脑 */
    U1_PRINTF("[NTP] Restoring UDP to PC...\r\n");
    if (!ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT)) {
        U1_PRINTF("[NTP] UDP restore FAIL, retrying...\r\n");
        delay_ms(500);
        ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT);
    }
#else
    /* MQTT 模式下没有 UDP 数据通道：这里的恢复（CIPMUX 2s + CIPSTART 5s +
       失败重试 ~7s）纯属浪费，且紧接着 main 会 CIPCLOSE 重建 MQTT TCP，
       跳过可省 ~7-14s 启动时间（P2 断网启动优化）。 */
#endif

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
    /* 1. 轻量探测：发 AT，1s 内无响应认为模块挂死。
       探测加一次重试：迟到的 +IPD 下行帧会与探测撞车挤掉 OK，
       单次探测失败就 Reset+完整重连（20~70s 调度冻结）属于误伤（实测复现）。 */
    uint8_t probe_ok = 0;
    for (uint8_t i = 0; i < 2 && !probe_ok; i++)
        probe_ok = ESP8266_SendAT("AT\r\n", "OK", 1000);

    if (!probe_ok)
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

    /* 2. 模块活着但连续发送失败 → 重建 MQTT（TCP + CONNECT + SUBSCRIBE） */
    if (s_send_fail_count >= 3)
    {
#if USE_MQTT
        U1_PRINTF("[ESP8266] %d send fails, reconnecting MQTT...\r\n", s_send_fail_count);
        if (ESP8266_MQTTConnect()) { if (ESP8266_MQTTSubscribe()) { s_mqtt_ok = 1; MQTT_ReportLightStatus(); MQTT_ReportModeStatus(); } }
#else
        U1_PRINTF("[ESP8266] %d send fails, reconnecting UDP...\r\n", s_send_fail_count);
        ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT);
#endif
        s_send_fail_count = 0;
    }

    return 1;
}
