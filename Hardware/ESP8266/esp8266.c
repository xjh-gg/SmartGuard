/**
  ******************************************************************************
  * @file    esp8266.c
  * @brief   ESP8266 WiFi ģ������ʵ�֣�AT ָ�� + ��д MQTT 3.1.1��
  *
  * ˵����
  *   ���� AT ָ���ͨ�� USART3 ���ͣ�ESP8266 Ӧ��Ҳ�� USART3 ���ա�
  *   ���ղ����ж� + ���λ���������ѭ��ͨ�� USART3_GetByte ��ȡ��
  *   MQTT �����ڱ��ļ��ֹ���֡���̶�ͷ/ʣ�೤��/�ɱ�ͷ/payload����
  *   ������ +IPD ǰ׺���� + MQTT �������״̬����δ���κ��ֳ� SDK��
  ******************************************************************************
  */
#include "esp8266.h"
#include "usart.h"
#include "delay.h"
#include "iwdg.h"
#include <string.h>
#include <stdio.h>

#define U1_PRINTF  USART1_Printf   /* easy to switch to USART2/other UART for debugging */

/* ���� #236-D: controlling expression is constant
   ԭ��MQTT_OnCmdReceived ����ָ���������ڳ����������������ж� */
#pragma diag_suppress 236

static uint8_t  s_send_fail_count = 0;  /* ��������ʧ�ܼ��������ڽ������ */
static volatile uint8_t s_wifi_ok = 0;  /* WiFi ����״̬���� OLED ״̬����ȡ */
static uint8_t  s_mqtt_ok = 0;          /* MQTT ������ɱ�־��CONNECT+SUBSCRIBE ���ɹ��� */

/* ���� +IPD ֡���������ֽ���ڣ��������ļ�ĩβ�� MQTT ����
   ��ǰ��������Ϊ ESP8266_SendTCP() ����ǰҲҪ���������ѵ��������֡
   ι��������������ֱ�Ӷ����� */
static void MQTT_FeedByte(uint8_t ch);

/* Consecutive full-reconnect failures: drives escalating backoff
   (5s -> 15s -> 60s) and, after 2 strikes, WiFi re-association / module reset. */
static uint8_t  s_reconnect_streak = 0;

/* AT �ȴ���϶�ص���P3-#4 ���з�������broker 崻�ʱ publish ·������������
   ����� ~15s��CIPSTART 10s ��������Э��ʽ���������嶳�ᣬ���� 5Hz
   ������ͣ���ڵȴ�ѭ����ÿ 200ms ��һ�α����ӣ�main ע�������ɷ���
   �����ڼ�Ļζ���ⲻ������������ι��SendAT ÿ 1ms IWDG_Feed���� */
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
  * @brief  ��ʼ�� ESP8266 ͨ�Žӿڲ����� WiFi
  * @note   �� 115200 ������̽�⣬ʧ���򽵼� 9600���ٲ� AT���� WiFi����ѡ�� UDP
  * @retval 1 = WiFi ���ӳɹ���0 = ʧ��
  */
uint8_t ESP8266_Init(void)
{
    uint32_t baud;
    uint8_t  at_ok = 0;

    U1_PRINTF("[ESP] --- ESP8266_Init start ---\r\n");

    /* 1. ������̽�⣺���� 115200�������ٽ��� 9600 */
    for (baud = 115200; baud >= 9600; baud = (baud == 115200) ? 9600 : 0)
    {
        if (baud == 0) break;
        U1_PRINTF("[ESP] trying USART3 @ %lu baud...\r\n", baud);
        USART3_Init(baud);
        delay_ms(200);   /* �� USART �ȶ� */
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
        /* ģ���ϵ��������ܱ� STM32 ����ESP8266 ���� boot �� ~1-2s����
           �� 2s ��һ��̽�������У����⿪������������ģʽ��ʵ�⸴�֣���
           ����ֶ�ι������������������ CheckHealth��re-init ·��Ҳ�ɴ
           ���� delay_ms(2000) ǡ�õ��� IWDG ���ڣ���ҧ��ͬ Reset �Ľ�ѵ���� */
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

    /* 2. ������λһ�Σ�ȷ���ɾ�״̬������һ�� AT */
    U1_PRINTF("[ESP] resetting module...\r\n");
    ESP8266_Reset();
    delay_ms(500);

    if (!ESP8266_SendAT("AT\r\n", "OK", 3000))
    {
        U1_PRINTF("[ESP] no AT after reset, giving up\r\n");
        s_wifi_ok = 0;
        return 0;
    }

    /* 3. ���� WiFi */
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
        s_mqtt_ok = 1;   /* CONNECT + SUBSCRIBE ���ɹ���MQTT �Ự���� */
        U1_PRINTF("[MQTT] Init OK (session ready)\r\n");
        MQTT_ReportLightStatus();    /* �״λỰ�����������ϱ���״̬��broker retain�� */
        MQTT_ReportModeStatus();   /* �״λỰ�����������ϱ�ģʽ״̬��broker retain�� */
    }
#else
    /* 3. ���� UDP ���ӵ����ԣ������Ƹ澯�� */
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

    return 1;   /* WiFi �����ϣ�UDP ʧ�ܲ����������� */
}

/**
  * @brief  ���� AT ָ��ȴ�ָ��Ӧ�𣨴�����������־��
  * @param  cmd:    Ҫ���͵� AT ָ���ַ��������Ѱ��� \r\n��
  * @param  ack:    �����յ���Ӧ���ַ�����NULL ��ʾ���ȴ�Ӧ��
  * @param  timeout_ms: ��ʱʱ�䣨���룩
  * @retval 1 = �յ�Ӧ��0 = ��ʱ
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
  * @brief  ������λ ESP8266
  */
void ESP8266_Reset(void)
{
    uint8_t i;

    ESP8266_SendAT("AT+RST\r\n", "ready", 3000);

    /* ��λ����Ҫ�ȴ�������ɡ�����ֶ�ι����IWDG ��ʱ������ 2000ms��
       ���� delay_ms(2000) ��������ι����������ʱ�޺���IWDG_Init ��
       ����ѧϰ֮���ִ�У����� IWDG һ������ֻ�ܿ�����رգ��ȸ�λ
       ��RESET �� / ������¼���ϵ磩���������ܣ��� 2 ���Ȼ��ҧ����
       ����ÿ���ȸ�λ����ɿ�����ѭ����ʵ�⣺+0.41/+2.40/+4.38s ���� banner���� */
    for (i = 0; i < 20; i++)
    {
        delay_ms(100);
        IWDG_Feed();
    }
}

/**
  * @brief  ���� ESP8266 ����ģʽ�����̼��汾 fallback��
  * @param  mode: 1=STA  Station ģʽ
  *              2=AP   Access Point ģʽ
  *              3=AP+STA ���ģʽ
  * @note   �°�̼��� CWMODE_CUR���Ϲ̼�ֻ�� CWMODE���޺�׺��
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
  * @brief  STA ģʽ����ָ�� WiFi�����̼��汾 fallback��
  * @note   �°�̼��� CWJAP_CUR���Ϲ̼�ֻ�� CWJAP���޺�׺��
  */
uint8_t ESP8266_ConnectAP(char *ssid, char *password)
{
    char cmd[128];
    uint8_t ok;

    /* 1. ���� STA ģʽ���� fallback�� */
    U1_PRINTF("[ESP] setting STA mode...\r\n");
    if (!ESP8266_SetMode(1))
    {
        U1_PRINTF("[ESP] SetMode FAIL, cannot continue\r\n");
        s_wifi_ok = 0;
        return 0;
    }

    /* 2. �ȶϿ��ϴ����ӣ��� WIFI DISCONNECT ������ */
    ESP8266_SendAT("AT+CWQAP\r\n", "OK", 1000);
    delay_ms(200);

    /* 3. ���� CWJAP_CUR���¹̼�����ʧ�� fallback CWJAP���Ϲ̼��� */
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
  * @brief  ��ѯ WiFi �Ƿ����ߣ��� OLED ״̬����ֻ��ʹ�ã�
  * @retval 1=���ߣ�0=����
  */
uint8_t ESP8266_IsWiFiOk(void)
{
    return s_wifi_ok;
}

/**
  * @brief  ��ѯ MQTT �Ự�Ƿ�������� OLED ״̬�е�ֻ��ʹ�ã�
  * @retval 1=�Ự������CONNECT+SUBSCRIBE �ɹ���δ�ϣ���0=δ����
  */
uint8_t ESP8266_IsMQTTOk(void)
{
    return s_mqtt_ok;
}

/**
  * @brief  UDP ���ӣ�������ģʽ��
  * @param  ip:   Ŀ�� IP ��ַ�ַ���
  * @param  port: Ŀ��˿ں�
  */
uint8_t ESP8266_ConnectUDP(char *ip, uint16_t port)
{
    char cmd[128];
    uint8_t ok;

    /* UDP ʹ�õ�����ģʽ */
    U1_PRINTF("[...] ESP8266 AT+CIPMUX=0\r\n");
    ok = ESP8266_SendAT("AT+CIPMUX=0\r\n", "OK", 2000);
    U1_PRINTF("  -> CIPMUX=0 %s\r\n", ok ? "OK" : "FAIL");
    if (!ok) {
        /* �����Ѿ��� 0 �ˣ����� */
    }
    delay_ms(200);

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n", ip, port);
    U1_PRINTF("[...] ESP8266 %s", cmd);
    ok = ESP8266_SendAT(cmd, "OK", 5000);
    U1_PRINTF("  -> CIPSTART UDP %s\r\n", ok ? "OK" : "FAIL (may be firewall, try anyway)");

    /* ���� CIPSTART ����ʵ�����ԭʵ�ֺ㷵�� 1��ʧ�ܱ��ڸǣ���
       UDP �����������塢ʧ�ܺ��Կ��ܷ��������� AT ʧ��ͨ����ζ��
       ģ�������쳣�����÷���esp8266.c:107/1250 �ȣ����� 0/1 ��֧���� */
    return ok;
}

/**
  * @brief  ���� UDP ����
  * @param  data: ���ݻ�����
  * @param  len:  ���ݳ���
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

    /* �� SEND OK�������ֽڲ�����ģ�����/�������� SendTCP ���룬P3-#8����
       ԭʵ�ַ���ֱ�ӱ��ɹ���ģ�����ʱ�澯��Ĭ��ʧ */
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
  * @brief  �� ESP8266 �����������ݣ����� +IPD ֡��
  * @param  buf:    �������ݻ�����
  * @param  len:    ��������󳤶�
  * @param  timeout_ms: �ȴ���ʱʱ�䣨���룩
  * @retval ʵ�ʽ��յ����ֽ�����0 ��ʾ��ʱ��δ�յ�����
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
                case 0:   /* �ȴ� '+' */
                    if (ch == '+') state = 1;
                    break;
                case 1:   /* '+' ��� 'I' */
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
                case 4:   /* 'IPD,' ���ȡ���� */
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
                case 5:   /* ��ȡʵ������ */
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
  * @brief  ʹ�� AT+PING ��� ESP8266 �Ƿ�����ͨ����
  * @retval 1=�� ping ͨ������ַ��0=����
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
  * @brief  TCP ���ӣ�������ģʽ��
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

/* TCP ʧ��ͳһ�Ǽǣ���Ự��־��������װ���ߣ�+ �ۼ�ʧ�ܼ�������������飩��
   ��ǰ SendTCP ��ʧ��·����Ĭ return 0��s_mqtt_ok �� 1��s_send_fail_count
   ֻ�� MQTT ģʽ���������õ� SendUDP ����������� TCP ������Զ�޻ָ��� */
static void TCP_NoteFail(void)
{
    s_mqtt_ok = 0;
    if (s_send_fail_count < 250) s_send_fail_count++;
}

/**
  * @brief  ���� TCP ���ݣ��� link �Զ�������
  */
uint8_t ESP8266_SendTCP(uint8_t *data, uint16_t len)
{
    char cmd[32];
    uint16_t i;
    uint8_t ch;
    uint8_t need_reconnect = 0;

    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", len);
    /* ����ǰ��ջ��λ��塣����ֱ�Ӷ���ң��ÿ 10s ��һ�Σ���������������
       �������������ͻᱻ�̵�������־����������Ϊι�� +IPD ��֡����
       FeedByte ֻ�ݴ�������ص�����˲����뱾�η����γ����롣 */
    while (USART3_GetByte(&ch)) MQTT_FeedByte(ch);

    if (!ESP8266_SendAT(cmd, ">", 3000))
    {
        /* �Ȳ��� ">" ���ж���·�쳣������������һ�� TCP ����+�ط���
           ԭʵ�ֿ� err_buf ƥ�� "link is not valid"/"ERROR" �����Ƿ�������
           �� SendAT �ڵ� ">" �� 3s �������Ѱ�ģ����Ӧ�̽��Լ��� rx_buf
           ����ʱ����ʱ��ջ֡�������������ٶ����λ��峣��ʱ���º�Ϊ�գ�
           ������ƥ��Ķ������ʵ�ʲ��ɴ���������롣 */
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
        s_mqtt_ok = 0;   /* TCP �ؽ��ˣ�MQTT �Ự���ˣ����������� */
        if (!ESP8266_ConnectTCP(MQTT_SERVER, MQTT_PORT))
        {
            U1_PRINTF("[ESP8266] TCP reconnect FAIL\r\n");
            TCP_NoteFail();
            return 0;
        }
        /* �� socket ��û�� MQTT CONNECT �Ự�����ط� PUBLISH ���Ļᱻ
           broker ��Э��Υ��ֱ�Ӷ���������ȴ����"�ɹ�"�����澯��Ĭ��ʧ��
           ��װ����ʵ����ʧ�ܣ�s_mqtt_ok ���壬��һ�η�������
           EnsureSession ��� CONNECT+SUBSCRIBE �����ֺ��ٷ���P3-#8�� */
        U1_PRINTF("[ESP8266] TCP reconnected, MQTT session lost, msg dropped honestly\r\n");
        return 0;
    }

    delay_ms(100);   /* �� > ��ʾ�ȶ� */

    for (i = 0; i < len; i++) USART3_SendByte(data[i]);
    U1_PRINTF("[ESP8266] sent %d bytes\r\n", len);

    /* Read SEND OK from ESP8266, else next AT+CIPSEND will be rejected */
    {
        uint8_t  ok_buf[64] = {0};
        uint16_t ok_idx = 0;
        uint16_t ok_timeout = 2000;
        while (ok_timeout--) {
            while (USART3_GetByte(&ch)) {
                /* ����ȴ������� 50~200ms���ڼ䵽������� +IPD ֡��ֻ�� ok_buf
                   �ͻ��� return һ�𶪵����ֽ�һ����ι���������������������
                   SEND OK ֮ǰ�����֡������ ok_buf 63 �ֽ����ޱ��ضϵ�֡��
                   �Լ���ʱ return 0 ·���ϵ�֡�������� return ǰ�ٻع� ok_buf��
                   ����ͬһ���ֽڻᱻι���Σ��ƻ� +IPD ״̬����ͬ���� */
                MQTT_FeedByte(ch);
                if (ok_idx < sizeof(ok_buf) - 1) {
                    ok_buf[ok_idx++] = ch;
                    ok_buf[ok_idx]   = '\0';
                }
            }
            if (strstr((char *)ok_buf, "SEND OK")) {
                s_send_fail_count = 0;   /* ���ͳɹ���������ʧ�ܼ��� */
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
 * MQTT 3.1.1 ����ʵ�֣����ԭ�ͷ��� TCP Э�飩
 * �ײ� TCP �շ�ֱ�Ӹ��� ESP8266_ConnectTCP / ESP8266_SendTCP
 * ===================================================================== */

/* д MQTT UTF-8 �ַ�����2 �ֽڴ�˳���ǰ׺ + payload�� */
static uint16_t MQTT_WriteStr(uint8_t *dst, const char *s)
{
    uint16_t len = (uint16_t)strlen(s);
    dst[0] = (len >> 8) & 0xFF;
    dst[1] =  len       & 0xFF;
    memcpy(dst + 2, s, len);
    return len + 2;
}

/**
  * @brief  ���� MQTT Broker��TCP ���� + MQTT CONNECT
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

    /* --- ���� MQTT CONNECT ���� ---
       �̶�ͷ: 0x10 | ʣ�೤�� (���ֽڣ�<=127)
       �ɱ�ͷ: Э���� "MQTT"(6) + ���� 0x04 + ��־ 0x02(Clean) + KeepAlive(2)
       Payload: ClientID
    */
    pos = 2;  /* Ԥ���̶�ͷ 2 �ֽ� */
    pos += MQTT_WriteStr(pkt + pos, "MQTT");    /* Э���� */
    pkt[pos++] = 0x04;                           /* Э�鼶�� 4 = 3.1.1 */
    pkt[pos++] = 0x02;                           /* Connect Flags: CleanSession=1 */
    pkt[pos++] = (MQTT_KEEPALIVE_SEC >> 8) & 0xFF;
    pkt[pos++] =  MQTT_KEEPALIVE_SEC       & 0xFF;
    pos += MQTT_WriteStr(pkt + pos, MQTT_CLIENT_ID);

    pkt[0] = 0x10;                               /* CONNECT */
    pkt[1] = (uint8_t)(pos - 2);                 /* ʣ�೤�� */

    U1_PRINTF("[MQTT] Sending CONNECT (%d bytes)\r\n", pos);
    if (!ESP8266_SendTCP(pkt, pos)) {
        U1_PRINTF("[MQTT] CONNECT send FAIL\r\n");
        return 0;
    }

    /* �� CONNACK���̶�ͷ 0x20 0x02 XX 0x00(������=0)
       ESP8266 ����������ͨ���� +IPD,N: ǰ׺���� CIPSEND �� SEND OK
       ��������ܰ�ǰ׺�̳ɲ�ȱ�� PD,/D,/��: �ȣ���˲��ܼ��� CONNACK
       ���ֽ� 0 ��ʼ����������λ���ɨ�� 0x20 0x02 XX 0x00 ǩ���� */
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
  * @brief  ���Ŀ���ָ������ MQTT_TOPIC_CMD
  * @retval 1=OK, 0=FAIL
  */
uint8_t ESP8266_MQTTSubscribe(void)
{
    uint8_t pkt[64];
    uint16_t pos;
    uint16_t pkt_id = 1;

    pos = 2;   /* �̶�ͷ 2 �ֽ� */
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

    /* �� SUBACK: 0x90 <ʣ�೤��> <pkt_id��> <pkt_id��> <0x00>
       �� CONNACK ͬ����SUBACK Ҳ�� +IPD,N: ǰ׺�ҿ��ܱ� SEND OK
       �������飬�ʶ���������ɨ�� 0x90 ... 0x00 ǩ���� */
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
  * @brief  ȷ�� MQTT �Ự��Ч��TCP + CONNECT + SUBSCRIBE �� OK��
  * @retval 1=�Ự������0=����ʧ��
  */
static uint8_t MQTT_EnsureSession(void)
{
    /* �����˱ܣ�broker ������ʱң��(10s)/����(30s)ÿ�� publish �����ߵ����
       ���˱�ʱÿ�����ڶ���һ���������֣�CIPSTART 10s ��ʱ������� ~15s����
       Э��ʽ������������������������/��ʾ/����ȫ��������
       ���������������ÿ 60s һ�Σ��ڼ����ʧ�ܣ��״ε��ñط��С� */
    /* Escalating backoff: 5s after 1st failure, 15s after 2nd, 60s cap.
       A brief broker/router blip heals in 5s; a long outage stays gentle. */
    static const uint32_t backoff_ms[3] = { 5000U, 15000U, 60000U };
    static uint32_t s_last_attempt = 0;
    uint32_t cur_backoff;

    if (s_mqtt_ok) return 1;   /* �Ự�����ţ�ֱ���� */

    cur_backoff = backoff_ms[(s_reconnect_streak >= 2) ? 2 : s_reconnect_streak];
    if (s_last_attempt != 0 &&
        (g_tick - s_last_attempt) < cur_backoff)
        return 0;
    s_last_attempt = g_tick;

    U1_PRINTF("[MQTT] session lost, reconnect (streak=%u backoff=%lums)...\r\n",
              (unsigned)s_reconnect_streak, (unsigned long)cur_backoff);
    if (ESP8266_MQTTConnect() && ESP8266_MQTTSubscribe())
    {
        s_reconnect_streak = 0;
        goto session_restored;
    }
    s_reconnect_streak++;

    /* Recovery ladder after 2 consecutive TCP-level failures:
       the AP itself is usually gone (hotspot/router rebooted), so re-associate
       WiFi before rebuilding the session. If re-association also fails, the
       module may be wedged -> AT+RST + full re-init (baud/WiFi/MQTT).
       All of this blocks TaskComm only; sensing/alarm/keys/display keep running. */
    if (s_reconnect_streak >= 2)
    {
        U1_PRINTF("[NET] 2 strikes, re-associating WiFi...\r\n");
        s_wifi_ok = 0;
        s_mqtt_ok = 0;
        if (ESP8266_ConnectAP(WIFI_SSID, WIFI_PASSWORD))
        {
            U1_PRINTF("[NET] WiFi back, rebuilding MQTT session...\r\n");
            if (ESP8266_MQTTConnect() && ESP8266_MQTTSubscribe())
            {
                s_reconnect_streak = 0;
                goto session_restored;
            }
            s_reconnect_streak++;
        }
        else
        {
            U1_PRINTF("[NET] WiFi re-assoc FAIL, AT+RST + re-init...\r\n");
            s_last_attempt = g_tick;   /* long init below; defer next attempt */
            ESP8266_Reset();
            if (ESP8266_Init())
            {
                s_reconnect_streak = 0;
                return s_mqtt_ok;   /* Init rebuilds the MQTT session if broker is up */
            }
            U1_PRINTF("[NET] module re-init FAIL, will retry later\r\n");
        }
    }

    U1_PRINTF("[MQTT] CONNECT FAIL\r\n");
    return 0;

session_restored:
    s_mqtt_ok = 1;
    s_send_fail_count = 0;   /* �Ự�ָ���������ʧ�ܼ��� */
    U1_PRINTF("[MQTT] session restored OK\r\n");
    MQTT_ReportLightStatus();    /* �Ự���������������ϱ���״̬��broker retain�� */
    MQTT_ReportModeStatus();       /* �Ự���������������ϱ�ģʽ״̬��broker retain�� */
    return 1;
}

/**
  * @brief  ��ָ�����ⷢ����Ϣ��QoS 0�����ȴ� PUBACK��
  * @param  retain: 1=broker �������һ����Ϣ���¿ͻ��˶���ʱ�����ʹ0=������
  */
uint8_t ESP8266_MQTTPublish(const char *topic, const char *payload, uint8_t retain)
{
    uint8_t pkt[256];
    uint16_t pos;
    uint16_t tlen, plen;

    if (!topic || !payload) return 0;

    /* ȷ�� MQTT �Ự��Ч��TCP link ���� + CONNECT + SUBSCRIBE ����ɣ� */
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

    pos = 2;   /* �̶�ͷ 2 �ֽ� */
    pos += MQTT_WriteStr(pkt + pos, topic);
    memcpy(pkt + pos, payload, plen);
    pos += plen;

    pkt[0] = 0x30 | (retain ? 0x01 : 0x00);  /* PUBLISH QoS 0, retain=bit0 */
    pkt[1] = (uint8_t)(pos - 2);

    return ESP8266_SendTCP(pkt, pos);
}

/**
  * @brief  MQTT PINGREQ ������ȷ���Ự��Ч���ٷ���
  */
uint8_t ESP8266_MQTTPing(void)
{
    static uint8_t ping[] = {0xC0, 0x00};   /* PINGREQ + ʣ�೤�� 0 */

    if (!MQTT_EnsureSession()) return 0;
    return ESP8266_SendTCP(ping, 2);
}

/**
  * @brief  ���� USART3 ���λ����е� MQTT ���а�
  *         ����ʱ������ѭ��ÿ tick ��һ�Σ����������������κ���ʱ��
  *
  * ����״̬����
  *   ��һ�� IPD_* ���� ���� ESP8266 AT �̼��� "+IPD,<len>:" ǰ׺
  *                   �������ӹ̼��� "+IPD,<link id>,<len>:"�����ֶ�֧�֣���
  *                   ֻ�н����غ�̬֮���ֽڲŻᱻ���� MQTT �̶�ͷ��
  *   �ڶ��� MQTT_ParseFrame() ���� �������غ��ϰ� MQTT 3.1.1 �����
  *
  * Ϊʲô�����������ASCII '0'-'9' �� 0x30-0x39��':' �� 0x3A��'=' �� 0x3D��
  * ���� & 0xF0 ȫ������ 0x30����ʵ��ֱ������ UART ������ (ch & 0xF0)==0x30��
  * ���ǰ� "+IPD,2:" ��� '2' ���� PUBLISH �̶�ͷ��':' ����ʣ�೤�� 58��
  * ��������������̵���topic/payload ȫ������������
  *
  * ����˵����MQTT_FeedByte() ֻ������ֻ�������ݴ浽 s_pending_cmd��
  * ����������ص�����Ϊ ESP8266_SendTCP() ����ǰҲ��� FeedByte �建�壬
  * �� MQTT_OnCmdReceived() ����Ҫ MQTTPublish �ϱ�״̬������ FeedByte ��
  * ֱ�ӻص��ͻ��γ� SendTCP �� �ص� �� MQTTPublish �� SendTCP �����롣
  */

/* MQTT �̶�ͷ�� 4 λ */
#define MQTT_PKT_PUBLISH    0x30
#define MQTT_PKT_PINGRESP   0xD0

#define IPD_BUF_SIZE        128   /* ��֡ +IPD �غ����ޣ�������֡�������Լ����ֽ��Ա���ͬ�� */
#define IPD_LEN_MAX         2048  /* +IPD �����ֶε������������ */
#define CMD_BUF_SIZE        32    /* �ݴ��������� payload��"on"/"off" ֮�ࣩ */
#define TOPIC_BUF_SIZE      40    /* topic ���ظ��������� MQTT_TOPIC_CMD ���� '\0' */

typedef enum {
    IPD_IDLE = 0,   /* �� '+' */
    IPD_I,          /* ���� '+'���� 'I' */
    IPD_P,          /* �� 'P' */
    IPD_D,          /* �� 'D' */
    IPD_NUM1,       /* ����һ�����֣�������=�غɳ��ȣ�������=link id */
    IPD_NUM2,       /* �����ӣ����ڶ�������=�غɳ��� */
    IPD_PAYLOAD,    /* ���غɣ���� s_ipd_buf */
    IPD_SKIP        /* �غɳ��� IPD_BUF_SIZE��ֻ�������棬�������� */
} IPD_State_t;

static IPD_State_t s_ipd_state = IPD_IDLE;
static uint16_t    s_ipd_len   = 0;     /* +IPD �������غɳ��� */
static uint16_t    s_ipd_got   = 0;     /* �����غ��ֽ��� */
static uint8_t     s_ipd_buf[IPD_BUF_SIZE];

/* �� +IPD ֡�İ���ݴ棺TCP ���һ�� MQTT �������֡��broker �� retained ʱ�߷�����
   ��������֡β������������ʱ�Ѳжδ������һ֡��ͷ��ƴ���ٽ��� */
static uint8_t     s_carry[IPD_BUF_SIZE];
static uint16_t    s_carry_len = 0;
static uint8_t     s_combined[IPD_BUF_SIZE];

static char        s_pending_cmd[CMD_BUF_SIZE];
static uint8_t     s_pending_valid = 0;

/**
  * @brief  ��һ֡������ +IPD �غ��Ͻ��� MQTT ����һ֡���ܺ��������
  * @param  buf: �غ��׵�ַ��len: �غ��ֽ���
  */
static void MQTT_ParseFrame(const uint8_t *buf, uint16_t len)
{
    /* topic / payload �ı��ظ����������ֶβ��� '\0'�����뿽��������ֹ��
       ������ buf �Ͼ͵�д��ֹ������ʵ�ֵ�Խ��д������ô���ģ���
       �� static ��Ϊ�˲�ռ��ѭ��ջ��Stack_Size ֻ�� 0x400���� */
    static char topic_s[TOPIC_BUF_SIZE];
    static char payload_s[CMD_BUF_SIZE];
    uint16_t off = 0;

    while (off + 2 <= len)
    {
        uint8_t  type = buf[off] & 0xF0;
        uint16_t rl   = buf[off + 1];    /* ʣ�೤�ȣ�����Ŀ QoS 0 С�������ֽڹ��� */
        const uint8_t *body;
        uint16_t tlen, plen, i;

        if (rl > 127) {
            U1_PRINTF("[MQTT] rx: remaining length >127 unsupported, frame dropped\r\n");
            return;
        }
        /* �����������������������һ֡�
           ������ = TCP ������жδ��� carry������һ֡ƴ�Ӻ��ٽ��������ٶ����� */
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
            /* CONNACK / SUBACK �ɸ��Ե�����·����ǩ��ɨ�账��������ֻ���� */
            off += 2 + rl;
            continue;
        }

        /* ---- PUBLISH��body = topic_len(2) + topic + payload ---- */
        body = buf + off + 2;
        tlen = ((uint16_t)body[0] << 8) | body[1];

        /* ������topic ���Ȳ��ܳ���ʣ�೤�� */
        if (tlen + 2 > rl) {
            U1_PRINTF("[MQTT] rx: bad topic len %u (rl=%u), frame dropped\r\n",
                      (unsigned)tlen, (unsigned)rl);
            return;
        }
        plen = rl - 2 - tlen;   /* QoS 0 �� packet id��ʣ�µ�ȫ�� payload */

        /* �����������ֶζ�Ҫװ���±��ظ����������ó� '\0' */
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
  * @brief  ��һ�� UART �ֽ�ι�� +IPD ��֡��
  * @note   ������������ʱ�����ص����ɰ�ȫ�شӷ���·������
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
            else if (ch == '+') s_ipd_state = IPD_I;   /* ���� '+'��ͣ������ʼ̬ */
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
            else if (ch == ',') {                 /* �����ӣ���һ���� link id */
                s_ipd_len   = 0;
                s_ipd_state = IPD_NUM2;
            }
            else if (ch == ':') {                 /* �����ӣ���һ�ξ��ǳ��� */
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
                    /* ֡ͷƴ����һ֡�����İ���ж� */
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
            /* ̫֡��װ���£���������ֱ��֡β����֤�����ֽ�ͬ���������� */
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

    /* �ص�ͳһ�������ɷ�����ѭ�������ģ���FeedByte �ڲ�ֻ�ݴ� */
    if (s_pending_valid) {
        s_pending_valid = 0;
        MQTT_OnCmdReceived(s_pending_cmd);
    }
}

// ԭ�ͷ���ʵ�֣������ߣ��������ο���
#if 0  /* �ͷ��������ߣ��������ο� */

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

#endif  /* �ͷ��ƴ������� */


/**
  * @brief  Unix ʱ���ת����ʱ�䣨���������㣬������ localtime��
  * @param  ts: Unix ʱ������룬�� 1970-01-01 00:00:00 UTC��
  * @param  dt: ��� RTC_DateTime_t��Year �� 0-99����� 2000 �꣩
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

    /* �� 1970-01-01 �����������������¿۳� */
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
  * @brief  ͨ�� NTP/UDP ��ȡ����ʱ�䣨UTC+8 ����ʱ�䣩
  * @param  dt: �������ʱ��ṹ��
  * @retval 1=�ɹ���0=ʧ��
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

    /* 1. �رյ�ǰ���ӣ�ѭ�����ֱ�� OK �� ERROR */
    for (retry = 0; retry < 3; retry++) {
        close_ok = ESP8266_SendAT("AT+CIPCLOSE\r\n", "OK", 2000);
        if (!close_ok) {
            close_ok = ESP8266_SendAT("AT+CIPCLOSE\r\n", "ERROR", 2000);
        }
        if (close_ok) break;
        delay_ms(300);
    }
    delay_ms(200);

    /* 2. ��� USART3 �������� */
    while (USART3_GetByte(&ch)) MQTT_FeedByte(ch);

    /* 3. ȷ�� CIPMUX=0 ������ģʽ */
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

    /* 4. ���� UDP ���ӵ� NTP ������ */
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n",
            ESP8266_NTP_SERVER_IP, ESP8266_NTP_SERVER_PORT);
    if (!ESP8266_SendAT(cmd, "OK", 10000)) {
        U1_PRINTF("[NTP] UDP connect FAIL\r\n");
        goto restore_udp;
    }
    U1_PRINTF("[NTP] UDP connected to NTP server\r\n");
    delay_ms(500);

    /* 5. ���� 48 �ֽ� NTP ����� */
    memset(ntp_req, 0, sizeof(ntp_req));
    ntp_req[0] = 0x23;   /* LI=0, VN=4, Mode=3 (client) */

    /* 6. ���� NTP ���� */
    if (!ESP8266_SendUDP(ntp_req, 48)) {
        U1_PRINTF("[NTP] send failed\r\n");
        goto restore_udp;
    }
    U1_PRINTF("[NTP] request sent, waiting response...\r\n");

    /* 7. ���� NTP ��Ӧ��ԭʼ��ȡ���ƹ� +IPD ��������
     *    ������ NTP ������ʵ�ʷ��� 56 �ֽڣ��� 8 �ֽ���չ����䣩��
     *    �� ESP8266 +IPD ͷ���� 48 �ֽڣ����� ESP8266_Receive() �ضϡ�
     *    ����ԭʼ USART3 ��ȡ������ȫ�����ݺ��ڻ������н����� */
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

        /* ���ԣ���ӡԭʼ����ʮ������ */
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

        /* ��λ +IPD ��� NTP �غ���ʼ */
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

        /* 8. ��ȡ Transmit Timestamp����׼ NTP ƫ�� 40-43������� */
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

    /* 9. NTP ʱ��� �� Unix ʱ�������ȥ 70 ��ƫ�ƣ� */
    unix_ts = ntp_ts - 2208988800UL;

    /* 10. UTC �� ����ʱ�䣨UTC+8�� */
    unix_ts += (uint32_t)ESP8266_TIMEZONE_HOUR * 3600;

    U1_PRINTF("[NTP] ntp_ts=%lu unix_ts=%lu\r\n", ntp_ts, unix_ts);

    /* 11. ת��Ϊ����ʱ�� */
    unix_to_datetime(unix_ts, dt);

    U1_PRINTF("[NTP] OK: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
              dt->Year, dt->Month, dt->Date,
              dt->Hours, dt->Minutes, dt->Seconds);

restore_udp:
    /* �ر� NTP UDP ���� */
    while (USART3_GetByte(&ch)) MQTT_FeedByte(ch);
    ESP8266_SendAT("AT+CIPCLOSE\r\n", NULL, 0);
    delay_ms(300);

#if !USE_MQTT
    /* �ָ� UDP ���ӵ����� */
    U1_PRINTF("[NTP] Restoring UDP to PC...\r\n");
    if (!ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT)) {
        U1_PRINTF("[NTP] UDP restore FAIL, retrying...\r\n");
        delay_ms(500);
        ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT);
    }
#else
    /* MQTT ģʽ��û�� UDP ����ͨ��������Ļָ���CIPMUX 2s + CIPSTART 5s +
       ʧ������ ~7s�������˷ѣ��ҽ����� main �� CIPCLOSE �ؽ� MQTT TCP��
       ������ʡ ~7-14s ����ʱ�䣨P2 ���������Ż����� */
#endif

    return (dt->Year > 0 || dt->Month > 0) ? 1 : 0;
}

/**
  * @brief  ���͸澯��Ϣ��JSON ��ʽ��+ 5 ����ȴ��ˢ��
  */
static uint32_t s_last_send_tick = 0;   /* �ϴη��͵�ϵͳ���� */
#define SEND_COOLDOWN_MS  5000          /* 5 ���ڲ��ظ��� */

void ESP8266_SendAlert(const char *msg)
{
    /* ��ȴ��飺�� g_tick��delay.h ��� 1ms ȫ�ֽ��ģ� */
    uint32_t now = g_tick;
    if ((now - s_last_send_tick) < SEND_COOLDOWN_MS)
    {
        return;   /* ��ȴ�У����ظ��� */
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
  * @brief  �������ݣ�JSON ��ʽ��
  * @param  topic:   ��������/����
  * @param  payload: ���������ַ���
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
  * @brief  ESP8266 ����������Զ��ָ�
  * @note   ��ѭ��ÿ 5 ���ӵ���һ�Ρ�
  *         1) AT ����Ӧ �� ��λ + ����������WiFi + UDP��
  *         2) AT ����Ӧ����������ʧ�� �� ���ؽ� UDP ����
  *         3) һ������ �� �����κβ���
  */
uint8_t ESP8266_CheckHealth(void)
{
    /* 1. ����̽�⣺�� AT��1s ������Ӧ��Ϊģ�������
       ̽���һ�����ԣ��ٵ��� +IPD ����֡����̽��ײ������ OK��
       ����̽��ʧ�ܾ� Reset+����������20~70s ���ȶ��ᣩ�������ˣ�ʵ�⸴�֣��� */
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

    /* 2. ģ����ŵ���������ʧ�� �� �ؽ� MQTT��TCP + CONNECT + SUBSCRIBE�� */
    if (s_send_fail_count >= 3)
    {
#if USE_MQTT
        U1_PRINTF("[ESP8266] %d send fails, reconnecting MQTT...\r\n", s_send_fail_count);
        if (ESP8266_MQTTConnect()) { if (ESP8266_MQTTSubscribe()) { s_mqtt_ok = 1; s_reconnect_streak = 0; MQTT_ReportLightStatus(); MQTT_ReportModeStatus(); } }
#else
        U1_PRINTF("[ESP8266] %d send fails, reconnecting UDP...\r\n", s_send_fail_count);
        ESP8266_ConnectUDP(ESP8266_UDP_SERVER_IP, ESP8266_UDP_SERVER_PORT);
#endif
        s_send_fail_count = 0;
    }

    return 1;
}
