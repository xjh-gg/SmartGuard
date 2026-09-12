/*
 * ================================================================
 *  SmartGuard ESP32-CAM Firmware
 *  ---------------------------------------------------------------
 *  AI Thinker ESP32-CAM (ESP32 + OV2640)
 *  Arduino IDE 1.8.x / 2.x | Board: AI Thinker ESP32-CAM
 *  ---------------------------------------------------------------
 *  功能：
 *    1. OV2640 摄像头初始化（QQVGA 160x120，省流量也够用）
 *    2. WiFi STA 模式连接路由器
 *    3. HTTP Server:
 *       GET /          → 首页（移动端可看实时视频）
 *       GET /stream    → MJPEG 实时视频流
 *       GET /shot.jpg  → 最新一张快照
 *       GET /snapshot  → 手动拍一张返回 JPEG
 *    4. 串口协议（115200, 8N1, Serial 0 → GPIO1/3）:
 *       STM32 → ESP32CAM: "STATUS\r\n"    → 回 "READY\r\n"
 *       STM32 → ESP32CAM: "CAPTURE\r\n"   → 拍照，回 "OK http://IP/shot.jpg\r\n"
 *
 *  硬件连接（和 STM32F407ZE）：
 *    STM32 USART2_TX (PA2) ──→ ESP32-CAM U0R (GPIO3)
 *    STM32 USART2_RX (PA3) ◄── ESP32-CAM U0T (GPIO1)
 *    STM32 GND            ──→ ESP32-CAM GND
 *    USB 5V               ──→ ESP32-CAM 5V（≥500mA！别用 STM32 板载 3.3V）
 * ================================================================
 */

#include "WiFi.h"
#include "WebServer.h"
#include "esp_camera.h"
#include "esp32-hal-cpu.h"

/* ============ WiFi 配置（和 ESP8266 连同一个路由器） ============ */
const char *WIFI_SSID = "你的WiFi名称";
const char *WIFI_PASS = "<redacted-password>";

/* ============ 摄像头配置（AI Thinker ESP32-CAM 固定引脚） ============ */
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

/* ============ HTTP 服务器 + MJPEG ============ */
WebServer server(80);
WiFiClient client = WiFiClient();

/* ============ 全局快照（/shot.jpg 返回这张） ============ */
camera_fb_t *g_last_snapshot = NULL;
bool g_snapshot_busy = false;

/* ================================================================
 *  拍照并保存到全局快照
 *  非阻塞调用，busy 时返回 false
 * ================================================================ */
bool captureSnapshot()
{
    if (g_snapshot_busy) return false;
    g_snapshot_busy = true;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("ERR: capture failed");
        g_snapshot_busy = false;
        return false;
    }

    /* 释放上一张，替换为新的 */
    if (g_last_snapshot) esp_camera_fb_return(g_last_snapshot);
    g_last_snapshot = fb;

    g_snapshot_busy = false;
    return true;
}

/* ================================================================
 *  HTTP 路由
 * ================================================================ */

/* 首页 */
void handleIndex()
{
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width'>"
                  "<title>SmartGuard ESP32-CAM</title></head><body style='background:#1a1a1a;color:#fff'>"
                  "<h2>SmartGuard ESP32-CAM</h2>"
                  "<img src='/stream' style='max-width:100%;border:1px solid #444'>"
                  "<p>IP: " + WiFi.localIP().toString() + "</p>"
                  "<p><a href='/shot.jpg' style='color:#0af'>查看最新快照</a></p>"
                  "</body></html>";
    server.send(200, "text/html", html);
}

/* 手动快照 */
void handleSnapshot()
{
    captureSnapshot();
    if (g_last_snapshot) {
        WiFiClient *client = server.client();
        if (!client) return;
        client->write("HTTP/1.1 200 OK\r\n");
        client->write("Content-Type: image/jpeg\r\n");
        client->write("Content-Length: ");
        client->write(String(g_last_snapshot->len).c_str());
        client->write("\r\nConnection: close\r\n\r\n");
        client->write((const char *)g_last_snapshot->buf, g_last_snapshot->len);
    } else {
        server.send(500, "text/plain", "NO_DATA");
    }
}

/* 查看全局快照（只读，不触发新拍照） */
void handleShotJpg()
{
    if (g_last_snapshot) {
        WiFiClient *client = server.client();
        if (!client) return;
        client->write("HTTP/1.1 200 OK\r\n");
        client->write("Content-Type: image/jpeg\r\n");
        client->write("Content-Length: ");
        client->write(String(g_last_snapshot->len).c_str());
        client->write("\r\nConnection: close\r\n\r\n");
        client->write((const char *)g_last_snapshot->buf, g_last_snapshot->len);
    } else {
        server.send(503, "text/plain", "NO_SNAPSHOT_YET");
    }
}

/* MJPEG 实时流 */
void handleStream()
{
    WiFiClient *client = server.client();
    if (!client || !client->connected()) return;

    client->write("HTTP/1.1 200 OK\r\n");
    client->write("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
    client->write("Cache-Control: no-cache\r\nConnection: close\r\n\r\n");

    unsigned long start = millis();
    while (client->connected() && (millis() - start < 30000)) {  /* 最长 30s */
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) break;

        client->write("--frame\r\n");
        client->write("Content-Type: image/jpeg\r\n");
        client->write("Content-Length: ");
        client->write(String(fb->len).c_str());
        client->write("\r\n\r\n");
        client->write((const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        delay(50);
    }
}

/* 404 */
void handleNotFound()
{
    server.send(404, "text/plain", "Not Found");
}

/* ================================================================
 *  串口命令处理
 * ================================================================ */
void handleSerialCmd()
{
    static String cmd_buf = "";
    if (!Serial.available()) return;

    char c = Serial.read();
    if (c == '\n' || c == '\r') {
        if (cmd_buf.length() == 0) { cmd_buf = ""; return; }

        cmd_buf.trim();

        if (cmd_buf == "STATUS") {
            Serial.println("READY");
        } else if (cmd_buf == "CAPTURE") {
            if (captureSnapshot()) {
                Serial.print("OK http://");
                Serial.print(WiFi.localIP());
                Serial.println("/shot.jpg");
            } else {
                Serial.println("BUSY");
            }
        } else if (cmd_buf == "PING") {
            Serial.println("PONG");
        } else {
            Serial.println("ERR_UNKNOWN");
        }

        cmd_buf = "";
    } else if (cmd_buf.length() < 32) {
        cmd_buf += c;
    }
}

/* ================================================================
 *  setup / loop
 * ================================================================ */

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println("\r\n=== ESP32-CAM Booting ===");

    /* ---- 1. 摄像头初始化 ---- */
    camera_config_t config = {
        .pin_pwdn  = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk  = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_d7    = Y9_GPIO_NUM,
        .pin_d6    = Y8_GPIO_NUM,
        .pin_d5    = Y7_GPIO_NUM,
        .pin_d4    = Y6_GPIO_NUM,
        .pin_d3    = Y5_GPIO_NUM,
        .pin_d2    = Y4_GPIO_NUM,
        .pin_d1    = Y3_GPIO_NUM,
        .pin_d0    = Y2_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href  = HREF_GPIO_NUM,
        .pin_pclk  = PCLK_GPIO_NUM,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_QVGA,    /* 320x240，够清楚又省带宽 */
        .jpeg_quality = 12,                 /* 1-63, 数值越高质量越好 */
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM /* 有 PSRAM 就用 PSRAM */
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("ERR: camera init failed (0x%x)\r\n", err);
        /* 红灯常亮提示 */
        pinMode(33, OUTPUT); digitalWrite(33, LOW);
        delay(5000);
        ESP.restart();
    }
    Serial.println("[CAM] OK");

    /* ---- 2. WiFi 连接 ---- */
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("[WiFi] connecting to '%s'", WIFI_SSID);
    int wifi_timeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_timeout < 40) {
        delay(500);
        Serial.print(".");
        wifi_timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\r\n[WiFi] OK, IP=%s\r\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\r\n[WiFi] FAIL! Using fallback AP...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("SmartGuard-CAM", "12345678");
        Serial.printf("[AP] IP=%s\r\n", WiFi.softAPIP().toString().c_str());
    }

    /* ---- 3. HTTP Server ---- */
    server.on("/",          handleIndex);
    server.on("/stream",    HTTP_GET, handleStream);
    server.on("/shot.jpg",  handleShotJpg);
    server.on("/snapshot",  handleSnapshot);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("[HTTP] server started on :80");

    /* ---- 4. 拍一张初始快照 ---- */
    captureSnapshot();
    Serial.println("[CAM] ready, waiting for STM32 commands...\r\n");
    Serial.println("READY");  /* 告诉 STM32 我们 ready 了 */
}

void loop()
{
    /* HTTP 请求处理 */
    server.handleClient();

    /* 串口命令（STM32 发过来的） */
    handleSerialCmd();
}
