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
 *       GET /            → 首页（移动端可看实时视频）
 *       GET /stream      → MJPEG 实时视频流
 *       GET /shot.jpg    → 最新一张抓拍（等价 ?i=0）
 *       GET /shot.jpg?i=N→ 往前第 N 张抓拍（0=最新，最多 5；越界 404）
 *       GET /snapshot    → 手动拍一张返回 JPEG
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
const char *WIFI_SSID = "REDACTED_WIFI_SSID";
const char *WIFI_PASS = "REDACTED_WIFI_PASSWORD";

/* 固定 IP（热点网段 192.168.137.x，热点不会改它） */
IPAddress LOCAL_IP(192, 168, 137, 250);
IPAddress GATEWAY (192, 168, 137, 1);
IPAddress SUBNET  (255, 255, 255, 0);
IPAddress DNS     (192, 168, 137, 1);

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

/* ============ 抓拍历史环（最近 6 张，JPEG 字节存 PSRAM） ============
   只留最后一张的缺陷：入侵者走过传感器只需 1-2 秒，"最后一张"往往是
   人离开后的空场景，真正拍到人的是中间某张。环形保留最近 6 张，
   Web 端以胶片条展示。JPEG 字节拷出后 fb 立即归还，
   环容量不受 camera fb_count=2 限制（6 张 × ~15KB ≈ 90KB，PSRAM 轻松放下） */
#define SHOT_HISTORY 12   /* 4MB PSRAM：12 张 × ~15KB ≈ 180KB，余量充足 */
uint8_t *g_shots[SHOT_HISTORY] = { NULL };
size_t   g_shot_len[SHOT_HISTORY] = { 0 };
uint32_t g_shot_seq[SHOT_HISTORY] = { 0 };  /* 每张的终身序号（URL 版本号） */
int      g_shot_head = 0;    /* 下一张写入槽位 */
int      g_shot_count = 0;   /* 有效张数（≤ SHOT_HISTORY） */
uint32_t g_seq_next = 1;     /* 下一张照片的序号 */
bool g_snapshot_busy = false;

/* 已连接 IP 的缓存：拍照 URL 用它而不是实时 WiFi.localIP()，
   避免 WiFi 掉线时给 STM32 回 0.0.0.0 */
String g_ip_str = "";

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

    /* JPEG 字节拷入历史环（优先 PSRAM，失败退回 DRAM），随后归还 fb */
    uint8_t *copy = (uint8_t *)ps_malloc(fb->len);
    if (!copy) copy = (uint8_t *)malloc(fb->len);
    if (copy) {
        memcpy(copy, fb->buf, fb->len);
        free(g_shots[g_shot_head]);
        g_shots[g_shot_head] = copy;
        g_shot_len[g_shot_head] = fb->len;
        g_shot_seq[g_shot_head] = g_seq_next++;
        g_shot_head = (g_shot_head + 1) % SHOT_HISTORY;
        if (g_shot_count < SHOT_HISTORY) g_shot_count++;
    } else {
        Serial.println("ERR: no mem for shot history");
    }
    esp_camera_fb_return(fb);

    g_snapshot_busy = false;
    return copy != NULL;
}

/* ================================================================
 *  HTTP 路由
 * ================================================================ */

/* 首页 */
void handleIndex()
{
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width'>"
                  "<title>SmartGuard ESP32-CAM</title></head><body style='background:#1a1a1a;color:#fff'>"
                  "<h2>SmartGuard ESP32-CAM</h2>"
                  "<img src='/stream' style='max-width:100%;border:1px solid #444'>"
                  "<p>IP: " + WiFi.localIP().toString() + "</p>"
                  "<p><a href='/shot.jpg' style='color:#0af'>查看最新快照</a></p>"
                  "</body></html>";
    server.send(200, "text/html; charset=utf-8", html);
}

/* 查看抓拍历史（只读，不触发新拍照）：
     ?s=N  → 按终身序号取（URL 不可变，配合 Cache-Control 长缓存，
             浏览器只下载新照片，旧照片全部缓存命中——胶片条秒开的关键）
     ?i=N  → 按相对位置取（0=最新，兼容旧链路；内容随环顺延变化，不适合缓存）
     无参  → 最新一张 */
void handleShotJpg()
{
    int slot = -1;

    if (server.hasArg("s")) {
        uint32_t want = (uint32_t)server.arg("s").toInt();
        for (int k = 0; k < g_shot_count; k++) {
            int sl = (g_shot_head - 1 - k + 2 * SHOT_HISTORY) % SHOT_HISTORY;
            if (g_shot_seq[sl] == want) { slot = sl; break; }
        }
    } else {
        int idx = server.hasArg("i") ? server.arg("i").toInt() : 0;
        if (idx >= 0 && idx < g_shot_count) {
            slot = (g_shot_head - 1 - idx + 2 * SHOT_HISTORY) % SHOT_HISTORY;
        }
    }

    if (slot < 0) {
        server.send(404, "text/plain", "NO_SUCH_SHOT");
        return;
    }

    NetworkClient client = server.client();
    if (!client) return;
    client.write("HTTP/1.1 200 OK\r\n");
    client.write("Content-Type: image/jpeg\r\n");
    client.write("Access-Control-Allow-Origin: *\r\n");
    /* URL 自带按槽位的版本号（?i=N&t=该张抓拍时刻），内容不变 URL 不变，
       放心长缓存：网页 13 张图只有最新一张在新抓拍后重下，
       其余 12 张全部浏览器缓存秒开（胶片条卡顿的主因就是没有缓存头） */
    client.write("Cache-Control: max-age=3600\r\n");
    client.write("Content-Length: ");
    client.write(String(g_shot_len[slot]).c_str());
    client.write("\r\nConnection: close\r\n\r\n");
    client.write((const char *)g_shots[slot], g_shot_len[slot]);
}

/* 手动快照：拍一张新的，返回最新那张（复用历史环读取） */
void handleSnapshot()
{
    captureSnapshot();
    if (g_shot_count == 0) {
        server.send(500, "text/plain", "NO_DATA");
        return;
    }
    handleShotJpg();
}

/* MJPEG 实时流 */
void handleStream()
{
    NetworkClient client = server.client();
    if (!client || !client.connected()) return;

    client.write("HTTP/1.1 200 OK\r\n");
    client.write("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
    client.write("Cache-Control: no-cache\r\nConnection: close\r\n\r\n");

    unsigned long start = millis();
    while (client.connected() && (millis() - start < 300000)) {  /* 最长 300s */
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) break;

        client.write("--frame\r\n");
        client.write("Content-Type: image/jpeg\r\n");
        client.write("Content-Length: ");
        client.write(String(fb->len).c_str());
        client.write("\r\n\r\n");
        size_t w = client.write((const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        /* 网页切走/关闭标签后 TCP 断开，client.connected() 感知很慢
           （LWIP 要等 FIN 处理），但 write 会立刻失败——靠返回值在
           ~100ms 内退出流循环，释放单客户端服务器，否则 /shot.jpg
           请求全被排队（大图卡在最后一帧、缩略图全空白的根因） */
        if (w != fb->len) break;

        /* 流式循环会独占 loop()：必须在帧间隙处理 STM32 串口命令，
           否则直播期间 CAPTURE/STATUS 被卡住直到流结束 */
        handleSerialCmd();

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
                /* URL 带终身序号：网页据此推算历史环中每张照片的不可变 URL */
                Serial.print("OK http://");
                Serial.print(g_ip_str.length() ? g_ip_str : WiFi.localIP().toString());
                Serial.print("/shot.jpg?s=");
                Serial.println(g_seq_next - 1);
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
    Serial.println("[CAM] fw=20260921c (seq+cache+fast-stream-exit)");  /* 版本标记 */

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
        .xclk_freq_hz = 12000000,   /* 实测对标官方固件:12MHz 流畅(15仍卡);更低主频还降功耗发热 */
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_QVGA,    /* 320x240，够清楚又省带宽 */
        .jpeg_quality = 12,                 /* 1-63, 数值越高质量越好 */
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM, /* 有 PSRAM 就用 PSRAM */
        .grab_mode    = CAMERA_GRAB_LATEST  /* 总取最新帧,不积压,官方流式配置 */
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

    /* ---- 2. WiFi 连接（固定 IP，热点不分配也变不了） ---- */
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                /* 关WiFi省电:消除beacon跳过造成的数百ms延迟抖动,流媒体关键 */
    WiFi.setTxPower(WIFI_POWER_19_5dBm); /* 满功率:独立供电后可承受,视频流更流畅(供电不足时改回 8_5dBm) */
    Serial.println("[PWR] TX=19.5dBm");  /* 版本标记:看到这行说明新固件生效 */
    WiFi.config(LOCAL_IP, GATEWAY, SUBNET, DNS);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("[WiFi] connecting to '%s'", WIFI_SSID);
    int wifi_timeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_timeout < 40) {
        delay(500);
        Serial.print(".");
        wifi_timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        g_ip_str = WiFi.localIP().toString();
        Serial.printf("\r\n[WiFi] OK, IP=%s\r\n", g_ip_str.c_str());
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
    /* 序号随机起点：防止 CAM 重启后 seq 从 1 重复，浏览器缓存里的
       旧 URL(?s=3) 命中新照片（缓存污染） */
    g_seq_next = (esp_random() % 1000000UL) + 1;
    captureSnapshot();
    Serial.println("[CAM] ready, waiting for STM32 commands...\r\n");
    Serial.println("READY");  /* 告诉 STM32 我们 ready 了 */
}

/* WiFi 看门狗：断线自动重连，重连后刷新 IP 缓存 */
static uint32_t g_wifi_check_ms = 0;
void wifiWatchdog()
{
    uint32_t now = millis();
    if (now - g_wifi_check_ms < 10000) return;   /* 每 10s 查一次 */
    g_wifi_check_ms = now;

    if (WiFi.getMode() != WIFI_STA) return;      /* AP fallback 模式不干预 */
    if (WiFi.status() == WL_CONNECTED) {
        g_ip_str = WiFi.localIP().toString();
        return;
    }
    Serial.println("[WiFi] lost, reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void loop()
{
    /* HTTP 请求处理 */
    server.handleClient();

    /* 串口命令（STM32 发过来的） */
    handleSerialCmd();

    /* WiFi 断线重连 */
    wifiWatchdog();
}
