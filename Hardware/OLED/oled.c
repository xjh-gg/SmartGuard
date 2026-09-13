/**
  ******************************************************************************
  * @file    oled.c
  * @brief   0.96寸 OLED SSD1306 SPI 驱动（软件模拟 SPI）
  *          基于卖家 0.96寸OLED参考源码 + v9 验证版
  *
  * 引脚: PB0(SCLK) PF11(MOSI) PB1(DC) PC13(CS1)
  * 接线: TFTLCD 触摸 SPI 区 (T_SCK/PB0, T_MOSI/PF11, T_PEN/PB1, T_CS/PC13)
  * 外部: CS2 必须接 VCC 禁用 GT20L1616 字库芯片！
  ******************************************************************************
  */
#include "oled.h"
#include "delay.h"
#include "oledfont.h"
#include <stdio.h>
#include <string.h>

/* ============ 前置声明 ============ */
static uint32_t oled_pow(uint8_t m, uint8_t n);

/* ============ SPI 软件模拟发送 1 字节 ============
   SSD1306 SPI Mode 0: CLK idle LOW, 上升沿采样
   ⚠️ F407 主频 168MHz，必须用 delay_us() 放慢时序！
      用 __nop() 会太快，OLED 采样不到 = 黑屏 */
static void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    uint8_t i;

    if (cmd) OLED_DC_Set();
    else     OLED_DC_Clr();

    OLED_CS_Clr();
    delay_us(1);

    for (i = 0; i < 8; i++)
    {
        /* SPI Mode 0: CLK idle LOW, rising edge sample */
        OLED_SCLK_Clr();
        delay_us(1);

        if (dat & 0x80) OLED_SDIN_Set();
        else            OLED_SDIN_Clr();
        delay_us(1);

        OLED_SCLK_Set();
        delay_us(1);

        dat <<= 1;
    }

    OLED_SCLK_Clr();
    delay_us(1);
    OLED_CS_Set();
    OLED_DC_Set();
}

/* ============ OLED 基础命令 ============ */

static void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xB0 + y, OLED_CMD);          /* page 地址 */
    OLED_WR_Byte(((x & 0xF0) >> 4) | 0x10, OLED_CMD);  /* 列高 4 bit */
    OLED_WR_Byte(x & 0x0F, OLED_CMD);          /* 列低 4 bit (无偏移!) */
}

void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xB0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(0, OLED_DATA);
    }
}

/* ============ 显示字符 (8x16) ============ */
static void OLED_ShowChar_raw(uint8_t x, uint8_t y, uint8_t chr)
{
    uint8_t c, i;

    c = chr - ' ';
    if (x > Max_Column - 1)
    {
        x = 0;
        y += 2;
    }

    OLED_Set_Pos(x, y);
    for (i = 0; i < 8; i++)
        OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);

    OLED_Set_Pos(x, y + 1);
    for (i = 0; i < 8; i++)
        OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    uint8_t j = 0;
    while (str[j] != '\0')
    {
        OLED_ShowChar_raw(x, y, str[j]);
        x += 8;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar_raw(x + (size / 2) * t, y, ' ');
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar_raw(x + (size / 2) * t, y, temp + '0');
    }
}

static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/* ============ 清局部区域 ============ */
void OLED_ClearArea(uint8_t x, uint8_t page, uint8_t width)
{
    uint8_t i;
    OLED_Set_Pos(x, page);
    for (i = 0; i < width; i++)
        OLED_WR_Byte(0, OLED_DATA);
}

/* ============ 画 1 像素横线 (page 内最底行, bit7=1) ============ */
void OLED_DrawHLine(uint8_t x, uint8_t page, uint8_t len)
{
    uint8_t i;
    OLED_Set_Pos(x, page);
    for (i = 0; i < len; i++)
        OLED_WR_Byte(0x80, OLED_DATA);   /* bit7 = 最下面 1 像素 */
}

/* ============ 画竖线 (跨 page) ============ */
void OLED_DrawVLine(uint8_t x, uint8_t page_start, uint8_t page_end)
{
    uint8_t p;
    for (p = page_start; p <= page_end; p++)
    {
        OLED_Set_Pos(x, p);
        OLED_WR_Byte(0xFF, OLED_DATA);
    }
}

/* ============ 显示浮点数 ============ */
void OLED_ShowFloat(uint8_t x, uint8_t y, float val, uint8_t int_len, uint8_t dec_len)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%.*f", dec_len, val);
    OLED_ShowString(x, y, buf);
}

/* ============ 显示 8x8 图标 (page 模式, 8字节=8列) ============ */
void OLED_ShowIcon8x8(uint8_t x, uint8_t page, const uint8_t *icon)
{
    uint8_t i;
    OLED_Set_Pos(x, page);
    for (i = 0; i < 8; i++)
        OLED_WR_Byte(icon[i], OLED_DATA);
}

/* ============ 显示 16x16 汉字 (Hzk[] 已是 SSD1306 Page 友好格式，直接写) ============
   Hzk 格式: 每字 32 字节
     Hzk[no][0..15]  = page 0 (上半页 8 行) 的 16 列
     Hzk[no][16..31] = page 1 (下半页 8 行) 的 16 列
   每字节 = 一列中 8 个垂直像素, bit0=顶行, bit7=底行
   与 SSD1306 page 格式完全一致，无需转置
   （厂商字库是 [N*2][16] 结构，我们重排成 [N][32]，数据布局不变） */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no)
{
    uint8_t t;

    /* 标准 16x16 渲染（Hzk[][32]：前 16 字节=上半页，后 16 字节=下半页） */
    OLED_Set_Pos(x, y);
    for (t = 0; t < 16; t++)
        OLED_WR_Byte(Hzk[no][t], OLED_DATA);

    OLED_Set_Pos(x, y + 1);   /* 切到下半页 */
    for (t = 0; t < 16; t++)
        OLED_WR_Byte(Hzk[no][16 + t], OLED_DATA);
}

/* ============ 汉字索引（与 oledfont.h 的 Hzk[] 内容严格对应） ============
   Hzk[0..6]  中景园电子科技  卖家例程原始字模，SmartGuard 未使用
   Hzk[7..28] tools/gen_chinese_font.py 取模所得 22 字
   注意：取模串里「度」出现三次，Hzk[10]、Hzk[12]、Hzk[14] 均为「度」，
   所以从「空」起下标比字面顺序大 1。改动字库后必须重跑
   tools/check_oled_font.py 校验，不要凭顺序推算下标。 */
#define HZ_GUANG    7   /* 光 */
#define HZ_ZHAO     8   /* 照 */
#define HZ_WEN      9   /* 温 */
#define HZ_DU      10   /* 度 */
#define HZ_SHI     11   /* 湿 */
#define HZ_KONG    13   /* 空 (Hzk[12] 是重复的「度」占位) */
#define HZ_QI      14   /* 气 */
#define HZ_LIANG   15   /* 良 */
#define HZ_HAO     16   /* 好 */
#define HZ_ZHONG   17   /* 中 */
#define HZ_CHA     18   /* 差 */
#define HZ_XUN     19   /* 巡 */
#define HZ_LUO     20   /* 逻 */
#define HZ_RU      21   /* 入 */
#define HZ_HUO     22   /* 火 */
#define HZ_ZAI     23   /* 灾 */
#define HZ_GAO     24   /* 告 */
#define HZ_JING    25   /* 警 */
#define HZ_BEI     26   /* 备 */
#define HZ_CHAI    27   /* 拆 */
#define HZ_QIAN    28   /* 前 */

/* ============ 辅助: 光照 ADC → 百分比 ============ */
static uint8_t light_to_pct(uint16_t adc)
{
    int pct;
    if (adc >= 4000) return 0;
    if (adc <= 200)  return 100;
    pct = (int)(100 - ((unsigned long)(adc - 200) * 100 / 3800));
    return (uint8_t)pct;
}

/* ============ 辅助: MQ-2 ADC → 空气质量英文 ============ */
static const char *air_quality_str(uint16_t adc)
{
    if (adc < 1000) return "GOOD";
    if (adc < 2000) return "MID";
    return "BAD";
}

/* ============ 8x8 小图标数据 (SSD1306 page 模式: 8字节=8列, bit0=top) ============ */
#if 0  /* 预留：OLED 图标界面升级用，当前未使用 */
static const uint8_t icon_light[8]  = {0x00, 0x0C, 0x5E, 0x7F, 0x7F, 0x5E, 0x0C, 0x00};  /* light icon */
static const uint8_t icon_dist[8]   = {0x18, 0x24, 0x42, 0x99, 0x99, 0x42, 0x24, 0x18};  /* distance/ultrasonic icon */
static const uint8_t icon_temp[8]   = {0x00, 0x40, 0xFF, 0xFF, 0xF0, 0xE0, 0xE0, 0x00};  /* temperature icon */
static const uint8_t icon_drop[8]   = {0x00, 0x70, 0xFC, 0xFF, 0xFF, 0xFC, 0x70, 0x00};  /* humidity/drop icon */
static const uint8_t icon_home[8]   = {0x08, 0x0C, 0xFE, 0x1F, 0x1F, 0xFE, 0x0C, 0x08};  /* home icon */
static const uint8_t icon_person[8] = {0x80, 0x48, 0x2C, 0x7F, 0x7F, 0x2C, 0x48, 0x80};  /* person icon */
static const uint8_t icon_moon[8]   = {0x3C, 0x7E, 0xFF, 0xE7, 0xC3, 0x42, 0x00, 0x00};  /* moon/sleep icon */
static const uint8_t icon_warn[8]   = {0x10, 0x18, 0x1C, 0x1E, 0xFF, 0xFD, 0x18, 0x10};  /* warning icon */
static const uint8_t icon_ok[8]     = {0x40, 0x60, 0x30, 0x18, 0x0C, 0x00, 0x00, 0x00};  /* OK/check icon */
#endif

/* ============ OLED 初始化 ============ */
void OLED_Init(void)
{
    GPIO_InitTypeDef g;

    /* 使能 GPIOB / GPIOC / GPIOF 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOF, ENABLE);

    /* PB0(SCLK) + PB1(DC) */
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &g);

    /* PF11(MOSI) */
    g.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOF, &g);

    /* PC13(CS1) — F4 上 PC13 是 RTC 引脚，驱动能力弱 (3mA)，降速 */
    g.GPIO_Pin   = GPIO_Pin_13;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &g);

    /* 初始电平 (SPI Mode 0: CLK idle LOW) */
    OLED_CS_Set();
    OLED_SCLK_Clr();
    OLED_DC_Set();
    OLED_SDIN_Set();

    /* 等 SSD1306 内部上电复位 */
    delay_ms(500);

    /* SSD1306 初始化命令序列 */
    OLED_WR_Byte(0xAE, OLED_CMD); /* display off */
    OLED_WR_Byte(0x20, OLED_CMD); /* memory addressing mode */
    OLED_WR_Byte(0x10, OLED_CMD); /* page mode */
    OLED_WR_Byte(0xB0, OLED_CMD); /* page start */
    OLED_WR_Byte(0xC8, OLED_CMD); /* COM scan remap */
    OLED_WR_Byte(0x00, OLED_CMD); /* column low */
    OLED_WR_Byte(0x10, OLED_CMD); /* column high */
    OLED_WR_Byte(0x40, OLED_CMD); /* display start line */
    OLED_WR_Byte(0x81, OLED_CMD); /* contrast */
    OLED_WR_Byte(0xFF, OLED_CMD); /* max */
    OLED_WR_Byte(0xA1, OLED_CMD); /* segment remap */
    OLED_WR_Byte(0xA6, OLED_CMD); /* normal display */
    OLED_WR_Byte(0xA8, OLED_CMD); /* multiplex ratio */
    OLED_WR_Byte(0x3F, OLED_CMD); /* 64 COM */
    OLED_WR_Byte(0xA4, OLED_CMD); /* follows RAM */
    OLED_WR_Byte(0xD3, OLED_CMD); /* display offset */
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD); /* clock divide */
    OLED_WR_Byte(0xF0, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD); /* pre-charge */
    OLED_WR_Byte(0x22, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD); /* COM pins */
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); /* vcomh deselect */
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD); /* charge pump */
    OLED_WR_Byte(0x14, OLED_CMD); /* enable */
    OLED_WR_Byte(0xAF, OLED_CMD); /* display ON */

    delay_ms(100);
    OLED_Clear();
}

/* ============ SmartGuard 主界面 v4 (纯英文 4 行, 无图标无分隔线) ============

   128x64 OLED (8x16 字体 = 2 page/行, 16 字符/行):
   +--------------------------------+
   | AW 12:34:56 W+M+               |  page 0-1 : 模式 时间 WiFi MQTT
   | T26C H39%                      |  page 2-3 : 温度 湿度
   | L50% D140cm                    |  page 4-5 : 光照 距离
   | MQ2189 A GOOD                  |  page 6-7 : MQ2 空气质量
   +--------------------------------+

   行宽硬约束: 字符数 <= 16 且 x + 字符数*8 <= 128。
   OLED_ShowString 在 x > 120 时折到 page+2，超出会覆盖下一行，
   改动任一行文案后必须重新复核宽度。

   告警页 OLED_ShowAlertPage 复用 page 0/2/4/6 同一套行位置。
*/

void OLED_ShowMode(Guard_Mode_t mode)
{
    const char *mode_str;

    OLED_ClearArea(0, 0, 40);
    OLED_ClearArea(0, 1, 40);

    switch (mode) {
        case MODE_HOME:  mode_str = "HOME";  break;
        case MODE_AWAY:  mode_str = "AWAY";  break;
        case MODE_SLEEP: mode_str = "SLEEP"; break;
        default:         mode_str = "???";   break;
    }
    OLED_ShowString(0, 0, mode_str);
}

void OLED_ShowStatusBar(Guard_Mode_t mode, const char *time_str, const char *wifi_status)
{
    OLED_ClearArea(0, 0, 128);
    OLED_ClearArea(0, 1, 128);

    OLED_ShowMode(mode);
    OLED_ShowString(48, 0, time_str);
    OLED_ShowString(112, 0, wifi_status);
}

void OLED_ShowHomeScreen(Sensor_Data_t *data, Guard_Mode_t mode,
                         const char *time_str, const char *wifi_status,
                         const char *mqtt_status)
{
    /* 模式 2 字母缩写，下标与 Guard_Mode_t 枚举顺序（HOME/AWAY/SLEEP）严格对应 */
    static const char *const mode_abbr[3] = { "HO", "AW", "SL" };
    const char *mtxt = ((unsigned)mode < 3u) ? mode_abbr[mode] : "??";
    char buf[24];

    /* 行 1 (page 0-1): 模式 · 时间 · WiFi · MQTT —— 恒定 16 字符 */
    OLED_ClearArea(0, 0, 128);
    OLED_ClearArea(0, 1, 128);
    snprintf(buf, sizeof(buf), "%s %s %s%s", mtxt, time_str, wifi_status, mqtt_status);
    OLED_ShowString(0, 0, buf);

    /* 行 2 (page 2-3): 温度整数 + 湿度整数 */
    OLED_ClearArea(0, 2, 128);
    OLED_ClearArea(0, 3, 128);
    snprintf(buf, sizeof(buf), "T%dC H%d%%", (int)data->temperature, (int)data->humidity);
    OLED_ShowString(0, 2, buf);

    /* 行 3 (page 4-5): 光照百分比 + 距离 cm */
    OLED_ClearArea(0, 4, 128);
    OLED_ClearArea(0, 5, 128);
    snprintf(buf, sizeof(buf), "L%u%% D%dcm",
             (unsigned)light_to_pct(data->light_adc), (int)data->distance);
    OLED_ShowString(0, 4, buf);

    /* 行 4 (page 6-7): MQ2 原始值 + 空气质量 */
    OLED_ClearArea(0, 6, 128);
    OLED_ClearArea(0, 7, 128);
    snprintf(buf, sizeof(buf), "MQ2%u A %s",
             (unsigned)data->mq2_adc, air_quality_str(data->mq2_adc));
    OLED_ShowString(0, 6, buf);
}

void OLED_ShowAlertPage(Sensor_Data_t *data, Alert_Type_t alert, const char *time_str, uint8_t cam_ok)
{
    char buf[32];

    OLED_ClearArea(0, 0, 128); OLED_ClearArea(0, 1, 128);
    OLED_ClearArea(0, 2, 128); OLED_ClearArea(0, 3, 128);
    OLED_ClearArea(0, 4, 128); OLED_ClearArea(0, 5, 128);
    OLED_ClearArea(0, 6, 128); OLED_ClearArea(0, 7, 128);

    switch (alert) {
        case ALERT_INTRUSION:
            OLED_ShowString(8, 0, "!! INTRUSION !!");   /* 15 字符 -> 8..128 */
            snprintf(buf, sizeof(buf), "DIST: %.1fcm", data->distance);
            OLED_ShowString(0, 2, buf);
            OLED_ShowString(0, 4, cam_ok ? "CAM:  OK" : "CAM:  --");
            snprintf(buf, sizeof(buf), "CHECK %s", time_str);
            OLED_ShowString(0, 6, buf);
            break;

        case ALERT_FIRE:
            OLED_ShowString(8, 0, "   !! FIRE !!");     /* 13 字符 -> 8..112 */
            snprintf(buf, sizeof(buf), "MQ2:%u T:%dC",
                     (unsigned)data->mq2_adc, (int)data->temperature);
            OLED_ShowString(0, 2, buf);
            OLED_ShowString(0, 4, cam_ok ? "CAM:  OK" : "CAM:  --");
            snprintf(buf, sizeof(buf), "EVAC %s", time_str);
            OLED_ShowString(0, 6, buf);
            break;

        case ALERT_ENV_ABNORMAL:
            OLED_ShowString(8, 0, "  AIR BAD");         /* 9 字符 -> 8..80 */
            snprintf(buf, sizeof(buf), "MQ2:%u", (unsigned)data->mq2_adc);
            OLED_ShowString(0, 2, buf);
            OLED_ShowString(0, 4, "CHECK NOW");
            OLED_ShowString(0, 6, time_str);
            break;

        case ALERT_TAMPER:
            OLED_ShowString(8, 0, "  TAMPER");          /* 8 字符 -> 8..72 */
            OLED_ShowString(0, 2, "DO NOT MOVE");
            OLED_ShowString(0, 4, "CHECK NOW");
            OLED_ShowString(0, 6, time_str);
            break;

        default:
            OLED_ShowString(0, 0, "!! ALERT !!");
            OLED_ShowString(0, 6, time_str);
            break;
    }
}

void OLED_ShowBootProgress(uint8_t percent, const char *status)
{
    char bar[16];
    uint8_t filled, i;

    if (percent > 100) percent = 100;
    filled = percent / 10;

    bar[0] = '[';
    for (i = 0; i < 10; i++) {
        bar[1 + i] = (i < filled) ? '=' : ' ';
    }
    bar[11] = ']';
    snprintf(bar + 12, 4, "%-3u", (unsigned)percent);
    bar[15] = '\0';

    OLED_ShowString(0, 0, "SmartGuard v0.4");
    OLED_ShowString(8, 1, "SECURITY AGENT");   /* 新增 page 1, 14 字符 -> 8..120 居中 */
    OLED_ShowString(0, 3, bar);
    OLED_ShowString(0, 5, status);
}
