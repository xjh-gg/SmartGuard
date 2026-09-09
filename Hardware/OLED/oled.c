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

void OLED_ShowString(uint8_t x, uint8_t y, char *str)
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

/* ============ 8x8 小图标数据 (SSD1306 page 模式: 8字节=8列, bit0=top) ============ */
static const uint8_t icon_light[8]  = {0x00, 0x0C, 0x5E, 0x7F, 0x7F, 0x5E, 0x0C, 0x00}; /* 💡灯泡 */
static const uint8_t icon_dist[8]   = {0x18, 0x24, 0x42, 0x99, 0x99, 0x42, 0x24, 0x18}; /* 📡超声 */
static const uint8_t icon_temp[8]  = {0x00, 0x40, 0xFF, 0xFF, 0xF0, 0xE0, 0xE0, 0x00}; /* 🌡温度 */
static const uint8_t icon_drop[8]  = {0x00, 0x70, 0xFC, 0xFF, 0xFF, 0xFC, 0x70, 0x00}; /* 💧湿度 */
static const uint8_t icon_home[8]   = {0x08, 0x0C, 0xFE, 0x1F, 0x1F, 0xFE, 0x0C, 0x08}; /* 🏠房子 */
static const uint8_t icon_person[8] = {0x80, 0x48, 0x2C, 0x7F, 0x7F, 0x2C, 0x48, 0x80}; /* 👤人 */
static const uint8_t icon_moon[8]   = {0x3C, 0x7E, 0xFF, 0xE7, 0xC3, 0x42, 0x00, 0x00}; /* 🌙月亮 */
static const uint8_t icon_warn[8]   = {0x10, 0x18, 0x1C, 0x1E, 0xFF, 0xFD, 0x18, 0x10}; /* ⚠警告 */
static const uint8_t icon_ok[8]     = {0x40, 0x60, 0x30, 0x18, 0x0C, 0x00, 0x00, 0x00}; /* ✓对勾 */

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

/* ============ SmartGuard 主界面 v3 (纯文字版, 无图标无分隔线) ============

   128x64 OLED (8x16 字体 = 2 page/行):
   ┌─────────────────────────────────┐
   │ HOME        v0.4  [AI:ON]       │  page 0 : 模式 + 版本 + AI 状态
   │ L:2018      D:25.3cm            │  page 2 : 光照 + 距离
   │ T:25.0C     H:50.0%             │  page 4 : 温度 + 湿度
   │ Status: OK  /  !! INTRUSION !!  │  page 6 : 告警
   └─────────────────────────────────┘
*/
void OLED_ShowHomeScreen(Sensor_Data_t *data, Guard_Mode_t mode, Alert_Type_t alert)
{
    char buf[20];

    /* ============ 每行画之前先清除对应 page ============ */

    /* 行 0: 模式 + AI 状态 (page 0 和 page 1, 宽度 128) */
    OLED_ClearArea(0, 0, 128);
    OLED_ClearArea(0, 1, 128);

    switch (mode) {
        case MODE_HOME:  OLED_ShowString(0, 0, "HOME");  break;
        case MODE_AWAY:  OLED_ShowString(0, 0, "AWAY");  break;
        case MODE_SLEEP: OLED_ShowString(0, 0, "SLEEP"); break;
        default:         OLED_ShowString(0, 0, "???");   break;
    }

#if defined(USE_ML) && USE_ML
    OLED_ShowString(64, 0, "AI:ON");
#else
    OLED_ShowString(64, 0, "AI:OFF");
#endif

    /* 行 2: 光照 + 距离 */
    OLED_ClearArea(0, 2, 128);
    OLED_ClearArea(0, 3, 128);

    snprintf(buf, sizeof(buf), "L:%d", data->light_adc);
    OLED_ShowString(0, 2, buf);

    snprintf(buf, sizeof(buf), "D:%.1fcm", data->distance);
    OLED_ShowString(64, 2, buf);

    /* 行 4: 温度 + 湿度 */
    OLED_ClearArea(0, 4, 128);
    OLED_ClearArea(0, 5, 128);

    snprintf(buf, sizeof(buf), "T:%.1fC", data->temperature);
    OLED_ShowString(0, 4, buf);

    snprintf(buf, sizeof(buf), "H:%.1f%%", data->humidity);
    OLED_ShowString(64, 4, buf);

    /* 行 6: 状态 */
    OLED_ClearArea(0, 6, 128);
    OLED_ClearArea(0, 7, 128);

    if (alert != ALERT_NONE) {
        switch (alert) {
            case ALERT_INTRUSION:
                OLED_ShowString(0, 6, "!! INTRUSION !!");
                break;
            case ALERT_FIRE:
                OLED_ShowString(0, 6, "!! FIRE ALERT !!");
                break;
            case ALERT_ENV_ABNORMAL:
                OLED_ShowString(0, 6, "!! ENV WARN !!");
                break;
            case ALERT_TAMPER:
                OLED_ShowString(0, 6, "!! TAMPER !!");
                break;
            default:
                OLED_ShowString(0, 6, "!! ALERT !!");
                break;
        }
    } else {
        OLED_ShowString(0, 6, "Status: OK");
    }
}
