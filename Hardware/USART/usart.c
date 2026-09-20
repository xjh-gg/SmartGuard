/**
  ******************************************************************************
  * @file    usart.c
  * @brief   串口驱动实现（USART1）
  *
  * 功能：
  *   1. USART1 初始化：PA9(TX) + PA10(RX)，AF7 复用，可设波特率
  *   2. 发送：SendByte / SendString / Printf（vsprintf 格式化）
  *   3. 接收：中断方式（RXNE），环形缓冲区缓存，GetByte 非阻塞读取
  *
  * 发送策略：
  *   轮询 TXE 标志（发送数据寄存器空）+ TC 标志（发送完成），
  *   简单可靠，适合教学示例。实际工程大数据量可改 DMA。
  *
  * 接收策略：
  *   RXNE 中断把字节写入环形缓冲区，主循环用 GetByte 取出。
  *   缓冲区满时丢弃新数据（教学简化，实际可做流控）。
  ******************************************************************************
  */
#include "usart.h"

/* 发送等待上限：外设/时钟异常时防止主循环永久卡死（P3-#3）。
   115200bps 单字节 ~87us，100000 次迭代远超 ms 级，正常路径零影响 */
#define USART_WAIT_MAX  100000U
#define USART_WAIT_FLAG(uart, flag)                               \
    do {                                                          \
        uint32_t _to = USART_WAIT_MAX;                            \
        while (USART_GetFlagStatus((uart), (flag)) == RESET)     \
        {                                                         \
            if (--_to == 0) break;                                \
        }                                                         \
    } while (0)

/* ========== USART1 环形缓冲区 ========== */
static volatile uint8_t  usart1_rx_buf[RX_BUF_SIZE];
static volatile uint16_t usart1_rx_head = 0;
static volatile uint16_t usart1_rx_tail = 0;

/* ========== USART2 环形缓冲区 ========== */
static volatile uint8_t  usart2_rx_buf[RX_BUF_SIZE];
static volatile uint16_t usart2_rx_head = 0;
static volatile uint16_t usart2_rx_tail = 0;

/* Printf 临时缓冲区（每个串口独立，避免并发调用时数据交叉） */
static char printf_buf1[128];
static char printf_buf2[128];
// static char printf_buf3[128];  /* 已删除：USART3 未使用 */

/**
  * @brief  初始化 USART1
  * @param  baud: 波特率，如 115200
  *   - 开 GPIOA + USART1 时钟
  *   - PA9 复用 AF7 做 TX（推挽），PA10 复用 AF7 做 RX（浮空）
  *   - 8N1 无硬件流控，使能 RXNE 中断 + NVIC
  */
void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    USART_InitTypeDef   USART_InitStruct;
    NVIC_InitTypeDef    NVIC_InitStruct;

    /* ---- 开时钟：USART1 在 APB2，GPIOA 在 AHB1 ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* ---- PA9(TX) + PA10(RX) 复用 USART1 ---- */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     /* TX 推挽 */
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;      /* RX 上拉（空闲高电平） */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = USART1_TX_PIN | USART1_RX_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(USART1_TX_PORT, USART1_TX_SRC, USART1_AF);
    GPIO_PinAFConfig(USART1_RX_PORT, USART1_RX_SRC, USART1_AF);

    /* ---- USART1 参数：波特率/8位/1停止位/无校验/收发使能 ---- */
    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);

    /* ---- 开 RXNE 中断：收到数据就触发 ---- */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ClearFlag(USART1, USART_FLAG_TC);          /* 清除发送完成标志 */
    USART_Cmd(USART1, ENABLE);

    /* ---- NVIC：USART1 全局中断（在启动文件里是 USART1_IRQn） ---- */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;   /* 抢占优先级 1 */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;   /* 子优先级 1 */
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/**
  * @brief  发送一个字节（阻塞等待 TXE）
  * @param  data: 要发送的字节
  */
void USART1_SendByte(uint8_t data)
{
    USART_SendData(USART1, (uint16_t)data);
    USART_WAIT_FLAG(USART1, USART_FLAG_TXE);
}

/**
  * @brief  发送字符串
  * @param  str: 以 '\0' 结尾的字符串
  */
void USART1_SendString(const char *str)
{
    while (*str != '\0')
    {
        USART1_SendByte((uint8_t)*str);
        str++;
    }
    /* 等最后一个字节真正发完（TC=发送完成） */
    USART_WAIT_FLAG(USART1, USART_FLAG_TC);
}

/**
  * @brief  格式化发送（用法同 printf）
  * @param  fmt: 格式字符串
  *   内部用 vsnprintf 格式化到 printf_buf 后逐字节发送
  *   注意：在嵌入式环境 vsnprintf 需要 --use_micro_lib 或完整 retarget
  */
void USART1_Printf(const char *fmt, ...)
{
    va_list ap;
    int len, i;

    va_start(ap, fmt);
    len = vsnprintf(printf_buf1, sizeof(printf_buf1), fmt, ap);
    va_end(ap);

    if (len < 0) return;
    if (len >= (int)sizeof(printf_buf1)) len = sizeof(printf_buf1) - 1;

    for (i = 0; i < len; i++)
    {
        USART1_SendByte((uint8_t)printf_buf1[i]);
    }
    USART_WAIT_FLAG(USART1, USART_FLAG_TC);
}

/**
  * @brief  非阻塞读取一字节
  * @param  data: 输出读取到的字节
  * @retval 1 = 成功读到数据，0 = 缓冲区空
  */
uint8_t USART1_GetByte(uint8_t *data)
{
    if (usart1_rx_head != usart1_rx_tail)
    {
        *data = usart1_rx_buf[usart1_rx_tail];
        usart1_rx_tail = (usart1_rx_tail + 1) % RX_BUF_SIZE;
        return 1;
    }
    return 0;
}

/**
  * @brief  获取接收缓冲区中未读字节数
  * @retval 当前可读字节数
  */
uint16_t USART1_GetRxCount(void)
{
    return (usart1_rx_head - usart1_rx_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

/**
  * @brief  清空接收缓冲区
  */
void USART1_FlushRx(void)
{
    usart1_rx_tail = usart1_rx_head;
}

/* ========================================================================== */
/* 以下函数供 stm32f4xx_it.c 的 USART1_IRQHandler 调用                       */
/* ========================================================================== */

/**
  * @brief  串口接收中断服务函数（由 USART1_IRQHandler 调用）
  *         读取 USART1->DR，写入环形缓冲区
  *         缓冲区满时丢弃新字节（覆盖最新的）
  */
void USART1_RxHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART1);    /* 读 DR 自动清 RXNE */

        uint16_t next = (usart1_rx_head + 1) % RX_BUF_SIZE;
        if (next != usart1_rx_tail)
        {
            usart1_rx_buf[usart1_rx_head] = ch;
            usart1_rx_head = next;
        }
        /* next == usart1_rx_tail 表示缓冲区满，丢弃当前字节 */
    }

    /* 处理溢出错误（ORE）：读 SR 再读 DR 清除 */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART1->SR;
        (void)USART1->DR;
    }
}

/* ========================================================================== */
/* USART2 驱动实现                                                            */
/*   与 USART1 完全独立，使用独立的 GPIO、NVIC、环形缓冲区                     */
/* ========================================================================== */

/**
  * @brief  初始化 USART2
  * @param  baud: 波特率，如 9600
  *   - 开 GPIOA + USART2 时钟
  *   - PA2 复用 AF7 做 TX（推挽），PA3 复用 AF7 做 RX（浮空）
  *   - 8N1 无硬件流控，使能 RXNE 中断 + NVIC
  */
void USART2_Init(uint32_t baud)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    USART_InitTypeDef   USART_InitStruct;
    NVIC_InitTypeDef    NVIC_InitStruct;

    /* ---- 开时钟：USART2 在 APB1，GPIOA 在 AHB1 ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* ---- PA2(TX) + PA3(RX) 复用 USART2 ---- */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = USART2_TX_PIN | USART2_RX_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(USART2_TX_PORT, USART2_TX_SRC, USART2_AF);
    GPIO_PinAFConfig(USART2_RX_PORT, USART2_RX_SRC, USART2_AF);

    /* ---- USART2 参数 ---- */
    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    /* ---- 开 RXNE 中断 ---- */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_Cmd(USART2, ENABLE);

    /* ---- NVIC：USART2 全局中断 ---- */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void USART2_SendByte(uint8_t data)
{
    USART_SendData(USART2, (uint16_t)data);
    USART_WAIT_FLAG(USART2, USART_FLAG_TXE);
}

void USART2_SendString(const char *str)
{
    while (*str != '\0')
    {
        USART2_SendByte((uint8_t)*str);
        str++;
    }
    USART_WAIT_FLAG(USART2, USART_FLAG_TC);
}

void USART2_Printf(const char *fmt, ...)
{
    va_list ap;
    int len, i;

    va_start(ap, fmt);
    len = vsnprintf(printf_buf2, sizeof(printf_buf2), fmt, ap);
    va_end(ap);

    if (len < 0) return;
    if (len >= (int)sizeof(printf_buf2)) len = sizeof(printf_buf2) - 1;

    for (i = 0; i < len; i++)
    {
        USART2_SendByte((uint8_t)printf_buf2[i]);
    }
    USART_WAIT_FLAG(USART2, USART_FLAG_TC);
}

uint8_t USART2_GetByte(uint8_t *data)
{
    if (usart2_rx_head != usart2_rx_tail)
    {
        *data = usart2_rx_buf[usart2_rx_tail];
        usart2_rx_tail = (usart2_rx_tail + 1) % RX_BUF_SIZE;
        return 1;
    }
    return 0;
}

uint16_t USART2_GetRxCount(void)
{
    return (usart2_rx_head - usart2_rx_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

void USART2_FlushRx(void)
{
    usart2_rx_tail = usart2_rx_head;
}

void USART2_RxHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART2);

        uint16_t next = (usart2_rx_head + 1) % RX_BUF_SIZE;
        if (next != usart2_rx_tail)
        {
            usart2_rx_buf[usart2_rx_head] = ch;
            usart2_rx_head = next;
        }
    }

    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {
        (void)USART2->SR;
        (void)USART2->DR;
    }
}

/* ========================================================================== */
/* USART3 驱动实现（接 ESP8266 WiFi 模块）                                     */
/* ========================================================================== */

/* ========== USART3 环形缓冲区 ==========
   单独放大到 512B：128B 在 115200bps 下只够 ~11ms，OLED 整屏重绘(~50ms)或
   MQTT 重连冻结期间，ESP8266 的 +IPD 下行会在 ISR 里静默丢弃（P3-#6） */
#define USART3_RX_BUF_SIZE  512
static volatile uint8_t  usart3_rx_buf[USART3_RX_BUF_SIZE];
static volatile uint16_t usart3_rx_head = 0;
static volatile uint16_t usart3_rx_tail = 0;

/**
  * @brief  初始化 USART3
  * @param  baud: 波特率，如 115200
  *   - 开 GPIOB + USART3 时钟
  *   - PB10 复用 AF7 做 TX（推挽），PB11 复用 AF7 做 RX（浮空）
  *   - 8N1 无硬件流控，使能 RXNE 中断 + NVIC
  */
void USART3_Init(uint32_t baud)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    USART_InitTypeDef   USART_InitStruct;
    NVIC_InitTypeDef    NVIC_InitStruct;

    /* ---- 开时钟：USART3 在 APB1，GPIOB 在 AHB1 ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* ---- PB10(TX) + PB11(RX) 复用 USART3 ---- */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     /* TX 推挽 */
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;      /* RX 上拉 */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = USART3_TX_PIN | USART3_RX_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(USART3_TX_PORT, USART3_TX_SRC, USART3_AF);
    GPIO_PinAFConfig(USART3_RX_PORT, USART3_RX_SRC, USART3_AF);

    /* ---- USART3 参数：波特率/8位/1停止位/无校验/收发使能 ---- */
    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStruct);

    /* ---- 开 RXNE 中断 ---- */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_Cmd(USART3, ENABLE);

    /* ---- NVIC：USART3 全局中断 ---- */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void USART3_SendByte(uint8_t data)
{
    USART_SendData(USART3, (uint16_t)data);
    USART_WAIT_FLAG(USART3, USART_FLAG_TXE);
}

void USART3_SendString(const char *str)
{
    while (*str != '\0')
    {
        USART3_SendByte((uint8_t)*str);
        str++;
    }
    USART_WAIT_FLAG(USART3, USART_FLAG_TC);
}

uint8_t USART3_GetByte(uint8_t *data)
{
    if (usart3_rx_head != usart3_rx_tail)
    {
        *data = usart3_rx_buf[usart3_rx_tail];
        usart3_rx_tail = (usart3_rx_tail + 1) % USART3_RX_BUF_SIZE;
        return 1;
    }
    return 0;
}

uint16_t USART3_GetRxCount(void)
{
    return (usart3_rx_head - usart3_rx_tail + USART3_RX_BUF_SIZE) % USART3_RX_BUF_SIZE;
}

void USART3_FlushRx(void)
{
    usart3_rx_tail = usart3_rx_head;
}

void USART3_RxHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART3);

        uint16_t next = (usart3_rx_head + 1) % USART3_RX_BUF_SIZE;
        if (next != usart3_rx_tail)
        {
            usart3_rx_buf[usart3_rx_head] = ch;
            usart3_rx_head = next;
        }
    }

    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        (void)USART3->SR;
        (void)USART3->DR;
    }
}
