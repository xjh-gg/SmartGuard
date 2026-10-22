/**
  ******************************************************************************
  * @file    usart.c
  * @brief   ��������ʵ�֣�USART1��
  *
  * ���ܣ�
  *   1. USART1 ��ʼ����PA9(TX) + PA10(RX)��AF7 ���ã����貨����
  *   2. ���ͣ�SendByte / SendString / Printf��vsprintf ��ʽ����
  *   3. ���գ��жϷ�ʽ��RXNE�������λ��������棬GetByte ��������ȡ
  *
  * ���Ͳ��ԣ�
  *   ��ѯ TXE ��־���������ݼĴ����գ�+ TC ��־��������ɣ���
  *   �򵥿ɿ����ʺϽ�ѧʾ����ʵ�ʹ��̴��������ɸ� DMA��
  *
  * ���ղ��ԣ�
  *   RXNE �жϰ��ֽ�д�뻷�λ���������ѭ���� GetByte ȡ����
  *   ��������ʱ���������ݣ���ѧ�򻯣�ʵ�ʿ������أ���
  ******************************************************************************
  */
#include "usart.h"

#if USE_FREERTOS
/* FreeRTOS: USART1 is shared by every task. printf_buf1 is a static buffer, so
   concurrent USART1_Printf/SendString calls from different tasks would corrupt
   the formatted text. A plain mutex (priority inheritance via heap_4) serializes
   log lines; before scheduler starts the lock helpers are pass-through. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
static SemaphoreHandle_t s_log1_mutex = (SemaphoreHandle_t)0;

void USART1_LockInit(void)
{
    if (s_log1_mutex == (SemaphoreHandle_t)0)
    {
        s_log1_mutex = xSemaphoreCreateMutex();
    }
}

static void log1_lock(void)
{
    if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&
        (s_log1_mutex != (SemaphoreHandle_t)0))
    {
        (void)xSemaphoreTake(s_log1_mutex, portMAX_DELAY);
    }
}

static void log1_unlock(void)
{
    if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&
        (s_log1_mutex != (SemaphoreHandle_t)0))
    {
        (void)xSemaphoreGive(s_log1_mutex);
    }
}
#endif /* USE_FREERTOS */

/* ���͵ȴ����ޣ�����/ʱ���쳣ʱ��ֹ��ѭ�����ÿ�����P3-#3����
   115200bps ���ֽ� ~87us��100000 �ε���Զ�� ms ��������·����Ӱ�� */
#define USART_WAIT_MAX  100000U
#define USART_WAIT_FLAG(uart, flag)                               \
    do {                                                          \
        uint32_t _to = USART_WAIT_MAX;                            \
        while (USART_GetFlagStatus((uart), (flag)) == RESET)     \
        {                                                         \
            if (--_to == 0) break;                                \
        }                                                         \
    } while (0)

/* ========== USART1 ���λ����� ========== */
static volatile uint8_t  usart1_rx_buf[RX_BUF_SIZE];
static volatile uint16_t usart1_rx_head = 0;
static volatile uint16_t usart1_rx_tail = 0;

/* ========== USART2 ���λ����� ========== */
static volatile uint8_t  usart2_rx_buf[RX_BUF_SIZE];
static volatile uint16_t usart2_rx_head = 0;
static volatile uint16_t usart2_rx_tail = 0;

/* Printf ��ʱ��������ÿ�����ڶ��������Ⲣ������ʱ���ݽ��棩��
   USART3 ֻ�� AT ָ���շ���û�� Printf����ֻ������ */
static char printf_buf1[128];
static char printf_buf2[128];

/**
  * @brief  ��ʼ�� USART1
  * @param  baud: �����ʣ��� 115200
  *   - �� GPIOA + USART1 ʱ��
  *   - PA9 ���� AF7 �� TX�����죩��PA10 ���� AF7 �� RX�����գ�
  *   - 8N1 ��Ӳ�����أ�ʹ�� RXNE �ж� + NVIC
  */
void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    USART_InitTypeDef   USART_InitStruct;
    NVIC_InitTypeDef    NVIC_InitStruct;

    /* ---- ��ʱ�ӣ�USART1 �� APB2��GPIOA �� AHB1 ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* ---- PA9(TX) + PA10(RX) ���� USART1 ---- */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     /* TX ���� */
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;      /* RX ���������иߵ�ƽ�� */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = USART1_TX_PIN | USART1_RX_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(USART1_TX_PORT, USART1_TX_SRC, USART1_AF);
    GPIO_PinAFConfig(USART1_RX_PORT, USART1_RX_SRC, USART1_AF);

    /* ---- USART1 ������������/8λ/1ֹͣλ/��У��/�շ�ʹ�� ---- */
    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);

    /* ---- �� RXNE �жϣ��յ����ݾʹ��� ---- */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ClearFlag(USART1, USART_FLAG_TC);          /* ���������ɱ�־ */
    USART_Cmd(USART1, ENABLE);

    /* ---- NVIC��USART1 ȫ���жϣ��������ļ����� USART1_IRQn�� ---- */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;   /* ��ռ���ȼ� 1 */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;   /* �����ȼ� 1 */
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/**
  * @brief  ����һ���ֽڣ������ȴ� TXE��
  * @param  data: Ҫ���͵��ֽ�
  */
void USART1_SendByte(uint8_t data)
{
    USART_SendData(USART1, (uint16_t)data);
    USART_WAIT_FLAG(USART1, USART_FLAG_TXE);
}

/**
  * @brief  �����ַ���
  * @param  str: �� '\0' ��β���ַ���
  */
void USART1_SendString(const char *str)
{
#if USE_FREERTOS
    log1_lock();
#endif
    while (*str != '\0')
    {
        USART1_SendByte((uint8_t)*str);
        str++;
    }
    /* 等最后一个字节发完（TC=发送完成） */
    USART_WAIT_FLAG(USART1, USART_FLAG_TC);
#if USE_FREERTOS
    log1_unlock();
#endif
}

/**
  * @brief  ��ʽ�����ͣ��÷�ͬ printf��
  * @param  fmt: ��ʽ�ַ���
  *   �ڲ��� vsnprintf ��ʽ���� printf_buf �����ֽڷ���
  *   ע�⣺��Ƕ��ʽ���� vsnprintf ��Ҫ --use_micro_lib ������ retarget
  */
void USART1_Printf(const char *fmt, ...)
{
    va_list ap;
    int len, i;

#if USE_FREERTOS
    log1_lock();   /* protect the static printf_buf1 + wire time from other tasks */
#endif

    va_start(ap, fmt);
    len = vsnprintf(printf_buf1, sizeof(printf_buf1), fmt, ap);
    va_end(ap);

    if (len < 0)
    {
#if USE_FREERTOS
        log1_unlock();
#endif
        return;
    }
    if (len >= (int)sizeof(printf_buf1)) len = sizeof(printf_buf1) - 1;

    for (i = 0; i < len; i++)
    {
        USART1_SendByte((uint8_t)printf_buf1[i]);
    }
    USART_WAIT_FLAG(USART1, USART_FLAG_TC);
#if USE_FREERTOS
    log1_unlock();
#endif
}

/**
  * @brief  ��������ȡһ�ֽ�
  * @param  data: �����ȡ�����ֽ�
  * @retval 1 = �ɹ��������ݣ�0 = ��������
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
  * @brief  ��ȡ���ջ�������δ���ֽ���
  * @retval ��ǰ�ɶ��ֽ���
  */
uint16_t USART1_GetRxCount(void)
{
    return (usart1_rx_head - usart1_rx_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

/**
  * @brief  ��ս��ջ�����
  */
void USART1_FlushRx(void)
{
    usart1_rx_tail = usart1_rx_head;
}

/* ========================================================================== */
/* ���º����� stm32f4xx_it.c �� USART1_IRQHandler ����                       */
/* ========================================================================== */

/**
  * @brief  ���ڽ����жϷ��������� USART1_IRQHandler ���ã�
  *         ��ȡ USART1->DR��д�뻷�λ�����
  *         ��������ʱ�������ֽڣ��������µģ�
  */
void USART1_RxHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART1);    /* �� DR �Զ��� RXNE */

        uint16_t next = (usart1_rx_head + 1) % RX_BUF_SIZE;
        if (next != usart1_rx_tail)
        {
            usart1_rx_buf[usart1_rx_head] = ch;
            usart1_rx_head = next;
        }
        /* next == usart1_rx_tail ��ʾ����������������ǰ�ֽ� */
    }

    /* �����������ORE������ SR �ٶ� DR ��� */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART1->SR;
        (void)USART1->DR;
    }
}

/* ========================================================================== */
/* USART2 ����ʵ��                                                            */
/*   �� USART1 ��ȫ������ʹ�ö����� GPIO��NVIC�����λ�����                     */
/* ========================================================================== */

/**
  * @brief  ��ʼ�� USART2
  * @param  baud: �����ʣ��� 9600
  *   - �� GPIOA + USART2 ʱ��
  *   - PA2 ���� AF7 �� TX�����죩��PA3 ���� AF7 �� RX�����գ�
  *   - 8N1 ��Ӳ�����أ�ʹ�� RXNE �ж� + NVIC
  */
void USART2_Init(uint32_t baud)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    USART_InitTypeDef   USART_InitStruct;
    NVIC_InitTypeDef    NVIC_InitStruct;

    /* ---- ��ʱ�ӣ�USART2 �� APB1��GPIOA �� AHB1 ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* ---- PA2(TX) + PA3(RX) ���� USART2 ---- */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = USART2_TX_PIN | USART2_RX_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(USART2_TX_PORT, USART2_TX_SRC, USART2_AF);
    GPIO_PinAFConfig(USART2_RX_PORT, USART2_RX_SRC, USART2_AF);

    /* ---- USART2 ���� ---- */
    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    /* ---- �� RXNE �ж� ---- */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_Cmd(USART2, ENABLE);

    /* ---- NVIC��USART2 ȫ���ж� ---- */
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
/* USART3 ����ʵ�֣��� ESP8266 WiFi ģ�飩                                     */
/* ========================================================================== */

/* ========== USART3 ���λ����� ==========
   �����Ŵ� 512B��128B �� 115200bps ��ֻ�� ~11ms��OLED �����ػ�(~50ms)��
   MQTT ���������ڼ䣬ESP8266 �� +IPD ���л��� ISR �ﾲĬ������P3-#6�� */
#define USART3_RX_BUF_SIZE  512
static volatile uint8_t  usart3_rx_buf[USART3_RX_BUF_SIZE];
static volatile uint16_t usart3_rx_head = 0;
static volatile uint16_t usart3_rx_tail = 0;

/**
  * @brief  ��ʼ�� USART3
  * @param  baud: �����ʣ��� 115200
  *   - �� GPIOB + USART3 ʱ��
  *   - PB10 ���� AF7 �� TX�����죩��PB11 ���� AF7 �� RX�����գ�
  *   - 8N1 ��Ӳ�����أ�ʹ�� RXNE �ж� + NVIC
  */
void USART3_Init(uint32_t baud)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    USART_InitTypeDef   USART_InitStruct;
    NVIC_InitTypeDef    NVIC_InitStruct;

    /* ---- ��ʱ�ӣ�USART3 �� APB1��GPIOB �� AHB1 ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* ---- PB10(TX) + PB11(RX) ���� USART3 ---- */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     /* TX ���� */
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;      /* RX ���� */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = USART3_TX_PIN | USART3_RX_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(USART3_TX_PORT, USART3_TX_SRC, USART3_AF);
    GPIO_PinAFConfig(USART3_RX_PORT, USART3_RX_SRC, USART3_AF);

    /* ---- USART3 ������������/8λ/1ֹͣλ/��У��/�շ�ʹ�� ---- */
    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStruct);

    /* ---- �� RXNE �ж� ---- */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_Cmd(USART3, ENABLE);

    /* ---- NVIC��USART3 ȫ���ж� ---- */
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
