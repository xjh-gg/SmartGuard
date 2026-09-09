/**
  ******************************************************************************
  * @file    dht11.c
  * @brief   DHT11 温湿度传感器驱动实现
  *
  * 通信协议：单总线（1-Wire），时序严格依赖 us 级延时。
  * 注意：PG9 配置为开漏输出（OD），DHT11 模块已板上拉电阻到 VCC。
  ******************************************************************************
  */
#include "dht11.h"
#include "delay.h"

/**
  * @brief  切换 DHT11 数据引脚为输入或输出模式
  * @param  mode: GPIO_Mode_IN 或 GPIO_Mode_OUT
  */
static void DHT11_PinMode(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = DHT11_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      /* 开漏，方便单总线双向通信 */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   /* 模块已有上拉电阻 */
    GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  初始化 DHT11 数据引脚
  */
void DHT11_Init(void)
{
    RCC_AHB1PeriphClockCmd(DHT11_GPIO_CLK, ENABLE);

    DHT11_PinMode(GPIO_Mode_OUT);

    /* 初始释放总线：输出高电平 */
    GPIO_SetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
}

/**
  * @brief  读取一次 DHT11 数据
  * @param  data: 接收缓冲区，长度至少 5 字节
  * @retval 0   读取成功
  *         -1  等待 DHT11 响应超时
  *         -2  响应低电平超时
  *         -3  响应高电平超时
  *         -4  数据位低电平超时
  *         -5  数据位高电平超时
  *         -6  校验和错误
  */
int32_t DHT11_Read(uint8_t *data)
{
    uint32_t t = 0;
    uint8_t  d = 0;
    int32_t  i = 0;
    int32_t  j = 0;
    uint32_t check_sum = 0;

    /* 1. MCU 发送起始信号：拉低总线 20ms */
    DHT11_PinMode(GPIO_Mode_OUT);
    GPIO_ResetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
    delay_ms(20);
    GPIO_SetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
    delay_us(30);

    /* 2. 切换为输入，等待 DHT11 响应 */
    DHT11_PinMode(GPIO_Mode_IN);

    /* 等待 DHT11 拉低总线（响应信号） */
    t = 0;
    while (GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) != Bit_RESET)
    {
        t++;
        delay_us(1);
        if (t >= 4000)
            return -1;
    }

    /* 等待 DHT11 释放总线（低电平 80us） */
    t = 0;
    while (GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == Bit_RESET)
    {
        t++;
        delay_us(1);
        if (t >= 100)
            return -2;
    }

    /* 等待 DHT11 拉低总线（高电平 80us） */
    t = 0;
    while (GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) != Bit_RESET)
    {
        t++;
        delay_us(1);
        if (t >= 100)
            return -3;
    }

    /* 3. 读取 5 字节数据（高位先传） */
    for (j = 0; j < 5; j++)
    {
        d = 0;
        for (i = 7; i >= 0; i--)
        {
            /* 等待每一位开始的低电平结束 */
            t = 0;
            while (GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == Bit_RESET)
            {
                t++;
                delay_us(1);
                if (t >= 100)
                    return -4;
            }

            /* 延时 40us 后采样总线电平：高=1，低=0 */
            delay_us(40);
            if (GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) != Bit_RESET)
            {
                d |= (uint8_t)(1 << i);

                /* 等待高电平结束 */
                t = 0;
                while (GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) != Bit_RESET)
                {
                    t++;
                    delay_us(1);
                    if (t >= 100)
                        return -5;
                }
            }
        }
        data[j] = d;
    }

    /* 4. 通信结束，释放总线 */
    DHT11_PinMode(GPIO_Mode_OUT);
    GPIO_SetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
    delay_us(100);

    /* 5. 校验和检查 */
    check_sum = data[0] + data[1] + data[2] + data[3];
    check_sum &= 0xFF;
    if (check_sum != data[4])
        return -6;

    return 0;
}

/**
  * @brief  读取 DHT11 温湿度（浮点封装，供 SensorHub 调用）
  * @param  temperature: 输出温度值（°C）
  * @param  humidity:    输出湿度值（%）
  * @retval 0 = 成功，1 = 失败
  */
uint8_t DHT11_Read_Data(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};
    int32_t ret;

    ret = DHT11_Read(data);
    if (ret != 0)
    {
        /* 失败时不修改输出参数，调用方保留上一次有效值 */
        return 1;
    }

    *humidity    = (float)data[0];
    *temperature = (float)data[2];

    return 0;
}
