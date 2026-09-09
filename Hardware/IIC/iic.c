/**
  ******************************************************************************
  * @file    iic.c
  * @brief   软件 I2C（模拟 IIC）总线驱动实现
  *
  * 时序要点：
  *   - 数据有效性：SDA 只能在 SCL 为低电平时改变
  *   - 启动信号：SCL=1 期间，SDA 由 1 -> 0
  *   - 停止信号：SCL=1 期间，SDA 由 0 -> 1
  *   - 应答信号：第 9 个时钟，SDA=0 表示 ACK
  ******************************************************************************
  */
#include "iic.h"
#include "delay.h"

/* 切换 SDA 引脚为输入/输出模式（SDA 是双向线） */
static void IIC_SDA_PinMode(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin   = IIC_SDA_PIN;
    g.GPIO_Mode  = mode;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_OType = GPIO_OType_PP;     /* 推挽输出，F407 软件 IIC 可工作 */
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(IIC_SDA_PORT, &g);
}

#define IIC_SDA_OUT()   IIC_SDA_PinMode(GPIO_Mode_OUT)
#define IIC_SDA_IN()    IIC_SDA_PinMode(GPIO_Mode_IN)

/* ---- 初始化 ---- */
void IIC_Init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(IIC_GPIO_CLK, ENABLE);

    /* SCL: 推挽输出 + 上拉 */
    g.GPIO_Pin   = IIC_SCL_PIN;
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(IIC_SCL_PORT, &g);

    /* SDA: 初始输出 + 上拉（后续双向切换） */
    IIC_SDA_PinMode(GPIO_Mode_OUT);

    /* 空闲状态释放总线 */
    IIC_SCL_H();
    IIC_SDA_H();
}

/* ---- 启动信号：SCL=1 期间，SDA 由 1 -> 0 ---- */
void IIC_Start(void)
{
    IIC_SDA_OUT();
    IIC_SDA_H();
    IIC_SCL_H();
    delay_us(5);
    IIC_SDA_L();       /* SCL 高电平期间 SDA 拉低 = 启动 */
    delay_us(5);
    IIC_SCL_L();
    delay_us(5);
}

/* ---- 停止信号：SCL=1 期间，SDA 由 0 -> 1 ---- */
void IIC_Stop(void)
{
    IIC_SDA_OUT();
    IIC_SCL_L();
    IIC_SDA_L();
    delay_us(5);
    IIC_SCL_H();
    delay_us(5);
    IIC_SDA_H();       /* SCL 高电平期间 SDA 拉高 = 停止 */
    delay_us(5);
}

/* ---- 发送一字节（MSB 在前） ---- */
void IIC_SendByte(uint8_t data)
{
    IIC_SDA_OUT();
    for (int i = 0; i < 8; i++)
    {
        if (data & 0x80)
            IIC_SDA_H();
        else
            IIC_SDA_L();
        delay_us(2);
        IIC_SCL_H();       /* 上升沿：从机采样 */
        delay_us(2);
        IIC_SCL_L();
        delay_us(2);
        data <<= 1;
    }
}

/* ---- 接收一字节（MSB 在前） ---- */
uint8_t IIC_ReceiveByte(void)
{
    uint8_t data = 0;
    IIC_SDA_IN();
    for (int i = 0; i < 8; i++)
    {
        IIC_SCL_L();
        delay_us(2);
        IIC_SCL_H();       /* 上升沿：从机把数据放到 SDA */
        delay_us(2);
        data <<= 1;
        if (IIC_SDA_READ() == Bit_SET)
            data |= 0x01;
    }
    IIC_SCL_L();
    delay_us(2);
    return data;
}

/* ---- 主机接收从机应答（第 9 个时钟）：返回 0=ACK，1=NACK ---- */
uint8_t IIC_ReceiveAck(void)
{
    uint8_t ack;
    IIC_SDA_IN();
    delay_us(2);
    IIC_SCL_H();
    delay_us(2);
    ack = IIC_SDA_READ();
    IIC_SCL_L();
    delay_us(2);
    return ack;
}

/* ---- 主机发送应答（第 9 个时钟）：0=ACK，1=NACK ---- */
void IIC_SendAck(uint8_t ack)
{
    IIC_SDA_OUT();
    if (ack)
        IIC_SDA_H();
    else
        IIC_SDA_L();
    delay_us(2);
    IIC_SCL_H();
    delay_us(2);
    IIC_SCL_L();
    delay_us(2);
    IIC_SDA_H();   /* 释放总线 */
}
