/**
  ******************************************************************************
  * @file    at24c02.c
  * @brief   AT24C02 串行 EEPROM 驱动实现（基于共享软件 IIC 总线 iic.c）
  *
  * 时序（与课程示例一致）：
  *   写：START -> 0xA0 -> ACK -> 字地址 -> ACK -> 数据*n -> ACK -> STOP
  *   读：START -> 0xA0 -> ACK -> 字地址 -> ACK -> 重复START -> 0xA1 -> ACK
  *       -> 读数据*(n-1) + ACK -> 读最后一字节 + NACK -> STOP
  *
  * 写周期：页写后器件内部擦写 ≤5ms，期间对任何访问回 NACK。
  *   AT24C02_WaitWriteDone() 用 ACK 轮询（数据手册推荐方式）代替盲等 5ms，
  *   实际等待时间 = 真实写周期，通常 2~4ms 即可继续。
  ******************************************************************************
  */
#include "at24c02.h"
#include "iic.h"
#include "delay.h"

/**
  * @brief  初始化：确保软件 IIC 总线就绪
  * @note   MPU6050_Init 也会调 IIC_Init，重复初始化幂等（同参数重配 GPIO），
  *         因此无论 USE_MPU6050 开关如何，这里都直接调用保证总线可用。
  */
void AT24C02_Init(void)
{
    IIC_Init();
}

/**
  * @brief  探测 AT24C02 是否在线（发设备地址看 ACK）
  * @retval 1=在线，0=无应答（未焊接/总线故障）
  */
uint8_t AT24C02_Check(void)
{
    uint8_t ack;

    IIC_Start();
    IIC_SendByte(AT24C02_ADDR_W);
    ack = IIC_ReceiveAck();
    IIC_Stop();

    return (ack == IIC_ACK) ? 1 : 0;
}

/**
  * @brief  等待 EEPROM 内部写周期完成（ACK 轮询，有界 ~2000 次）
  */
void AT24C02_WaitWriteDone(void)
{
    uint16_t t = 2000;

    while (t--)
    {
        IIC_Start();
        IIC_SendByte(AT24C02_ADDR_W);
        if (IIC_ReceiveAck() == IIC_ACK)
        {
            IIC_Stop();
            return;
        }
        IIC_Stop();
    }
}

/**
  * @brief  单字节写
  * @retval 0=成功；1=设备地址无应答；2=字地址无应答；3=数据无应答
  */
uint8_t AT24C02_WriteByte(uint8_t mem_addr, uint8_t data)
{
    IIC_Start();
    IIC_SendByte(AT24C02_ADDR_W);
    if (IIC_ReceiveAck()) { IIC_Stop(); return 1; }

    IIC_SendByte(mem_addr);
    if (IIC_ReceiveAck()) { IIC_Stop(); return 2; }

    IIC_SendByte(data);
    if (IIC_ReceiveAck()) { IIC_Stop(); return 3; }

    IIC_Stop();
    AT24C02_WaitWriteDone();
    return 0;
}

/**
  * @brief  缓冲写：自动按 16 字节页边界切分（页写不可跨页，跨页会回卷覆盖页首）
  * @param  mem_addr: 起始存储地址
  * @param  buf:      数据缓冲区
  * @param  len:      数据长度（不得超过 256 - mem_addr）
  * @retval 0=成功，非 0=失败（同 WriteByte 错误码）
  */
uint8_t AT24C02_Write(uint8_t mem_addr, const uint8_t *buf, uint16_t len)
{
    while (len > 0)
    {
        uint8_t page_off = mem_addr & (AT24C02_PAGE_SIZE - 1);
        uint8_t chunk    = AT24C02_PAGE_SIZE - page_off;
        uint8_t i;

        if (chunk > len) chunk = (uint8_t)len;

        IIC_Start();
        IIC_SendByte(AT24C02_ADDR_W);
        if (IIC_ReceiveAck()) { IIC_Stop(); return 1; }

        IIC_SendByte(mem_addr);
        if (IIC_ReceiveAck()) { IIC_Stop(); return 2; }

        for (i = 0; i < chunk; i++)
        {
            IIC_SendByte(buf[i]);
            if (IIC_ReceiveAck()) { IIC_Stop(); return 3; }
        }
        IIC_Stop();
        AT24C02_WaitWriteDone();

        mem_addr += chunk;
        buf      += chunk;
        len      -= chunk;
    }
    return 0;
}

/**
  * @brief  随机读一个字节
  * @retval 0=成功；1/2/3=各阶段无应答
  */
uint8_t AT24C02_ReadByte(uint8_t mem_addr, uint8_t *data)
{
    return AT24C02_Read(mem_addr, data, 1);
}

/**
  * @brief  顺序读 len 字节（器件内部地址指针自增）
  * @retval 0=成功，非 0=失败
  */
uint8_t AT24C02_Read(uint8_t mem_addr, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (len == 0) return 0;

    IIC_Start();
    IIC_SendByte(AT24C02_ADDR_W);
    if (IIC_ReceiveAck()) { IIC_Stop(); return 1; }

    IIC_SendByte(mem_addr);
    if (IIC_ReceiveAck()) { IIC_Stop(); return 2; }

    IIC_Start();
    IIC_SendByte(AT24C02_ADDR_R);
    if (IIC_ReceiveAck()) { IIC_Stop(); return 3; }

    for (i = 0; i < len; i++)
    {
        buf[i] = IIC_ReceiveByte();
        IIC_SendAck((i == len - 1) ? IIC_NACK : IIC_ACK);
    }
    IIC_Stop();
    return 0;
}
