/**
  ******************************************************************************
  * @file    w25qxx.c
  * @brief   W25Qxx SPI NOR FLASH 驱动实现（软件 SPI，模式 0：CPOL=0 CPHA=0）
  *
  * 模式 0 时序（与课程示例 19spi 一致）：
  *   时钟低电平期间主机改变 MOSI；上升沿双方采样；下降沿移位准备下一位。
  ******************************************************************************
  */
#include "w25qxx.h"
#include "delay.h"
#include "iwdg.h"

/* ---- 引脚控制宏 ---- */
#define W25QXX_CS_H()     GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define W25QXX_CS_L()     GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define W25QXX_SCK_H()    GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define W25QXX_SCK_L()    GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define W25QXX_MOSI_H()   GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define W25QXX_MOSI_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define W25QXX_MISO_RD()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)

/* ---- 指令集 ---- */
#define W25QXX_CMD_READ_ID     0x90
#define W25QXX_CMD_READ_SR1    0x05
#define W25QXX_CMD_WRITE_EN    0x06
#define W25QXX_CMD_SECTOR_ER   0x20   /* 4KB */
#define W25QXX_CMD_PAGE_PROG   0x02
#define W25QXX_CMD_READ_DATA   0x03

/**
  * @brief  软件 SPI 收发一个字节（模式 0，MSB 在前）
  */
static uint8_t W25QXX_SPI_Byte(uint8_t data)
{
    uint8_t recv = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80) W25QXX_MOSI_H();
        else             W25QXX_MOSI_L();

        W25QXX_SCK_H();              /* 上升沿：从机采样 MOSI，主机采样 MISO */
        delay_us(1);
        if (W25QXX_MISO_RD())
            recv |= (uint8_t)(0x80 >> i);

        W25QXX_SCK_L();              /* 下降沿：准备下一位 */
        delay_us(1);
        data <<= 1;
    }
    return recv;
}

/**
  * @brief  初始化 SPI 引脚（CS/SCK/MOSI 推挽输出，MISO 输入；空闲 CS=1 SCK=0）
  */
void W25QXX_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    W25QXX_CS_H();
    W25QXX_SCK_L();
    delay_ms(10);
}

/**
  * @brief  读厂商/设备 ID（指令 0x90）
  * @retval 高字节=厂商 ID（Winbond=0xEF），低字节=容量 ID
  *         （0x16=W25Q64，0x17=W25Q128 等）；未焊接时全 0xFF 或 0
  */
uint16_t W25QXX_ReadID(void)
{
    uint16_t id;

    W25QXX_CS_L();
    W25QXX_SPI_Byte(W25QXX_CMD_READ_ID);
    W25QXX_SPI_Byte(0x00);
    W25QXX_SPI_Byte(0x00);
    W25QXX_SPI_Byte(0x00);
    id  = (uint16_t)W25QXX_SPI_Byte(0xFF) << 8;
    id |= W25QXX_SPI_Byte(0xFF);
    W25QXX_CS_H();

    return id;
}

/**
  * @brief  读状态寄存器 1（bit0: 1=忙，0=空闲）
  */
uint8_t W25QXX_ReadSR(void)
{
    uint8_t sr;

    W25QXX_CS_L();
    W25QXX_SPI_Byte(W25QXX_CMD_READ_SR1);
    sr = W25QXX_SPI_Byte(0xFF);
    W25QXX_CS_H();

    return sr;
}

/**
  * @brief  等待擦写完成（扇区擦除典型 45ms/最大 400ms，页编程最大 3ms）
  * @note   有界 ~3s 防芯片异常死等；循环内喂狗（本函数在 IWDG 启动后也可达）
  */
void W25QXX_WaitBusy(void)
{
    uint32_t start = g_tick;

    while (W25QXX_ReadSR() & 0x01)
    {
        if ((g_tick - start) > 3000) break;
        IWDG_Feed();
    }
}

/**
  * @brief  写使能（每次擦除/编程前必须发，芯片执行完自动复位该位）
  */
void W25QXX_WriteEnable(void)
{
    W25QXX_CS_L();
    W25QXX_SPI_Byte(W25QXX_CMD_WRITE_EN);
    W25QXX_CS_H();
}

/**
  * @brief  擦除 addr 所在的 4KB 扇区（全变 0xFF）
  */
void W25QXX_SectorErase(uint32_t addr)
{
    W25QXX_WriteEnable();

    W25QXX_CS_L();
    W25QXX_SPI_Byte(W25QXX_CMD_SECTOR_ER);
    W25QXX_SPI_Byte((uint8_t)(addr >> 16));
    W25QXX_SPI_Byte((uint8_t)(addr >> 8));
    W25QXX_SPI_Byte((uint8_t)addr);
    W25QXX_CS_H();

    W25QXX_WaitBusy();
}

/**
  * @brief  页编程（只把 1 写成 0，写前目标区域必须已擦除）
  * @param  len ≤ 256，且 [addr, addr+len) 不得跨 256B 页边界
  */
void W25QXX_PageWrite(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    W25QXX_WriteEnable();

    W25QXX_CS_L();
    W25QXX_SPI_Byte(W25QXX_CMD_PAGE_PROG);
    W25QXX_SPI_Byte((uint8_t)(addr >> 16));
    W25QXX_SPI_Byte((uint8_t)(addr >> 8));
    W25QXX_SPI_Byte((uint8_t)addr);
    for (i = 0; i < len; i++)
        W25QXX_SPI_Byte(buf[i]);
    W25QXX_CS_H();

    W25QXX_WaitBusy();
}

/**
  * @brief  读数据（长度不限，地址自增）
  */
void W25QXX_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t i;

    W25QXX_CS_L();
    W25QXX_SPI_Byte(W25QXX_CMD_READ_DATA);
    W25QXX_SPI_Byte((uint8_t)(addr >> 16));
    W25QXX_SPI_Byte((uint8_t)(addr >> 8));
    W25QXX_SPI_Byte((uint8_t)addr);
    for (i = 0; i < len; i++)
        buf[i] = W25QXX_SPI_Byte(0xFF);
    W25QXX_CS_H();
}
