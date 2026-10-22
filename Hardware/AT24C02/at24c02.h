/**
  ******************************************************************************
  * @file    at24c02.h
  * @brief   AT24C02 串行 EEPROM 驱动（板载，软件 IIC 总线）
  *
  * 硬件连接（GEC-M4 板载，与 MPU6050 共用同一条软件 IIC 总线）：
  *   SCL -> PB8，SDA -> PB9
  *   7bit 从机地址 0x50（A2/A1/A0 接地），与 MPU6050(0x68) 地址不冲突。
  *   协作式调度下总线事务天然串行，两个从机可安全共存。
  *
  * 器件参数：2Kbit = 256 字节；页写 16 字节（不可跨页）；写周期 ≤5ms。
  *
  * 用途：SmartGuard 配置掉电存储（config_store.c），替代内部 FLASH 方案：
  *   按字节擦写，无 128KB 扇区擦除的 ~1s CPU 停顿（IWDG 隐患根治），
  *   擦写寿命 ~100 万次（内部 FLASH 仅 ~1 万次）。
  ******************************************************************************
  */
#ifndef __AT24C02_H__
#define __AT24C02_H__

#include "stm32f4xx.h"

/* ---- 器件地址（7bit 0x50 左移 1 位，末位读/写位）---- */
#define AT24C02_ADDR_W      0xA0
#define AT24C02_ADDR_R      0xA1

#define AT24C02_PAGE_SIZE   8       /* 经典 AT24C02 页写缓冲为 8 字节
                                       （16 字节页的新型号同样兼容 8 字节块；
                                        误设 16 在 8 字节页芯片上会回卷覆盖） */
#define AT24C02_CAPACITY    256     /* 总容量 256 字节 */

void    AT24C02_Init(void);                 /* 初始化（内含 IIC_Init，重复调用安全） */
uint8_t AT24C02_Check(void);                /* 探测器件是否在线：1=在，0=不在 */
uint8_t AT24C02_WriteByte(uint8_t mem_addr, uint8_t data);
uint8_t AT24C02_ReadByte(uint8_t mem_addr, uint8_t *data);
/* 缓冲写：自动按 16 字节页切分 + 每页写后 ACK 轮询等待。0=成功 */
uint8_t AT24C02_Write(uint8_t mem_addr, const uint8_t *buf, uint16_t len);
/* 顺序读 len 字节。0=成功 */
uint8_t AT24C02_Read(uint8_t mem_addr, uint8_t *buf, uint16_t len);
void    AT24C02_WaitWriteDone(void);        /* ACK 轮询等待内部写周期完成 */

#endif /* __AT24C02_H__ */
