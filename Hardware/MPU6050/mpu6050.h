/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 6 轴姿态传感器驱动头文件
  *
  * 硬件连接：
  *   MPU6050 VCC -> 3.3V
  *   MPU6050 GND -> GND
  *   MPU6050 SCL -> PB8 (软件 IIC)
  *   MPU6050 SDA -> PB9 (软件 IIC)
  *   AD0 悬空/接 GND -> 7bit 地址 = 0x68
  *   AD0 接 VCC       -> 7bit 地址 = 0x69
  *   CSB 必须接 VCC    -> 启用 I2C 模式
  *
  * 特性：
  *   - 3 轴加速度计 + 3 轴陀螺仪 + 温度
  *   - 原始数据 16bit，一次读 14 字节
  *   - 可选 DMP 硬件姿态解算（减轻 MCU 负担）
  ******************************************************************************
  */
#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f4xx.h"

/* ---- 7bit 从机地址（AD0 悬空/接地 = 0x68）---- */
#define MPU6050_ADDR_7BIT       0x68
#define MPU6050_ADDR_W          (MPU6050_ADDR_7BIT << 1)     /* 0xD0 */
#define MPU6050_ADDR_R          ((MPU6050_ADDR_7BIT << 1) | 0x01) /* 0xD1 */

/* ---- 寄存器地址 ---- */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C

/* 原始数据（连续 14 字节，地址自动自增） */
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

/* 电源管理 */
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C

/* WHO_AM_I：读回应等于 MPU6050_ADDR_7BIT */
#define MPU6050_WHO_AM_I        0x75

/* 信号路径复位：写 0x07 复位温度/加速度/陀螺仪信号路径 */
#define MPU6050_SIGNAL_PATH_RST 0x68

/* ---- 量程换算系数 ---- */
/* 加速度 ±2g: 16384 LSB/g */
#define ACCEL_SCALE_2G          16384.0f
/* 陀螺仪 ±2000°/s: 16.4 LSB/(°/s) */
#define GYRO_SCALE_2000DPS      16.4f

/* ---- 接口函数 ---- */
uint8_t  MPU6050_Init(void);
uint8_t  MPU6050_ReadReg(uint8_t reg);
void     MPU6050_WriteReg(uint8_t reg, uint8_t val);
void     MPU6050_ReadReg_Continue(uint8_t reg, uint8_t len, uint8_t *data);
void     MPU6050_GetRaw(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz,
                        int16_t *temp_raw);
void     MPU6050_GetPhysical(float *ax_g, float *ay_g, float *az_g,
                             float *gx_dps, float *gy_dps, float *gz_dps,
                             float *temp_c);

#endif /* __MPU6050_H__ */
