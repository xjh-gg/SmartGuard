/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 6 轴姿态传感器驱动实现
  *
  * 依赖 Hardware/IIC 软件 IIC 驱动
  *
  * 量程配置（默认）：
  *   加速度 ±2g
  *   陀螺仪 ±2000°/s
  ******************************************************************************
  */
#include "mpu6050.h"
#include "iic.h"
#include "delay.h"
#include "usart.h"

/* ---- 写单个寄存器 ---- */
void MPU6050_WriteReg(uint8_t reg, uint8_t val)
{
    IIC_Start();
    IIC_SendByte(MPU6050_ADDR_W);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    IIC_SendByte(val);
    IIC_ReceiveAck();
    IIC_Stop();
}

/* ---- 读单个寄存器 ---- */
uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t data;

    IIC_Start();
    IIC_SendByte(MPU6050_ADDR_W);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();

    IIC_Start();                    /* 重启 */
    IIC_SendByte(MPU6050_ADDR_R);
    IIC_ReceiveAck();

    data = IIC_ReceiveByte();
    IIC_SendAck(IIC_NACK);
    IIC_Stop();

    return data;
}

/* ---- 连续读寄存器（地址自动自增）---- */
void MPU6050_ReadReg_Continue(uint8_t reg, uint8_t len, uint8_t *data)
{
    IIC_Start();
    IIC_SendByte(MPU6050_ADDR_W);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();

    IIC_Start();                    /* 重启 */
    IIC_SendByte(MPU6050_ADDR_R);
    IIC_ReceiveAck();

    for (uint8_t i = 0; i < len; i++)
    {
        data[i] = IIC_ReceiveByte();
        if (i == len - 1)
            IIC_SendAck(IIC_NACK); /* 最后一字节发 NACK */
        else
            IIC_SendAck(IIC_ACK);
    }

    IIC_Stop();
}

/* ---- 初始化 ----
 * 返回值：
 *   0x00 = 成功（WHO_AM_I 匹配）
 *   其他 = 失败（WHO_AM_I 不匹配，返回读到的实际值）
 */
uint8_t MPU6050_Init(void)
{
    uint8_t id;

    /* 1. 软件 IIC 初始化 */
    IIC_Init();

    /* 1.5 设备复位（数据手册推荐上电后执行） */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
    delay_ms(100);

    /* 2. 唤醒，选 X 轴陀螺 PLL 时钟（GY-521 资料/Arduino 库做法）*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    delay_ms(100);   /* 加速度计需要 ≥50ms 稳定 */

    /* 3. 电源管理寄存器2，保持0，所有轴不待机 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);

    /* 4. WHO_AM_I 验证 */
    id = MPU6050_ReadReg(MPU6050_WHO_AM_I);

    /* 5. 配置量程（陀螺仪 ±2000°/s，加速度 ±2g） */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);

    /* 6. 采样率分频 + 数字低通滤波 */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);

    delay_ms(50);    /* 等待配置生效 */

    /* 7. 回读关键寄存器（调试用） */
    {
        uint8_t pwr1  = MPU6050_ReadReg(MPU6050_PWR_MGMT_1);
        uint8_t pwr2  = MPU6050_ReadReg(MPU6050_PWR_MGMT_2);
        uint8_t accel_c = MPU6050_ReadReg(MPU6050_ACCEL_CONFIG);
        uint8_t gyro_c  = MPU6050_ReadReg(MPU6050_GYRO_CONFIG);
        USART1_Printf("[MPU6050] reg: PWR1=0x%02X PWR2=0x%02X ACCEL_C=0x%02X GYRO_C=0x%02X\r\n",
                      pwr1, pwr2, accel_c, gyro_c);
    }

    return (id == MPU6050_ADDR_7BIT) ? 0 : id;
}

/* ---- 读原始数据（14 字节连续读 + 拼接）---- */
void MPU6050_GetRaw(int16_t *ax, int16_t *ay, int16_t *az,
                    int16_t *gx, int16_t *gy, int16_t *gz,
                    int16_t *temp_raw)
{
    uint8_t buf[14];

    MPU6050_ReadReg_Continue(MPU6050_ACCEL_XOUT_H, 14, buf);

    *ax       = (int16_t)(buf[0] << 8 | buf[1]);
    *ay       = (int16_t)(buf[2] << 8 | buf[3]);
    *az       = (int16_t)(buf[4] << 8 | buf[5]);
    *temp_raw = (int16_t)(buf[6] << 8 | buf[7]);
    *gx       = (int16_t)(buf[8] << 8 | buf[9]);
    *gy       = (int16_t)(buf[10] << 8 | buf[11]);
    *gz       = (int16_t)(buf[12] << 8 | buf[13]);
}

/* ---- 转物理量 ----
 * 加速度：g（重力倍数）
 * 陀螺仪：°/s（角速度）
 * 温度  ：°C
 */
void MPU6050_GetPhysical(float *ax_g, float *ay_g, float *az_g,
                         float *gx_dps, float *gy_dps, float *gz_dps,
                         float *temp_c)
{
    int16_t ax, ay, az, gx, gy, gz, t_raw;

    MPU6050_GetRaw(&ax, &ay, &az, &gx, &gy, &gz, &t_raw);

    *ax_g   = ax   / ACCEL_SCALE_2G;
    *ay_g   = ay   / ACCEL_SCALE_2G;
    *az_g   = az   / ACCEL_SCALE_2G;
    *gx_dps = gx   / GYRO_SCALE_2000DPS;
    *gy_dps = gy   / GYRO_SCALE_2000DPS;
    *gz_dps = gz   / GYRO_SCALE_2000DPS;
    /* 温度转换公式（数据手册）：(raw - 只读偏移) / 340 + 25 */
    *temp_c = (t_raw / 340.0f) + 25.0f;
}
