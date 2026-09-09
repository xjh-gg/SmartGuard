/**
  ******************************************************************************
  * @file    mq2.c
  * @brief   MQ-2 烟雾/可燃气体传感器驱动实现
  *
  * 硬件连接（STM32F407ZE）：
  *   MQ-2 AO -> PA6 / ADC1_IN6
  *   MQ-2 VCC -> 5V
  *   MQ-2 GND -> GND
  *
  * 说明：
  *   MQ-2 模拟输出电压随气体浓度变化，通过 ADC 采样得到 0~4095 的值。
  *   浓度越高，输出电压通常越低（取决于模块设计），因此 ADC 值越小。
  ******************************************************************************
  */
#include "mq2.h"
#include "adc.h"

static uint16_t s_mq2_baseline = 0;

/**
  * @brief  MQ-2 初始化
  * @note   ADC 硬件已在 ADC_Hardware_Init() 中初始化，此处无需重复配置 GPIO。
  *         上电时读取一次当前环境基线。
  */
void MQ2_Init(void)
{
    /* 延时等待传感器预热稳定（MQ-2 建议预热 20s 以上，此处只读一次基线） */
    s_mq2_baseline = ADC_ReadMQ2();
}

/**
  * @brief  读取 MQ-2 当前 ADC 值
  * @retval 0 ~ 4095
  */
uint16_t MQ2_Read(void)
{
    return ADC_ReadMQ2();
}

/**
  * @brief  获取 MQ-2 环境基线值
  */
uint16_t MQ2_Get_Baseline(void)
{
    return s_mq2_baseline;
}
