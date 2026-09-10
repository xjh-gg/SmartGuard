/**
  ******************************************************************************
  * @file    adc.c
  * @brief   ADC 多通道采样驱动实现
  *
  * 支持：
  *   ADC1_IN5 -> PA5（电位器）
  *   ADC3_IN5 -> PF7（光敏电阻，板载）
  *
  * 转换公式：
  *   Vref = 3.3V，12 位分辨率 -> 0 ~ 4095
  *   电压值(V) = ADC_Value * 3.3 / 4095
  ******************************************************************************
  */
#include "adc.h"

/**
  * @brief  初始化 ADC 相关 GPIO（模拟输入）
  */
static void ADC_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 只初始化有硬件的通道：PA5 电位器 + PF7 光敏电阻 */
    RCC_AHB1PeriphClockCmd(POT_GPIO_CLK | LIGHT_GPIO_CLK
#if USE_MQ2
                           | MQ2_GPIO_CLK
#endif
                           , ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    /* PA5 电位器 */
    GPIO_InitStructure.GPIO_Pin = POT_GPIO_PIN;
    GPIO_Init(POT_GPIO_PORT, &GPIO_InitStructure);

    /* PF7 板载光敏电阻 */
    GPIO_InitStructure.GPIO_Pin = LIGHT_GPIO_PIN;
    GPIO_Init(LIGHT_GPIO_PORT, &GPIO_InitStructure);

#if USE_MQ2
    /* PA4 MQ-2 烟雾模拟输入（USE_MQ2=1 时启用，MQ2 与电位器共用 ADC1） */
    GPIO_InitStructure.GPIO_Pin = MQ2_GPIO_PIN;
    GPIO_Init(MQ2_GPIO_PORT, &GPIO_InitStructure);
#endif
}

/**
  * @brief  配置 ADC 通用参数（ADC1/ADC2/ADC3 共享）
  */
static void ADC_Common_Config(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
}

/**
  * @brief  初始化单个 ADC 外设
  */
static void ADC_Periph_Init(ADC_TypeDef *ADCx)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode         = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode   = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion      = 1;
    ADC_Init(ADCx, &ADC_InitStructure);

    ADC_Cmd(ADCx, ENABLE);
}

/**
  * @brief  初始化 ADC1（电位器）与 ADC3（光敏电阻）
  */
void ADC_Hardware_Init(void)
{
    RCC_APB2PeriphClockCmd(POT_ADC_CLK | LIGHT_ADC_CLK, ENABLE);

    ADC_GPIO_Init();
    ADC_Common_Config();

    ADC_Periph_Init(POT_ADC);   /* ADC1 -> PA5 电位器 */
    ADC_Periph_Init(LIGHT_ADC); /* ADC3 -> PF7 光敏电阻 */
}

/**
  * @brief  读取指定 ADC 通道
  * @param  ADCx: ADC 外设
  * @param  channel: ADC 通道号
  * @retval 0 ~ 4095
  */
static uint16_t ADC_ReadChannel(ADC_TypeDef *ADCx, uint8_t channel)
{
    ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime_480Cycles);

    ADC_SoftwareStartConv(ADCx);

    while (ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET)
    {
    }

    return ADC_GetConversionValue(ADCx);
}

/**
  * @brief  读取电位器 ADC 值（PA5 / ADC1_IN5）
  */
uint16_t ADC_ReadPotentiometer(void)
{
    return ADC_ReadChannel(POT_ADC, POT_ADC_CHANNEL);
}

/**
  * @brief  读取光敏电阻 ADC 值（PF7 / ADC3_IN5）
  * @note   板载光敏电阻：光照越强，ADC 值越小
  */
uint16_t ADC_ReadPhotoresistor(void)
{
    return ADC_ReadChannel(LIGHT_ADC, LIGHT_ADC_CHANNEL);
}

/**
  * @brief  读取 MQ-2 烟雾传感器 ADC 值（PA4 / ADC1_IN4）
  * @note   由 adc.h 的 USE_MQ2 控制：未接线时恒返回 0，
  *         DecisionEngine 的 CheckFire 见到 0 会自动跳过火灾判断。
  */
uint16_t ADC_ReadMQ2(void)
{
#if USE_MQ2
    return ADC_ReadChannel(MQ2_ADC, MQ2_ADC_CHANNEL);
#else
    return 0;
#endif
}
