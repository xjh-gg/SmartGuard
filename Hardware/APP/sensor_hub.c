/**
  ******************************************************************************
  * @file    sensor_hub.c
  * @brief   传感器中枢：统一采集所有传感器数据
  *
  * 已接入硬件：
  *   - 光敏电阻 PF7 / ADC3_IN5（板载）
  *   - HC-SR04 超声波（Trig=PC8, Echo=PC6）
  *   - DHT11 温湿度 PG9（开发板自带）
  *   - MQ-2 烟雾 PA4 / ADC1_IN4（adc.h 的 USE_MQ2 控制，默认已启用）
  *
  * 注：MPU6050 防拆采样不走这里，由 main.c 的 TamperUpdate 独立高频读取
  ******************************************************************************
  */
#include "sensor_hub.h"
#include "adc.h"
#include "hcsr04.h"
#include "usart.h"
#include "delay.h"
#include "decision_engine.h"

#if USE_DHT11
  #include "dht11.h"
#endif

static Sensor_Data_t s_sensor_data;

/* DHT11 节流计数器：主循环 500ms 一次，DHT11 要求 ≥ 1.5s 读一次
   所以每 4 次循环（2 秒）读一次 DHT11 */
static uint8_t s_dht11_counter = 0;

void SensorHub_Init(void)
{
#if USE_DHT11
    DHT11_Init();
    /* 启动时先尝试读一次，失败也没关系（会保留默认值） */
    DHT11_Read_Data(&s_sensor_data.temperature, &s_sensor_data.humidity);
#else
    s_sensor_data.temperature = 25.0f;  /* 无 DHT11，默认值 */
    s_sensor_data.humidity    = 50.0f;
#endif
    s_sensor_data.mq2_adc     = 0;      /* 初值 0，Update 中按 USE_MQ2 决定是否真实采样 */
}

void SensorHub_Update(void)
{
    /* 1. 板载光敏电阻（每次都读，ADC 无节流） */
    s_sensor_data.light_adc = ADC_ReadPhotoresistor();

    /* 2. 超声波测距（每次都读，~100ms 超时可接受） */
    s_sensor_data.distance  = HCSR04_GetDistance();

    /* 3. DHT11 温湿度（节流：每 2 秒读一次） */
#if USE_DHT11
    s_dht11_counter++;
    if (s_dht11_counter >= 4)   /* 4 × 500ms = 2s */
    {
        s_dht11_counter = 0;
        /* DHT11_Read_Data 失败时不再覆盖输出参数，
           自动保留上一次成功读取的有效值 */
        DHT11_Read_Data(&s_sensor_data.temperature,
                       &s_sensor_data.humidity);
    }
#endif

    /* 4. MQ-2 烟雾：USE_MQ2=0（未接线）时 ADC_ReadMQ2 恒返回 0，
          DecisionEngine 的 CheckFire 见到 0 自动跳过；到货把 adc.h 宏置 1 即可 */
    s_sensor_data.mq2_adc = ADC_ReadMQ2();
}

Sensor_Data_t *SensorHub_GetData(void)
{
    return &s_sensor_data;
}

void SensorHub_Print(void)
{
    if (s_sensor_data.distance > 0.0f)
    {
        USART1_Printf("L:%4d D:%.1fcm T:%.1fC H:%.1f%%\r\n",
                      s_sensor_data.light_adc,
                      s_sensor_data.distance,
                      s_sensor_data.temperature,
                      s_sensor_data.humidity);
    }
    else
    {
        USART1_Printf("L:%4d D:ERR[%d] T:%.1fC H:%.1f%%\r\n",
                      s_sensor_data.light_adc,
                      g_hcsr04_err,
                      s_sensor_data.temperature,
                      s_sensor_data.humidity);
    }
}
