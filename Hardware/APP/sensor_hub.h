#ifndef __SENSOR_HUB_H
#define __SENSOR_HUB_H

#include "stm32f4xx.h"

typedef struct {
    uint16_t light_adc;     /* 光照 ADC，光照越强值越小 */
    uint16_t mq2_adc;       /* MQ-2 烟雾 ADC */
    float    temperature;   /* DHT11 温度，单位 °C */
    float    humidity;      /* DHT11 湿度，单位 % */
    float    distance;      /* HC-SR04 距离，单位 cm */
    uint8_t  tamper_flag;   /* MPU6050 防拆/震动检测标志，1=触发 */
} Sensor_Data_t;

void SensorHub_Init(void);
void SensorHub_Update(void);
Sensor_Data_t *SensorHub_GetData(void);
void SensorHub_Print(void);

#endif
