#include "servo.h"

/* TODO: 根据实际接线配置 PWM 引脚，例如 PA8/TIM1_CH1 */

void SERVO_Init(void)
{
    /* TODO: 配置 PWM 输出，周期 20ms（50Hz），脉宽 0.5~2.5ms */
}

void SERVO_SetAngle(uint8_t angle)
{
    /* TODO: 把角度 0~180 映射到对应占空比 */
}
