/**
  ******************************************************************************
  * @file    pwm.c
  * @brief   TIM 双通道 PWM 驱动实现
  *
  * 默认配置：
  *   手机模式灯 TIM14：APB1 84MHz，预分频 84 -> 1MHz，周期 1000 -> 1kHz
  *   家居模式灯 TIM1： APB2 168MHz，预分频 168 -> 1MHz，周期 1000 -> 1kHz
  *
  * 注意：两个 LED 均为低电平点亮，因此输出极性配置为 Low。
  *      CCR 越大，低电平时间越长，LED 越亮。
  ******************************************************************************
  */
#include "pwm.h"

static uint16_t s_pwm_period = PWM_PERIOD;

/**
  * @brief  初始化两个 PWM 通道
  * @param  period: PWM 周期计数值（例如 1000）
  */
void PWM_Init(uint16_t period)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    s_pwm_period = period;

    /* ==================== 手机模式：TIM14_CH1 -> PF9 ==================== */
    RCC_AHB1PeriphClockCmd(PWM_PHONE_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(PWM_PHONE_TIM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = PWM_PHONE_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(PWM_PHONE_GPIO_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(PWM_PHONE_GPIO_PORT, PWM_PHONE_GPIO_SRC, PWM_PHONE_GPIO_AF);

    TIM_TimeBaseStructure.TIM_Period        = period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 84 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_PHONE_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = 0;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;
    TIM_OC1Init(PWM_PHONE_TIM, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(PWM_PHONE_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(PWM_PHONE_TIM, ENABLE);
    TIM_Cmd(PWM_PHONE_TIM, ENABLE);

    /* ==================== 家居模式：TIM1_CH4 -> PE14 ==================== */
    RCC_AHB1PeriphClockCmd(PWM_HOME_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(PWM_HOME_TIM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = PWM_HOME_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(PWM_HOME_GPIO_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(PWM_HOME_GPIO_PORT, PWM_HOME_GPIO_SRC, PWM_HOME_GPIO_AF);

    TIM_TimeBaseStructure.TIM_Period        = period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = 168 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_HOME_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = 0;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;
    TIM_OC4Init(PWM_HOME_TIM, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(PWM_HOME_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(PWM_HOME_TIM, ENABLE);

    /* TIM1 是高级定时器，必须使能主输出 MOE */
    TIM_Cmd(PWM_HOME_TIM, ENABLE);
    TIM_CtrlPWMOutputs(PWM_HOME_TIM, ENABLE);
}

/**
  * @brief  设置手机模式灯 PWM 占空比
  * @param  duty: 0 ~ PWM_PERIOD
  */
void PWM_Phone_SetDuty(uint16_t duty)
{
    if (duty > s_pwm_period)
    {
        duty = s_pwm_period;
    }
    TIM_SetCompare1(PWM_PHONE_TIM, duty);
}

/**
  * @brief  设置家居模式灯 PWM 占空比
  * @param  duty: 0 ~ PWM_PERIOD
  */
void PWM_Home_SetDuty(uint16_t duty)
{
    if (duty > s_pwm_period)
    {
        duty = s_pwm_period;
    }
    TIM_SetCompare4(PWM_HOME_TIM, duty);
}

/**
  * @brief  关闭手机模式灯
  */
void PWM_Phone_Off(void)
{
    TIM_SetCompare1(PWM_PHONE_TIM, 0);
}

/**
  * @brief  关闭家居模式灯
  */
void PWM_Home_Off(void)
{
    TIM_SetCompare4(PWM_HOME_TIM, 0);
}
