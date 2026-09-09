/**
  ******************************************************************************
  * @file    led.c
  * @brief   LED 驱动程序（推挽输出）
  ******************************************************************************
  */
#include "led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = LED0_PIN | LED1_PIN;
    GPIO_Init(LED0_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LED2_PIN | LED3_PIN;
    GPIO_Init(LED2_PORT, &GPIO_InitStructure);

    LED_AllOff();
}

void LED_ON(uint8_t x)
{
    switch (x)
    {
        case 1: GPIO_ResetBits(LED0_PORT, LED0_PIN); break;
        case 2: GPIO_ResetBits(LED1_PORT, LED1_PIN); break;
        case 3: GPIO_ResetBits(LED2_PORT, LED2_PIN); break;
        case 4: GPIO_ResetBits(LED3_PORT, LED3_PIN); break;
        default: break;
    }
}

void LED_OFF(uint8_t x)
{
    switch (x)
    {
        case 1: GPIO_SetBits(LED0_PORT, LED0_PIN); break;
        case 2: GPIO_SetBits(LED1_PORT, LED1_PIN); break;
        case 3: GPIO_SetBits(LED2_PORT, LED2_PIN); break;
        case 4: GPIO_SetBits(LED3_PORT, LED3_PIN); break;
        default: break;
    }
}

void LED_Toggle(uint8_t x)
{
    switch (x)
    {
        case 1: GPIO_ToggleBits(LED0_PORT, LED0_PIN); break;
        case 2: GPIO_ToggleBits(LED1_PORT, LED1_PIN); break;
        case 3: GPIO_ToggleBits(LED2_PORT, LED2_PIN); break;
        case 4: GPIO_ToggleBits(LED3_PORT, LED3_PIN); break;
        default: break;
    }
}

void LED_AllOn(void)
{
    GPIO_ResetBits(LED0_PORT, LED0_PIN | LED1_PIN);
    GPIO_ResetBits(LED2_PORT, LED2_PIN | LED3_PIN);
}

void LED_AllOff(void)
{
    GPIO_SetBits(LED0_PORT, LED0_PIN | LED1_PIN);
    GPIO_SetBits(LED2_PORT, LED2_PIN | LED3_PIN);
}
