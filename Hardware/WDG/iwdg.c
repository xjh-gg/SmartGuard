/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   STM32F407ZE 独立看门狗（IWDG）驱动
  *
  * 实现完全参考老师课件《22_看门狗.md》+ ST 官方库：
  *   - IWDG_Enable() 自动开 LSI，不需手动开
  *   - 不需等待 PVU/RVU（老师例程没做）
  *   - 直接写寄存器，避开 ST 库 assert_param（Debug 下会死循环）
  *
  * 参数计算（2s 超时，LSI 32KHz）：
  *   Prescaler = 256 → 32000/256 = 125Hz（每 tick 8ms）
  *   RLR = 249       → (249+1) × 8ms = 2000ms = 2s
  ******************************************************************************
  */
#include "iwdg.h"

void IWDG_Init(void)
{
    /* 1. 解锁：往 IWDG_KR 写 0x5555 */
    IWDG->KR = 0x5555;

    /* 2. 预分频：256 → 每 tick 8ms */
    IWDG->PR = IWDG_Prescaler_256;

    /* 3. 重载值：249 → (249+1)×8ms = 2000ms */
    IWDG->RLR = 249;

    /* 4. 喂一次狗：写 0xAAAA */
    IWDG->KR = 0xAAAA;

    /* 5. 启动看门狗：写 0xCCC（自动开 LSI） */
    IWDG->KR = 0xCCCC;
}

void IWDG_Feed(void)
{
    IWDG->KR = 0xAAAA;
}
