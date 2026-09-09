/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "delay.h"      /* SysTick_Update 更新 1ms 节拍 */
#include "usart.h"      /* USART1_RxHandler 接收中断处理 */
#if USE_RTC
#include "rtc.h"        /* RTC_IRQ_Callback 秒中断处理 */
#endif

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  * @note   每 1ms 进入一次：更新 g_tick 节拍 + 递减阻塞延时计数（delay.c）
  */
void SysTick_Handler(void)
{
    SysTick_Update();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/* ========================================================================== */
/* USART1 全局中断服务函数                                                    */
/*   触发条件：RXNE 标志置位（收到一个字节）                                   */
/*   处理逻辑：调用 USART1_RxHandler() 把字节写入环形缓冲区                    */
/*             读 DR 自动清 RXNE；ORE 溢出读 SR+DR 清除                       */
/*   注意：中断服务函数名必须与启动文件向量表一致（USART1_IRQn -> USART1_IRQHandler） */
/* ========================================================================== */
void USART1_IRQHandler(void)
{
    USART1_RxHandler();
}

/* ========================================================================== */
/* USART2 全局中断服务函数                                                    */
/*   触发条件：RXNE 标志置位（收到一个字节）                                   */
/*   处理逻辑：调用 USART2_RxHandler() 把字节写入独立环形缓冲区                */
/* ========================================================================== */
void USART2_IRQHandler(void)
{
    USART2_RxHandler();
}

/* ========================================================================== */
/* USART3 全局中断服务函数                                                    */
/* ========================================================================== */
void USART3_IRQHandler(void)
{
    USART3_RxHandler();
}

#if USE_RTC
/* ========================================================================== */
/* RTC Wakeup Timer 全局中断服务函数 —— 每秒触发一次                          */
/*   触发条件：RTC Wakeup Timer 倒计时到 0（wakeup_clk=ck_spre=1Hz，计数器=0）*/
/*   EXTI Line22 自动置位 → 进入此中断                                        */
/*   处理逻辑：调用 RTC_IRQ_Callback() 清标志、+1秒计数、调用户回调            */
/*   入口名必须与 startup_stm32f40_41xxx.s 向量表一致：RTC_WKUP_IRQHandler    */
/* ========================================================================== */
void RTC_WKUP_IRQHandler(void)
{
    RTC_IRQ_Callback();
}
#endif /* USE_RTC */

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
