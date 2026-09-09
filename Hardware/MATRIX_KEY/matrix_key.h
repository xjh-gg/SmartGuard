/**
  ******************************************************************************
  * @file    matrix_key.h
  * @brief   4x4 矩阵键盘驱动头文件（列扫描 + 非阻塞状态机）
  *
  * 物理键盘布局（左 -> 右，上 -> 下）：
  *   第1行：1   2   3   A
  *   第2行：4   5   6   B
  *   第3行：7   8   9   C
  *   第4行：*   0   #   D
  *
  * 硬件连接（STM32F407ZE，列=推挽输出，行=上拉输入）：
  *   列输出 COL1 -> PB7     COL2 -> PA4     COL3 -> PG15    COL4 -> PC7
  *   行输入 ROW1 -> PC9(上拉)  ROW2 -> PB6(上拉)
  *           ROW3 -> PE6(上拉)  ROW4 -> PA8(上拉)
  *
  * 软件映射说明：
  *   硬件行列与软件扫描是转置关系（代码行=物理列，代码列=物理行），
  *   因此 key_map 按"物理列从左到右、每列从上到下"填入，保证
  *   按下物理按键后返回的字符与丝印一致。
  *
  * 使用方法：
  *   Matrix_Key_Init();                // 初始化 GPIO
  *   char key = Key_Scan();            // 非阻塞扫描，返回 '1'~'9','0','A'~'D','*','#'，无键=0
  ******************************************************************************
  */
#ifndef __MATRIX_KEY_H__
#define __MATRIX_KEY_H__

#include "stm32f4xx.h"

/* ========== 列线（输出，推挽） ========== */
#define COL1_PIN     GPIO_Pin_7
#define COL1_PORT    GPIOB
#define COL1_CLK     RCC_AHB1Periph_GPIOB

#define COL2_PIN     GPIO_Pin_4
#define COL2_PORT    GPIOA
#define COL2_CLK     RCC_AHB1Periph_GPIOA

#define COL3_PIN     GPIO_Pin_15
#define COL3_PORT    GPIOG
#define COL3_CLK     RCC_AHB1Periph_GPIOG

#define COL4_PIN     GPIO_Pin_7
#define COL4_PORT    GPIOC
#define COL4_CLK     RCC_AHB1Periph_GPIOC

/* ========== 行线（输入，上拉） ========== */
#define ROW1_PIN     GPIO_Pin_9
#define ROW1_PORT    GPIOC
#define ROW1_CLK     RCC_AHB1Periph_GPIOC

#define ROW2_PIN     GPIO_Pin_6
#define ROW2_PORT    GPIOB
#define ROW2_CLK     RCC_AHB1Periph_GPIOB

#define ROW3_PIN     GPIO_Pin_6
#define ROW3_PORT    GPIOE
#define ROW3_CLK     RCC_AHB1Periph_GPIOE

#define ROW4_PIN     GPIO_Pin_8
#define ROW4_PORT    GPIOA
#define ROW4_CLK     RCC_AHB1Periph_GPIOA

/* ========== 接口函数 ========== */
void Matrix_Key_Init(void);   /* 列推挽输出 + 行上拉输入，初始化完成后列全部拉高 */
char Key_Scan(void);          /* 非阻塞状态机扫描，返回 '1'~'9','0','A'~'D','*','#'，无键=0 */

#endif /* __MATRIX_KEY_H__ */
