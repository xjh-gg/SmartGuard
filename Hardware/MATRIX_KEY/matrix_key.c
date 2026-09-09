/**
  ******************************************************************************
  * @file    matrix_key.c
  * @brief   4x4 矩阵键盘驱动程序（列扫描方式，带消抖与松手等待）
  *
  * 扫描原理：
  *   列线设为推挽输出，行线设为上拉输入。
  *   依次将每一列拉低，其余列拉高；若某一行被读为低电平，说明该 (行,列)
  *   位置有键按下。通过 10ms 消抖 + 松手等待，保证一次按下仅返回一次键值。
  *
  * 按键映射关系（已做转置补偿）：
  *   物理(row,col) -> 代码扫描到(r=col, c=row)
  *   因此 key_map[r][c] = expected[c][r]，即对期望矩阵做转置。
  ******************************************************************************
  */
#include "matrix_key.h"

/* ========== 简易毫秒延时（矩阵键盘内部自用，避免依赖外部 delay） ========== */
static void MatrixKey_DelayMs(uint32_t ms)
{
    volatile uint32_t n;
    while (ms--)
    {
        for (n = 0; n < 16800; n++);
    }
}

/* ============================================================
 *  按键映射表（物理丝印 123A / 456B / 789C / *0#D）
 *  由于硬件行列与代码扫描呈转置关系，key_map 的"一行"
 *  对应物理键盘的"一列"（从上到下四个键）。
 * ============================================================ */
static const char key_map[4][4] = {
    {'1', '4', '7', '*'},   /* 代码行0 -> 物理第1列（最左列）：1,4,7,* */
    {'2', '5', '8', '0'},   /* 代码行1 -> 物理第2列       ：2,5,8,0 */
    {'3', '6', '9', '#'},   /* 代码行2 -> 物理第3列       ：3,6,9,# */
    {'A', 'B', 'C', 'D'}    /* 代码行3 -> 物理第4列（最右列）：A,B,C,D */
};

/* ========== 列/行 端口与引脚查表数组 ========== */
static GPIO_TypeDef* COL_PORT[4] = {COL1_PORT, COL2_PORT, COL3_PORT, COL4_PORT};
static const uint16_t COL_PIN[4]  = {COL1_PIN,  COL2_PIN,  COL3_PIN,  COL4_PIN};
static const uint32_t COL_CLK[4]  = {COL1_CLK,  COL2_CLK,  COL3_CLK,  COL4_CLK};

static GPIO_TypeDef* ROW_PORT[4] = {ROW1_PORT, ROW2_PORT, ROW3_PORT, ROW4_PORT};
static const uint16_t ROW_PIN[4]  = {ROW1_PIN,  ROW2_PIN,  ROW3_PIN,  ROW4_PIN};
static const uint32_t ROW_CLK[4]  = {ROW1_CLK,  ROW2_CLK,  ROW3_CLK,  ROW4_CLK};

/**
  * @brief  矩阵键盘 GPIO 初始化
  *         列（COL1~COL4）：推挽输出，初始全部拉高（无选中）
  *         行（ROW1~ROW4）：带上拉输入（未按下时为高电平，按下被列拉低）
  */
void Matrix_Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    int i;
    uint32_t all_clk = 0;

    /* 一次性开启所有用到的 GPIO 时钟 */
    for (i = 0; i < 4; i++)
    {
        all_clk |= COL_CLK[i];
        all_clk |= ROW_CLK[i];
    }
    RCC_AHB1PeriphClockCmd(all_clk, ENABLE);

    /* ---------- 列：推挽输出，初始拉高 ---------- */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    for (i = 0; i < 4; i++)
    {
        GPIO_InitStruct.GPIO_Pin = COL_PIN[i];
        GPIO_Init(COL_PORT[i], &GPIO_InitStruct);
        GPIO_SetBits(COL_PORT[i], COL_PIN[i]);
    }

    /* ---------- 行：带上拉输入 ---------- */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    for (i = 0; i < 4; i++)
    {
        GPIO_InitStruct.GPIO_Pin = ROW_PIN[i];
        GPIO_Init(ROW_PORT[i], &GPIO_InitStruct);
    }
}

/**
  * @brief  扫描一次 4x4 矩阵键盘（单次调用最多返回一个有效键值）
  * @retval 按键对应的字符：'1'~'9','0','A'~'D','*','#'；无键按下返回 0
  *
  * 说明：
  *   1. 逐列拉低 -> 读行电平 -> 命中后 10ms 消抖二次确认
  *   2. 松手等待（避免长按连续触发）
  *   3. 退出前所有列重新拉高，保证下一次扫描一致
  */
char Key_Scan(void)
{
    int col, row, i;
    char key_val = 0;

    /* 逐列扫描：选中列 -> 拉低；未选中 -> 拉高 */
    for (col = 0; col < 4; col++)
    {
        /* 先把 4 列全部拉高，再把当前列拉低（安全切换） */
        for (i = 0; i < 4; i++)
            GPIO_SetBits(COL_PORT[i], COL_PIN[i]);

        GPIO_ResetBits(COL_PORT[col], COL_PIN[col]);
        MatrixKey_DelayMs(1);   /* 等列线电平稳定 */

        /* 遍历 4 条行线，看哪条被拉低 */
        for (row = 0; row < 4; row++)
        {
            if (GPIO_ReadInputDataBit(ROW_PORT[row], ROW_PIN[row]) == Bit_RESET)
            {
                /* 消抖：10ms 后再次判定仍为低才算有效按下 */
                MatrixKey_DelayMs(10);
                if (GPIO_ReadInputDataBit(ROW_PORT[row], ROW_PIN[row]) == Bit_RESET)
                {
                    /* 查表得到该位置对应的字符 */
                    key_val = key_map[row][col];

                    /* 松手等待：用户手指抬起后才允许返回 */
                    while (GPIO_ReadInputDataBit(ROW_PORT[row], ROW_PIN[row]) == Bit_RESET);
                    MatrixKey_DelayMs(10);   /* 松手消抖，避免弹片抖动 */

                    /* 恢复所有列为高电平，保持扫描空闲态 */
                    for (i = 0; i < 4; i++)
                        GPIO_SetBits(COL_PORT[i], COL_PIN[i]);

                    return key_val;
                }
            }
        }
    }

    /* 无按键：确保所有列都回到高电平 */
    for (i = 0; i < 4; i++)
        GPIO_SetBits(COL_PORT[i], COL_PIN[i]);

    return 0;
}
