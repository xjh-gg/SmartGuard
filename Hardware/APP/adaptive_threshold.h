/**
  ******************************************************************************
  * @file    adaptive_threshold.h
  * @brief   自适应阈值模块：上电基线学习 + 滑动窗口 + 动态告警阈值
  *
  * 原理：
  *   1. 上电进入基线学习期（默认 20 秒，每秒 1 次采样），填充环形缓冲区
  *   2. 学习期结束后计算均值 baseline 和标准差 sigma
  *   3. 告警阈值 = baseline + k * sigma（k 由 AT_SIGMA_K 宏控制，默认 3）
  *   4. 运行期用 EWMA（指数加权移动平均）缓慢更新 baseline，
  *      使系统能慢慢适应环境缓慢变化（如昼夜温湿度变化）
  *   5. 距离 / 光照使用"相对变化率"检测，烟雾 / 温湿度使用"绝对偏离 baseline"
  *
  * 为什么不用固定阈值：
  *   - 不同房间 / 不同时间，烟雾基线值差异巨大
  *   - 夏季 vs 冬季，温度基线完全不同
  *   - 自适应让系统第一次用就自动"适应当地环境"
  ******************************************************************************
  */
#ifndef __ADAPTIVE_THRESHOLD_H__
#define __ADAPTIVE_THRESHOLD_H__

#include "stm32f4xx.h"
#include "sensor_hub.h"

/* ========== 可调参数 ========== */
#define AT_LEARN_SECONDS        20      /* 基线学习期（秒），建议 >= 10 */
#define AT_SIGMA_K              3       /* 告警触发系数：偏离 baseline 几个 sigma 才算异常 */
#define AT_DISTANCE_DELTA_CM    30.0f   /* 距离变化触发入侵告警的最小变化量（cm） */
#define AT_TEMP_RISE_PER_MIN    3.0f    /* 火灾辅助：1 分钟内温度上升 >= 3°C 算异常 */
#define AT_EWMA_ALPHA           0.01f   /* EWMA 学习率：baseline 向新值靠拢的速度（0~1） */

/* ========== 基线数据结构 ========== */
typedef struct {
    /* 学习期得到的初始基线 */
    uint16_t light_baseline;
    uint16_t mq2_baseline;
    float    temp_baseline;
    float    humi_baseline;

    /* 计算得到的 sigma（标准差） */
    uint16_t light_sigma;
    uint16_t mq2_sigma;
    float    temp_sigma;
    float    humi_sigma;

    /* 运行期 EWMA 动态基线（缓慢更新） */
    float    light_ewma;
    float    mq2_ewma;
    float    temp_ewma;
    float    humi_ewma;
    float    dist_ewma;             /* 距离 EWMA 基线：ML dist_dev 特征的减数 */

    /* 告警阈值（由 baseline + k*sigma 得到） */
    uint16_t mq2_fire_threshold;    /* 烟雾 > 此值 → 触发火灾告警（需同时满足温度条件） */

    uint8_t  is_learned;            /* 1=学习完成，0=学习中 */
} AT_Baseline_t;

/* ========== 接口函数 ========== */
void AT_Init(void);
void AT_LearnBaseline(void);                      /* 上电后阻塞执行基线学习 */

/* 学习进度回调：由 main.c 注册（OLED 进度条用），AT_LearnBaseline 每采 1 次调一次 */
typedef void (*AT_ProgressCb_t)(uint8_t done, uint8_t total);
void AT_SetProgressCallback(AT_ProgressCb_t cb);
void AT_Update(uint16_t light, uint16_t mq2, float temp, float humi, float dist); /* 每次主循环调用，更新 EWMA */

/* 告警判断：返回 1=触发，0=未触发 */
uint8_t AT_CheckFire(const Sensor_Data_t *data);
uint8_t AT_CheckIntrusion(const Sensor_Data_t *data, float dist_prev, float dist_now);

const AT_Baseline_t *AT_GetBaseline(void);

#endif /* __ADAPTIVE_THRESHOLD_H__ */
