/**
  ******************************************************************************
  * @file    adaptive_threshold.c
  * @brief   自适应阈值模块实现
  *
  * 算法：
  *   - 基线学习：上电阻塞采 AT_LEARN_SECONDS 次，算均值 ± k*sigma 得初始阈值
  *   - EWMA 滑动更新：运行期每次调用都用 alpha 系数缓慢更新 baseline，
  *     让系统能适应昼夜温湿度缓慢变化
  *   - 异常检测：mq2 偏离 baseline + 温度快速上升 → 火灾；
  *               距离突变 → 入侵（AWAY/SLEEP 模式下）
  ******************************************************************************
  */
#include "adaptive_threshold.h"
#include "sensor_hub.h"
#include "usart.h"
#include "delay.h"
#include <math.h>

static AT_Baseline_t s_bl;

/* 学习进度回调（可选，main.c 注册用于 OLED 进度显示） */
static AT_ProgressCb_t s_progress_cb = NULL;

void AT_SetProgressCallback(AT_ProgressCb_t cb)
{
    s_progress_cb = cb;
}

/* ========== 学习期环形缓冲区 ========== */
#define AT_LEARN_MAX   30   /* 最大学习采样次数（>= AT_LEARN_SECONDS） */
static uint16_t s_buf_light[AT_LEARN_MAX];
static uint16_t s_buf_mq2[AT_LEARN_MAX];
static float    s_buf_temp[AT_LEARN_MAX];
static float    s_buf_humi[AT_LEARN_MAX];
static float    s_buf_dist[AT_LEARN_MAX];

/* ========== helpers ========== */
static float mean_u16(const uint16_t *buf, uint8_t n)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) sum += buf[i];
    return (float)sum / (float)n;
}
static float mean_f(const float *buf, uint8_t n)
{
    float sum = 0;
    for (uint8_t i = 0; i < n; i++) sum += buf[i];
    return sum / (float)n;
}
static float sigma_u16(const uint16_t *buf, uint8_t n, float mean)
{
    float s = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float d = (float)buf[i] - mean;
        s += d * d;
    }
    return sqrtf(s / (float)n);
}
static float sigma_f(const float *buf, uint8_t n, float mean)
{
    float s = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float d = buf[i] - mean;
        s += d * d;
    }
    return sqrtf(s / (float)n);
}

void AT_Init(void)
{
    s_bl.is_learned = 0;
}

/**
  * @brief  上电后调用：阻塞学习基线
  *         每秒采 1 次，共 AT_LEARN_SECONDS 次
  *         期间可在 main 里先开 LED 做"学习中"提示
  */
void AT_LearnBaseline(void)
{
    Sensor_Data_t *d = SensorHub_GetData();

    uint8_t n = AT_LEARN_SECONDS;
    if (n > AT_LEARN_MAX) n = AT_LEARN_MAX;

    /* ====== DHT11 预热：直接绕过节流，读 2 次丢弃 ======
       SensorHub_Update 内部 DHT11 节流 2~4s 才读一次，
       基线学习每秒调一次 Update，前几次 temp/humidity 会保持 0.0
       预热让第一次基线采样就有有效值 */
    USART1_Printf("[AT] DHT11 warming up 2 times...\r\n");
    for (uint8_t w = 0; w < 2; w++) {
        extern uint8_t DHT11_Read_Data(float *temp, float *humi);
        DHT11_Read_Data(&d->temperature, &d->humidity);
        USART1_Printf("[AT]   warmup %d: t=%.1f h=%.1f\r\n", w+1, d->temperature, d->humidity);
        delay_ms(2000);   /* DHT11 要求 ≥ 1.5s 读间隔 */
    }
    USART1_Printf("[AT] warmup done, baseline learn begins\r\n");

    /* 调试日志用英文：无 BOM UTF-8 源文件按 GBK 扫描时，个别汉字尾字节
       (如"开"=E5 BC 80 的 0x80) 会构成非法 GBK 首字节触发 #870-D，英文可根治 */
    USART1_Printf("[AT] baseline learning: %d s\r\n", n);

    for (uint8_t i = 0; i < n; i++)
    {
        SensorHub_Update();
        s_buf_light[i] = d->light_adc;
        s_buf_mq2[i]   = d->mq2_adc;
        s_buf_temp[i]  = d->temperature;
        s_buf_humi[i]  = d->humidity;
        s_buf_dist[i]  = d->distance;
        USART1_Printf("[AT] 采样 %2d/%d  light=%4d mq2=%4d temp=%.1f humi=%.1f\r\n",
                      i + 1, n,
                      s_buf_light[i], s_buf_mq2[i],
                      s_buf_temp[i], s_buf_humi[i]);
        if (s_progress_cb) s_progress_cb(i + 1, n);
        delay_ms(1000);
    }

    /* 计算均值 */
    float m_light = mean_u16(s_buf_light, n);
    float m_mq2   = mean_u16(s_buf_mq2, n);
    float m_temp  = mean_f(s_buf_temp, n);
    float m_humi  = mean_f(s_buf_humi, n);
    float m_dist  = mean_f(s_buf_dist, n);

    /* 计算标准差 */
    s_bl.light_sigma = (uint16_t)sigma_u16(s_buf_light, n, m_light);
    s_bl.mq2_sigma   = (uint16_t)sigma_u16(s_buf_mq2,   n, m_mq2);
    s_bl.temp_sigma  = sigma_f(s_buf_temp,  n, m_temp);
    s_bl.humi_sigma  = sigma_f(s_buf_humi,  n, m_humi);

    /* 保存 baseline */
    s_bl.light_baseline = (uint16_t)m_light;
    s_bl.mq2_baseline   = (uint16_t)m_mq2;
    s_bl.temp_baseline  = m_temp;
    s_bl.humi_baseline  = m_humi;

    /* EWMA 初始值 = baseline */
    s_bl.light_ewma = m_light;
    s_bl.mq2_ewma   = m_mq2;
    s_bl.temp_ewma  = m_temp;
    s_bl.humi_ewma  = m_humi;
    s_bl.dist_ewma  = m_dist;

    /* 动态阈值：烟雾触发 = baseline + k*sigma（最少 400，避免环境 sigma 很小时阈值太低） */
    uint16_t calc = (uint16_t)(m_mq2 + AT_SIGMA_K * (float)s_bl.mq2_sigma);
    s_bl.mq2_fire_threshold = calc > 400 ? calc : 400;

    /* MQ-2 基线异常高（> 2000）说明传感器未正常接入，
       禁用火灾检测，避免自然波动触发误报 */
    if (m_mq2 > 2000.0f) {
        s_bl.mq2_fire_threshold = 0xFFFF;   /* 永不可达 */
        USART1_Printf("[AT] WARN: MQ-2 baseline=%.0f too high, fire detection DISABLED\r\n", m_mq2);
    }

    s_bl.is_learned = 1;

    USART1_Printf("[AT] 基线学习完成\r\n");
    USART1_Printf("[AT] light_base=%4d sigma=%d\r\n", s_bl.light_baseline, s_bl.light_sigma);
    USART1_Printf("[AT] mq2_base=%4d sigma=%d fire_th=%4d\r\n",
                  s_bl.mq2_baseline, s_bl.mq2_sigma, s_bl.mq2_fire_threshold);
    USART1_Printf("[AT] temp_base=%.1f sigma=%.1f  humi_base=%.1f sigma=%.1f\r\n",
                  s_bl.temp_baseline, s_bl.temp_sigma,
                  s_bl.humi_baseline, s_bl.humi_sigma);
}

/**
  * @brief  运行期更新 EWMA（指数加权移动平均）
  *         每次主循环调用，慢慢吸收环境慢变化
  */
void AT_Update(uint16_t light, uint16_t mq2, float temp, float humi, float dist)
{
    if (!s_bl.is_learned) return;

    /* 用 alpha 系数缓慢靠拢新值，只对"小幅偏离"做吸收；
       大幅偏离（可能是真的异常）不被吸收，防止阈值被污染 */
    float diff_light = (float)light - s_bl.light_ewma;
    float diff_mq2   = (float)mq2   - s_bl.mq2_ewma;
    float diff_temp  = temp         - s_bl.temp_ewma;
    float diff_humi  = humi         - s_bl.humi_ewma;

    /* 烟雾：偏离超过 2*sigma 不吸收（可能是真的烟雾） */
    if (fabsf(diff_mq2) < 2.0f * (float)s_bl.mq2_sigma) {
        s_bl.mq2_ewma   += AT_EWMA_ALPHA * diff_mq2;
        /* 基线学习时已禁用（threshold=0xFFFF）时保持禁用，不要覆盖 */
        if (s_bl.mq2_fire_threshold != 0xFFFF) {
            /* 必须与基线学习期保持同一语义：阈值 = max(ewma + k*sigma, 400)。
               漏掉 400 下限时，安静环境下 ewma≈180、sigma≈7 会把阈值压到 ~201，
               而 2*sigma 吸收门（上面那个 if）又不让 203 这类值更新 ewma，
               于是阈值钉死在 201，MQ-2 自然抖动跨线就反复误报火灾。 */
            uint16_t calc = (uint16_t)(s_bl.mq2_ewma + (float)AT_SIGMA_K * (float)s_bl.mq2_sigma);
            s_bl.mq2_fire_threshold = calc > 400 ? calc : 400;
        }
    }

    /* 温湿度：偏离超过 3*sigma 不吸收 */
    if (fabsf(diff_temp) < 3.0f * s_bl.temp_sigma)  s_bl.temp_ewma  += AT_EWMA_ALPHA * diff_temp;
    if (fabsf(diff_humi) < 3.0f * s_bl.humi_sigma)  s_bl.humi_ewma  += AT_EWMA_ALPHA * diff_humi;

    /* 光照：昼夜会大幅变化，所以对光照用更强的 EWMA */
    s_bl.light_ewma += (AT_EWMA_ALPHA * 2.0f) * diff_light;

    /* 距离：alpha 与其他量一致。dist_ewma 未持久化到 Flash，
       掉电恢复后为 0，此时用首个有效样本直接播种，避免从 0 缓慢收敛 */
    if (dist > 0.0f) {
        if (s_bl.dist_ewma <= 0.0f) {
            s_bl.dist_ewma = dist;
        } else {
            s_bl.dist_ewma += AT_EWMA_ALPHA * (dist - s_bl.dist_ewma);
        }
    }
}

/**
  * @brief  火灾判断：烟雾 > 动态阈值 且 温度明显高于 baseline
  */
uint8_t AT_CheckFire(const Sensor_Data_t *data)
{
    if (!s_bl.is_learned) return 0;

    /* 烟雾超过动态阈值 */
    if (data->mq2_adc < s_bl.mq2_fire_threshold) return 0;

    /* 温度也明显高于 baseline（比 ewma 高 2*sigma，或绝对值 > 40°C）。
       sigma 下限保护：学习期温度恒定时 sigma=0，门槛退化为
       T >= ewma（刀刃条件，任何不小于基线的温度都算"异常"），
       叠加 MQ-2 预热漂移会造成持续火灾误报（断电实测复现）。
       故 margin 至少 1.5°C——真实火灾温升远超此值，不受影响。 */
    {
        float temp_margin = 2.0f * s_bl.temp_sigma;
        if (temp_margin < 1.5f) temp_margin = 1.5f;
        if (data->temperature < s_bl.temp_ewma + temp_margin &&
            data->temperature < 40.0f)
        {
            return 0;
        }
    }

    return 1;
}

/**
  * @brief  入侵判断：距离相对于上一次（主循环频率 500ms）突然变小
  */
uint8_t AT_CheckIntrusion(const Sensor_Data_t *data, float dist_prev, float dist_now)
{
    if (!s_bl.is_learned) return 0;
    if (data->distance <= 0.0f) return 0;

    /* 距离从远突然变近：变化量 > delta，且当前距离很近 */
    float delta = dist_prev - dist_now;
    if (delta > AT_DISTANCE_DELTA_CM && dist_now < 120.0f)
    {
        return 1;
    }

    /* 绝对阈值兜底：AWAY/SLEEP 模式下距离持续很近 */
    if (dist_now > 0.0f && dist_now < 30.0f)
    {
        return 1;
    }

    return 0;
}

const AT_Baseline_t *AT_GetBaseline(void)
{
    return &s_bl;
}
