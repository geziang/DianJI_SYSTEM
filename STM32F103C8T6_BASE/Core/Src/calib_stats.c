/*
 * 标定小样本统计库实现。纯函数、零 HAL 依赖，可在 PC 与 MCU 两端同一份编译。
 * 排序使用插入排序（n <= 64 时确定且足够快），排序在栈上副本进行，不改入参。
 */
#include "calib_stats.h"

#include <math.h>

#define CALIB_STATS_PI (3.1415926535897932f)
#define CALIB_STATS_TWO_PI (2.0f * CALIB_STATS_PI)

static uint8_t calib_stats_copy(const float *src, float *dst, uint16_t n)
{
  uint16_t i;
  if ((src == 0) || (dst == 0) || (n == 0U) || (n > CALIB_STATS_MAX_SAMPLES))
  {
    return 0U;
  }
  for (i = 0U; i < n; ++i)
  {
    dst[i] = src[i];
  }
  return 1U;
}

/* 升序插入排序。 */
static void calib_stats_insertion_sort(float *x, uint16_t n)
{
  uint16_t i;
  for (i = 1U; i < n; ++i)
  {
    float key = x[i];
    int16_t j = (int16_t)i - 1;
    while ((j >= 0) && (x[j] > key))
    {
      x[j + 1] = x[j];
      --j;
    }
    x[j + 1] = key;
  }
}

/* 闭区间 [lo,hi] 的中位数。 */
static float calib_stats_range_median(const float *x, uint16_t lo, uint16_t hi)
{
  uint16_t count = (uint16_t)(hi - lo + 1U);
  uint16_t mid = (uint16_t)(lo + count / 2U);
  if ((count & 1U) != 0U)
  {
    return x[mid];
  }
  return 0.5f * (x[mid - 1U] + x[mid]);
}

float calib_trimmed_mean(const float *x, uint16_t n, uint16_t drop_each)
{
  float work[CALIB_STATS_MAX_SAMPLES];
  uint16_t lo, hi, k;
  float sum = 0.0f;
  uint16_t count;

  if (calib_stats_copy(x, work, n) == 0U)
  {
    return 0.0f;
  }
  lo = drop_each;
  if ((uint16_t)(2U * drop_each) >= n)
  {
    return 0.0f;
  }
  hi = (uint16_t)(n - 1U - drop_each);
  calib_stats_insertion_sort(work, n);
  for (k = lo; k <= hi; ++k)
  {
    sum += work[k];
  }
  count = (uint16_t)(hi - lo + 1U);
  return sum / (float)count;
}

void calib_median_iqr(const float *x, uint16_t n, float *median, float *iqr)
{
  float work[CALIB_STATS_MAX_SAMPLES];
  float q1, q3;

  if ((median == 0) || (iqr == 0))
  {
    return;
  }
  *median = 0.0f;
  *iqr = 0.0f;
  if (calib_stats_copy(x, work, n) == 0U)
  {
    return;
  }
  calib_stats_insertion_sort(work, n);
  *median = calib_stats_range_median(work, 0U, (uint16_t)(n - 1U));
  /* Tukey 箱线：n 为奇数时上下半段都包含整体中位数。 */
  q1 = calib_stats_range_median(work, 0U, (uint16_t)((n - 1U) / 2U));
  q3 = calib_stats_range_median(work, (uint16_t)(n / 2U), (uint16_t)(n - 1U));
  *iqr = q3 - q1;
}

calib_line_t calib_least_squares(const float *x, const float *y, uint16_t n)
{
  calib_line_t result;
  float x_mean = 0.0f;
  float y_mean = 0.0f;
  float sxx = 0.0f;
  float sxy = 0.0f;
  float syy = 0.0f;
  uint16_t i;

  result.slope = 0.0f;
  result.intercept = 0.0f;
  result.r2 = -1.0f;
  if ((x == 0) || (y == 0) || (n < 2U))
  {
    return result;
  }
  for (i = 0U; i < n; ++i)
  {
    x_mean += x[i];
    y_mean += y[i];
  }
  x_mean /= (float)n;
  y_mean /= (float)n;
  /* 中心化公式，避免大数相减吃掉有效位。 */
  for (i = 0U; i < n; ++i)
  {
    float dx = x[i] - x_mean;
    float dy = y[i] - y_mean;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  if (sxx <= 0.0f)
  {
    return result;
  }
  result.slope = sxy / sxx;
  result.intercept = y_mean - result.slope * x_mean;
  if (syy > 0.0f)
  {
    float r2 = (sxy * sxy) / (sxx * syy);
    result.r2 = (r2 > 1.0f) ? 1.0f : r2;
  }
  return result;
}

float calib_circular_mean(const float *ang, uint16_t n)
{
  float sum_sin = 0.0f;
  float sum_cos = 0.0f;
  uint16_t i;
  if ((ang == 0) || (n == 0U))
  {
    return 0.0f;
  }
  for (i = 0U; i < n; ++i)
  {
    sum_sin += sinf(ang[i]);
    sum_cos += cosf(ang[i]);
  }
  return atan2f(sum_sin, sum_cos);
}

float calib_circular_std(const float *ang, uint16_t n)
{
  float sum_sin = 0.0f;
  float sum_cos = 0.0f;
  float r_length;
  uint16_t i;
  if ((ang == 0) || (n == 0U))
  {
    return CALIB_STATS_PI;
  }
  for (i = 0U; i < n; ++i)
  {
    sum_sin += sinf(ang[i]);
    sum_cos += cosf(ang[i]);
  }
  r_length = sqrtf(sum_sin * sum_sin + sum_cos * sum_cos) / (float)n;
  if (r_length >= 1.0f)
  {
    return 0.0f;
  }
  if (r_length <= 1e-6f)
  {
    return CALIB_STATS_PI; /* 完全弥散，无可信平均方向 */
  }
  return sqrtf(-2.0f * logf(r_length));
}

float calib_angle_unwrap(float prev, float now)
{
  float diff = now - prev;
  while (diff > CALIB_STATS_PI)
  {
    diff -= CALIB_STATS_TWO_PI;
  }
  while (diff < -CALIB_STATS_PI)
  {
    diff += CALIB_STATS_TWO_PI;
  }
  return prev + diff;
}

/* ============ S7 低速动态 R 块统计（SPEC-RUN FR-1.2） ============ */

void calib_s7_dyn_r_init(calib_s7_dyn_r_accum_t *acc,
                         const calib_s7_dyn_r_cfg_t *cfg)
{
  uint16_t i;

  if (acc == 0)
  {
    return;
  }
  acc->skip_remaining_ms = (cfg != 0) ? cfg->skip_ms : 0U;
  acc->block_ms_accum = 0U;
  acc->block_samples = 0U;
  acc->block_count = 0U;
  acc->sample_count = 0UL;
  acc->block_v_sum = 0.0f;
  acc->block_i_sum = 0.0f;
  for (i = 0U; i < CALIB_STATS_S7_MAX_BLOCKS; ++i)
  {
    acc->block_mean_r[i] = 0.0f;
  }
}

void calib_s7_dyn_r_feed(calib_s7_dyn_r_accum_t *acc,
                         const calib_s7_dyn_r_cfg_t *cfg,
                         float vq_v, float iq_a)
{
  uint16_t capacity;

  if ((acc == 0) || (cfg == 0) || (cfg->block_ms == 0U))
  {
    return;
  }

  /* 建立段：转速/电流未稳的样本不计入（FR-1.2）。 */
  if (acc->skip_remaining_ms > 0U)
  {
    acc->skip_remaining_ms--;
    return;
  }

  /* 样本纳入门：|Iq| 过门才累计（与 S7 工作点联动，防形同虚设）。 */
  if (((iq_a >= cfg->min_iq_a) || (iq_a <= -cfg->min_iq_a)) && (iq_a != 0.0f))
  {
    acc->block_v_sum += vq_v;
    acc->block_i_sum += iq_a;
    acc->block_samples++;
    acc->sample_count++;
  }

  acc->block_ms_accum++;
  if (acc->block_ms_accum >= cfg->block_ms)
  {
    capacity = (cfg->max_blocks < CALIB_STATS_S7_MAX_BLOCKS)
                   ? cfg->max_blocks
                   : CALIB_STATS_S7_MAX_BLOCKS;
    /* 空块丢弃；有样本块结算 ratio-of-means 后入数组。 */
    if ((acc->block_samples > 0U) && (acc->block_count < capacity) &&
        (acc->block_i_sum != 0.0f))
    {
      acc->block_mean_r[acc->block_count] = acc->block_v_sum / acc->block_i_sum;
      acc->block_count++;
    }
    acc->block_ms_accum = 0U;
    acc->block_samples = 0U;
    acc->block_v_sum = 0.0f;
    acc->block_i_sum = 0.0f;
  }
}

uint8_t calib_s7_dyn_r_finalize(const calib_s7_dyn_r_accum_t *acc,
                                float *dyn_r_out)
{
  uint16_t drop_each;

  if ((acc == 0) || (dyn_r_out == 0) || (acc->block_count == 0U))
  {
    return 0U;
  }
  /* 块数足够（>=5）才截尾：两端各丢 1 块；否则退化为全块均值。 */
  drop_each = (acc->block_count >= 5U) ? 1U : 0U;
  *dyn_r_out = calib_trimmed_mean(acc->block_mean_r, acc->block_count, drop_each);
  return 1U;
}
