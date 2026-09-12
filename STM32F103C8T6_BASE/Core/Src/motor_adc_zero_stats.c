/*
 * 失能态 ADC 零偏样本的稳健统计实现。
 *
 * 采用中位数 + MAD 识别孤立尖峰：
 *   limit = max(12, ceil(4.5 * MAD))
 *   abs(sample - median) > limit -> outlier
 * 调用者仍需使用过滤后的 peak-to-peak 与原有 32 code 门限比较，并额外
 * 检查有效样本数；原始 min/max/peak-to-peak 始终保留用于量程保护和诊断。
 */
#include "motor_adc_zero_stats.h"

#define MOTOR_ADC_ZERO_STATS_OUTLIER_FLOOR_RAW (12U)
#define MOTOR_ADC_ZERO_STATS_MAD_SCALE_NUM     (9U)
#define MOTOR_ADC_ZERO_STATS_MAD_SCALE_DEN     (2U)

static uint16_t motor_adc_zero_stats_sorted[MOTOR_ADC_ZERO_STATS_MAX_SAMPLES];
static uint16_t motor_adc_zero_stats_deviation[MOTOR_ADC_ZERO_STATS_MAX_SAMPLES];

static void motor_adc_zero_stats_sort(uint16_t *values, uint16_t count)
{
  uint16_t i;

  for (i = 1U; i < count; ++i)
  {
    uint16_t key = values[i];
    uint16_t j = i;

    while ((j > 0U) && (values[j - 1U] > key))
    {
      values[j] = values[j - 1U];
      --j;
    }
    values[j] = key;
  }
}

static uint16_t motor_adc_zero_stats_median_sorted(const uint16_t *values,
                                                   uint16_t count)
{
  uint16_t middle;

  middle = (uint16_t)(count / 2U);
  if ((count & 1U) != 0U)
  {
    return values[middle];
  }
  return (uint16_t)(((uint32_t)values[middle - 1U] + values[middle]) / 2U);
}

uint8_t motor_adc_zero_stats_compute(const uint16_t *samples,
                                     uint16_t count,
                                     motor_adc_zero_stats_t *stats)
{
  uint32_t raw_sum;
  uint32_t filtered_sum;
  uint16_t raw_min;
  uint16_t raw_max;
  uint16_t filtered_min;
  uint16_t filtered_max;
  uint16_t median;
  uint16_t mad;
  uint16_t outlier_limit;
  uint16_t outlier_count;
  uint16_t valid_count;
  uint16_t i;

  if ((samples == 0) || (stats == 0) ||
      (count == 0U) || (count > MOTOR_ADC_ZERO_STATS_MAX_SAMPLES))
  {
    return 0U;
  }

  raw_sum = 0U;
  raw_min = samples[0];
  raw_max = samples[0];
  for (i = 0U; i < count; ++i)
  {
    motor_adc_zero_stats_sorted[i] = samples[i];
    raw_sum += samples[i];
    if (samples[i] < raw_min) raw_min = samples[i];
    if (samples[i] > raw_max) raw_max = samples[i];
  }
  motor_adc_zero_stats_sort(motor_adc_zero_stats_sorted, count);
  median = motor_adc_zero_stats_median_sorted(motor_adc_zero_stats_sorted, count);

  for (i = 0U; i < count; ++i)
  {
    motor_adc_zero_stats_deviation[i] =
        (motor_adc_zero_stats_sorted[i] >= median) ?
        (uint16_t)(motor_adc_zero_stats_sorted[i] - median) :
        (uint16_t)(median - motor_adc_zero_stats_sorted[i]);
  }
  motor_adc_zero_stats_sort(motor_adc_zero_stats_deviation, count);
  mad = motor_adc_zero_stats_median_sorted(motor_adc_zero_stats_deviation, count);

  /* ceil(4.5 * mad) = (9 * mad + 1) / 2 for integer mad. */
  outlier_limit = (uint16_t)(((uint32_t)MOTOR_ADC_ZERO_STATS_MAD_SCALE_NUM * mad + 1U) /
                             MOTOR_ADC_ZERO_STATS_MAD_SCALE_DEN);
  if (outlier_limit < MOTOR_ADC_ZERO_STATS_OUTLIER_FLOOR_RAW)
  {
    outlier_limit = MOTOR_ADC_ZERO_STATS_OUTLIER_FLOOR_RAW;
  }

  filtered_sum = 0U;
  filtered_min = 0xFFFFU;
  filtered_max = 0U;
  outlier_count = 0U;
  valid_count = 0U;
  for (i = 0U; i < count; ++i)
  {
    uint16_t deviation;

    deviation = (samples[i] >= median) ?
                (uint16_t)(samples[i] - median) :
                (uint16_t)(median - samples[i]);
    if (deviation > outlier_limit)
    {
      outlier_count++;
      continue;
    }

    if (samples[i] < filtered_min) filtered_min = samples[i];
    if (samples[i] > filtered_max) filtered_max = samples[i];
    filtered_sum += samples[i];
    valid_count++;
  }

  stats->raw_mean = (uint16_t)(raw_sum / count);
  stats->raw_min = raw_min;
  stats->raw_max = raw_max;
  stats->raw_peak_to_peak = (uint16_t)(raw_max - raw_min);
  stats->median = median;
  stats->mad = mad;
  stats->outlier_limit = outlier_limit;
  stats->outlier_count = outlier_count;
  stats->valid_count = valid_count;

  if (valid_count == 0U)
  {
    /* 仅用于避免除零；调用者会因 valid_count=0 判为 NOISY。 */
    stats->filtered_mean = median;
    stats->filtered_min = median;
    stats->filtered_max = median;
    stats->filtered_peak_to_peak = 0U;
  }
  else
  {
    stats->filtered_mean = (uint16_t)(filtered_sum / valid_count);
    stats->filtered_min = filtered_min;
    stats->filtered_max = filtered_max;
    stats->filtered_peak_to_peak =
        (uint16_t)(filtered_max - filtered_min);
  }

  return 1U;
}
