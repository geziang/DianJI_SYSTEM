/*
 * ADC 零偏稳健统计 PC 单元测试。
 *
 * 编译（在 STM32F103C8T6_BASE 目录下）：
 *   gcc -std=c99 tests/pc/test_motor_adc_zero_stats.c \
 *       Core/Src/motor_adc_zero_stats.c -I Core/Inc \
 *       -o tests/pc/test_motor_adc_zero_stats
 */
#include <stdio.h>

#include "motor_adc_zero_stats.h"

static int failures;

static void check(int condition, const char *name)
{
  if (condition)
  {
    printf("[PASS] %s\n", name);
  }
  else
  {
    printf("[FAIL] %s\n", name);
    failures++;
  }
}

static void fill_nominal(uint16_t *samples, uint16_t base)
{
  static const int16_t offsets[5] = {-2, -1, 0, 1, 2};
  uint16_t i;

  for (i = 0U; i < 64U; ++i)
  {
    samples[i] = (uint16_t)((int32_t)base + offsets[i % 5U]);
  }
}

int main(void)
{
  uint16_t samples[64];
  uint16_t i;
  motor_adc_zero_stats_t stats;

  fill_nominal(samples, 1000U);
  check(motor_adc_zero_stats_compute(samples, 64U, &stats) != 0U,
        "normal.compute");
  check((stats.outlier_count == 0U) && (stats.valid_count == 64U),
        "normal.no_outliers");
  check(stats.filtered_peak_to_peak <= 32U,
        "normal.filtered_p2p_pass");

  fill_nominal(samples, 1500U);
  samples[29] = 1440U;
  check(motor_adc_zero_stats_compute(samples, 64U, &stats) != 0U,
        "low_spike.compute");
  check((stats.raw_peak_to_peak > 32U) && (stats.outlier_count == 1U),
        "low_spike.detected");
  check((stats.valid_count == 63U) && (stats.filtered_peak_to_peak <= 32U),
        "low_spike.filtered_pass");

  fill_nominal(samples, 2000U);
  samples[17] = 2060U;
  check(motor_adc_zero_stats_compute(samples, 64U, &stats) != 0U,
        "high_spike.compute");
  check((stats.raw_peak_to_peak > 32U) && (stats.outlier_count == 1U),
        "high_spike.detected");
  check(stats.filtered_peak_to_peak <= 32U,
        "high_spike.filtered_pass");

  fill_nominal(samples, 2500U);
  for (i = 0U; i < 9U; ++i)
  {
    samples[i] = (uint16_t)(2400U - i * 10U);
  }
  check(motor_adc_zero_stats_compute(samples, 64U, &stats) != 0U,
        "many_spikes.compute");
  check((stats.outlier_count > 8U) && (stats.valid_count < 56U),
        "many_spikes_limit");

  fill_nominal(samples, 3000U);
  for (i = 0U; i < 64U; ++i)
  {
    samples[i] = (uint16_t)(2900U + i * 2U);
  }
  check(motor_adc_zero_stats_compute(samples, 64U, &stats) != 0U,
        "wide_body.compute");
  check(stats.outlier_count == 0U,
        "wide_body_not_spike_filtered");
  check(stats.filtered_peak_to_peak > 32U,
        "wide_body_noise_remains");

  check(motor_adc_zero_stats_compute(samples, 0U, &stats) == 0U,
        "invalid_count_rejected");

  if (failures == 0)
  {
    printf("ALL ADC ZERO STATS TESTS PASSED\n");
    return 0;
  }
  printf("%d ADC ZERO STATS TEST(S) FAILED\n", failures);
  return 1;
}
