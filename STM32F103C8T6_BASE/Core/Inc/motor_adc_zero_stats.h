/*
 * 失能态 ADC 零偏样本的稳健统计。
 *
 * 本模块只处理整数 ADC code，不访问 HAL、DMA、UART 或控制器；调用者负责
 * 保证输入来自安全态零偏窗口。工作缓冲放在实现文件静态区，避免把 64 点
 * 排序数组压入 STM32F103 的启动栈。本模块只允许在主循环启动校准慢路径
 * 串行调用，不可从中断或多个上下文并发调用。
 */
#ifndef MOTOR_ADC_ZERO_STATS_H
#define MOTOR_ADC_ZERO_STATS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MOTOR_ADC_ZERO_STATS_MAX_SAMPLES (128U)

typedef struct
{
  uint16_t raw_mean;
  uint16_t raw_min;
  uint16_t raw_max;
  uint16_t raw_peak_to_peak;
  uint16_t median;
  uint16_t mad;
  uint16_t outlier_limit;
  uint16_t outlier_count;
  uint16_t valid_count;
  uint16_t filtered_mean;
  uint16_t filtered_min;
  uint16_t filtered_max;
  uint16_t filtered_peak_to_peak;
} motor_adc_zero_stats_t;

/*
 * 对一条 ADC 零偏序列计算原始统计和 Hampel/MAD 稳健统计。
 * 返回 1 表示参数有效并完成计算，返回 0 表示输入无效。
 * outlier_limit = max(12 code, ceil(4.5 * MAD))，判断使用严格的 >。
 */
uint8_t motor_adc_zero_stats_compute(const uint16_t *samples,
                                     uint16_t count,
                                     motor_adc_zero_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ADC_ZERO_STATS_H */
