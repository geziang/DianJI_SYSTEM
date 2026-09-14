/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    baseline_diag.h
  * @brief   Baseline static diagnostic pipeline.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BASELINE_DIAG_H
#define BASELINE_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  /* 只检查 MCU 自身和安全状态，不访问驱动板侧 ADC/编码器。 */
  BASELINE_DIAG_MODE_MCU_ONLY = 0,

  /* 完整静态诊断：检查 ADC 两相采样和 MT6701，但仍不放开功率输出。 */
  BASELINE_DIAG_MODE_FULL_STATIC_DIAG
} baseline_diag_mode_t;

typedef enum
{
  /* 检查项正常。 */
  BASELINE_DIAG_RESULT_PASS = 0,

  /* 有可疑现象，但不一定代表硬件错误，例如 ADC 全 0 或磁铁状态不理想。 */
  BASELINE_DIAG_RESULT_WARN,

  /* 检查失败，例如驱动被使能、PWM 非零、ADC/I2C 读取失败。 */
  BASELINE_DIAG_RESULT_FAIL
} baseline_diag_result_t;

/* 初始化并立即运行一次静态诊断流水线。 */
void baseline_diag_init(void);

/* 周期轮询，默认每 500 ms 输出一次监视日志。 */
void baseline_diag_poll(void);

/* 切换诊断模式；无效模式会被忽略。 */
void baseline_diag_set_mode(baseline_diag_mode_t mode);

/* 查询当前诊断模式和最近一次诊断结果，方便后续状态机复用。 */
baseline_diag_mode_t baseline_diag_get_mode(void);
baseline_diag_result_t baseline_diag_get_last_result(void);

#ifdef __cplusplus
}
#endif

#endif /* BASELINE_DIAG_H */
