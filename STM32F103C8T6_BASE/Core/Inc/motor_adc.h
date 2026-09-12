/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_adc.h
  * @brief   Raw ADC readout for F103 phase-current channels.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  /* 采样成功。 */
  MOTOR_ADC_STATUS_OK = 0,

  /* 调用参数为空。 */
  MOTOR_ADC_STATUS_INVALID_ARG,

  /* HAL ADC 启动或等待转换失败。 */
  MOTOR_ADC_STATUS_HAL_ERROR,

  /* ADC 硬件自校准失败。 */
  MOTOR_ADC_STATUS_CALIBRATION_ERROR,

  /* 零偏采样请求时 PWM 或功率级未处于安全失能状态。 */
  MOTOR_ADC_STATUS_UNSAFE_STATE,

  /* PWM 触发 ADC/DMA 链路尚未启动或无法配置。 */
  MOTOR_ADC_STATUS_SYNC_NOT_READY
} motor_adc_status_t;

typedef enum
{
  /* 尚未执行失能态零偏采样。 */
  MOTOR_ADC_ZERO_QUALITY_NOT_RUN = 0,

  /* 零偏和原始样本的静态质量检查通过。 */
  MOTOR_ADC_ZERO_QUALITY_OK,

  /* ADC 读取或硬件校准失败。 */
  MOTOR_ADC_ZERO_QUALITY_ADC_ERROR,

  /* 零偏接近 ADC 上下量程边界。 */
  MOTOR_ADC_ZERO_QUALITY_NEAR_RAIL,

  /* 采样窗口内没有任何码值变化，需检查模拟链路。 */
  MOTOR_ADC_ZERO_QUALITY_FLATLINE,

  /* 剔除孤立尖峰后的有效样本峰峰值超过当前静态检查阈值。 */
  MOTOR_ADC_ZERO_QUALITY_NOISY
} motor_adc_zero_quality_t;

typedef enum
{
  /* 未完成 ADC 自校准或零偏采样。 */
  MOTOR_ADC_CURRENT_STATE_UNINITIALIZED = 0,

  /* 静态零偏正常，但比例、方向和 PWM 同步尚未验证。 */
  MOTOR_ADC_CURRENT_STATE_CALIBRATION_REQUIRED,

  /* ADC 硬件校准、读取或零偏质量检查失败。 */
  MOTOR_ADC_CURRENT_STATE_FAULT
} motor_adc_current_state_t;

typedef struct
{
  /* PB0 / ADC1_IN8，对应文档中的 M0_OUT1_CS。 */
  uint16_t phase_u_raw;

  /* PB1 / ADC1_IN9，对应文档中的 M0_OUT2_CS。 */
  uint16_t phase_v_raw;

  /* 当前 F103 基线没有确认 VBUS 接入 ADC，保留字段但标记不可用。 */
  uint16_t vbus_raw;
  uint8_t vbus_available;
} motor_adc_raw_sample_t;

/* PWM 关联样本。只有 valid=1 且未超时、未饱和时才可供未来电流环使用。 */
typedef struct
{
  uint16_t phase_u_raw;
  uint16_t phase_v_raw;
  uint32_t pwm_cycle;
  uint32_t timestamp_ms;
  uint8_t valid;
  uint8_t saturated;
  uint8_t timeout;
} motor_current_sample_t;

typedef void (*motor_adc_sample_callback_t)(const motor_current_sample_t *sample);

/* 失能态零偏采样结果。所有 raw 数值均为 ADC code，不是安培。 */
typedef struct
{
  uint16_t phase_u_zero_raw;
  uint16_t phase_v_zero_raw;
  uint16_t phase_u_min_raw;
  uint16_t phase_u_max_raw;
  uint16_t phase_v_min_raw;
  uint16_t phase_v_max_raw;
  uint16_t phase_u_peak_to_peak_raw;
  uint16_t phase_v_peak_to_peak_raw;
  /* 全部 64 点的原始均值，仅用于诊断对比。 */
  uint16_t phase_u_raw_mean_raw;
  uint16_t phase_v_raw_mean_raw;
  /* 中位数/MAD 稳健统计，用于识别孤立尖峰。 */
  uint16_t phase_u_median_raw;
  uint16_t phase_v_median_raw;
  uint16_t phase_u_mad_raw;
  uint16_t phase_v_mad_raw;
  uint16_t phase_u_outlier_limit_raw;
  uint16_t phase_v_outlier_limit_raw;
  uint16_t phase_u_outlier_count;
  uint16_t phase_v_outlier_count;
  uint16_t phase_u_valid_count;
  uint16_t phase_v_valid_count;
  /* 剔除孤立尖峰后的有效样本统计；32 code 门限使用 filtered p2p。 */
  uint16_t phase_u_filtered_zero_raw;
  uint16_t phase_v_filtered_zero_raw;
  uint16_t phase_u_filtered_min_raw;
  uint16_t phase_u_filtered_max_raw;
  uint16_t phase_v_filtered_min_raw;
  uint16_t phase_v_filtered_max_raw;
  uint16_t phase_u_filtered_peak_to_peak_raw;
  uint16_t phase_v_filtered_peak_to_peak_raw;
  uint16_t sample_count;
  motor_adc_status_t adc_status;
  motor_adc_zero_quality_t quality;
} motor_adc_zero_calibration_t;

/* 失能态零偏校准期间保存的一帧原始 ADC 样本。 */
typedef struct
{
  uint16_t phase_u_raw;
  uint16_t phase_v_raw;
} motor_adc_zero_sample_t;

/*
 * 当前电流链路的放行状态。
 * nominal_* 是板级标准增益（INA240A2=50V/V、shunt=0.01Ω，已对原理图/datasheet
 * 确认）：零偏质量 OK 即置 scale_verified；direction 由 S1a 开环判定置位；
 * closed_loop_allowed = scale && direction；sample_timing(S4) 不阻塞首个电流环。
 */
typedef struct
{
  motor_adc_zero_calibration_t zero;
  motor_adc_current_state_t state;
  uint8_t adc_hardware_calibrated;
  uint8_t scale_verified;
  uint8_t direction_verified;
  uint8_t sample_timing_verified;
  uint8_t closed_loop_allowed;
  float nominal_adc_reference_v;
  float nominal_shunt_ohms;
  float nominal_amplifier_gain;
  float nominal_volts_per_amp;
} motor_adc_current_diagnostic_t;

/* 在 PWM 停止和 MOTOR_EN 失能后调用，执行 ADC1 硬件自校准并预启动首个 DMA 扫描。 */
motor_adc_status_t motor_adc_init_static(void);

/* 使用 DMA 请求 ADC1 扫描并读取 IN8/IN9 两个 rank。 */
motor_adc_status_t motor_adc_read_raw(motor_adc_raw_sample_t *sample);

/* 切换 ADC1 至 TIM2_CH2 触发、DMA 循环采样模式。必须先完成示波器窗口验证。 */
motor_adc_status_t motor_adc_start_synchronized(void);
void motor_adc_stop_synchronized(void);
const motor_current_sample_t *motor_adc_get_latest_sample(void);
uint8_t motor_adc_sample_is_valid(void);

/* 将已验证的运行零偏写入当前运行时配置；不写入长期参数包。 */
void motor_adc_set_runtime_zero(uint16_t phase_u_zero_raw, uint16_t phase_v_zero_raw);
void motor_adc_set_sample_callback(motor_adc_sample_callback_t callback);

/* S1 raw 层符号/通道判定通过后置位电流方向验证标志。 */
void motor_adc_set_direction_verified(uint8_t verified);

/* S4 电流环阶跃自检通过后置位采样/控制时序验证标志。 */
void motor_adc_set_sample_timing_verified(uint8_t verified);

/* 在驱动失能、PWM 为零且无相电流时执行多次零偏采样和质量判断。 */
motor_adc_status_t motor_adc_calibrate_zero_current(void);

/* 读取最近一次零偏、候选硬件参数和闭环门禁状态。 */
void motor_adc_get_current_diagnostic(motor_adc_current_diagnostic_t *diagnostic);

/* 读取最近一次零偏校准保存的原始样本序列；有效长度由 getter 返回。 */
const motor_adc_zero_sample_t *motor_adc_get_zero_samples(void);
uint16_t motor_adc_get_zero_sample_count(void);

/* 返回 scale_verified && direction_verified：标准增益确定且 S1a 方向验证通过才为 1。 */
uint8_t motor_adc_is_closed_loop_allowed(void);

const char *motor_adc_zero_quality_text(motor_adc_zero_quality_t quality);
const char *motor_adc_current_state_text(motor_adc_current_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ADC_H */
