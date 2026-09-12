/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_adc.c
  * @brief   Raw ADC readout for F103 phase-current channels.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor_adc.h"

#include "board_config.h"
#include "motor_drv.h"
#include "motor_pwm.h"
#include "motor_adc_zero_stats.h"
#include <stddef.h>

#define MOTOR_ADC_DMA_TIMEOUT_MS   (5U)
#define MOTOR_ADC_DMA_SAMPLE_COUNT (2U)
/* 硬件零位噪声大（raw_p2p 达 98 code ≈158mA 单点）：采样数提到 128 增强统计稳健性；
 * 噪声阈值 32→64（1code≈1.61mA）以匹配现实，避免 S0 被噪声误判卡死；保留多轮重采兜底。 */
#define MOTOR_ADC_ZERO_SAMPLE_COUNT (128U)
#define MOTOR_ADC_RAIL_MARGIN_RAW   (16U)
#define MOTOR_ADC_MAX_ZERO_NOISE_RAW (64U)
#define MOTOR_ADC_ZERO_MAX_OUTLIERS (8U)
#define MOTOR_ADC_ZERO_MAX_RETRIES  (4U)/* 零位采集不合格重采轮数，兜底取最干净一轮 */
#define MOTOR_ADC_SAMPLE_MAX_AGE_MS  (2U)

static motor_adc_current_diagnostic_t motor_adc_diagnostic;
static volatile uint16_t motor_adc_dma_raw[MOTOR_ADC_DMA_SAMPLE_COUNT];
static volatile uint8_t motor_adc_dma_active;
static volatile uint8_t motor_adc_dma_complete;
static volatile uint8_t motor_adc_dma_error;
static volatile uint8_t motor_adc_synchronised;
static volatile motor_current_sample_t motor_adc_latest_sample;
static motor_adc_sample_callback_t motor_adc_sample_callback;
static motor_adc_zero_sample_t motor_adc_zero_samples[MOTOR_ADC_ZERO_SAMPLE_COUNT];
static uint16_t motor_adc_zero_u_values[MOTOR_ADC_ZERO_SAMPLE_COUNT];
static uint16_t motor_adc_zero_v_values[MOTOR_ADC_ZERO_SAMPLE_COUNT];

/*
 * ADC 采样层。
 *
 * F103 当前只有 ADC1 扫描两个 rank：
 * - Rank 1: ADC1_IN8 / PB0 / M0_OUT1_CS
 * - Rank 2: ADC1_IN9 / PB1 / M0_OUT2_CS
 *
 * 文档中明确 VBUS/POWER_ADC 未确认接入，因此这里不读取母线电压。
 */

static motor_adc_status_t motor_adc_start_dma(ADC_HandleTypeDef *adc)
{
  if (adc == NULL)
  {
    return MOTOR_ADC_STATUS_INVALID_ARG;
  }

  motor_adc_dma_complete = 0U;
  motor_adc_dma_error = 0U;
  if (HAL_ADC_Start_DMA(adc, (uint32_t *)motor_adc_dma_raw,
                        MOTOR_ADC_DMA_SAMPLE_COUNT) != HAL_OK)
  {
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }
  /* DMA 长度为 2，HAL 默认同时使能半传输(HT)与传输完成(TC)中断，会每周期多
   * 进一次空的 HT 中断（中断频率翻倍）。快路径只在 TC 取数，关闭 HT。 */
  __HAL_DMA_DISABLE_IT(adc->DMA_Handle, DMA_IT_HT);

  motor_adc_dma_active = 1U;
  return MOTOR_ADC_STATUS_OK;
}

static void motor_adc_stop_dma(ADC_HandleTypeDef *adc)
{
  if ((adc != NULL) && (motor_adc_dma_active != 0U))
  {
    (void)HAL_ADC_Stop_DMA(adc);
  }

  motor_adc_dma_active = 0U;
}

static motor_adc_status_t motor_adc_configure_trigger(uint32_t trigger)
{
  ADC_HandleTypeDef *adc;

  adc = board_config_get_motor_adc();
  if (adc == NULL)
  {
    return MOTOR_ADC_STATUS_INVALID_ARG;
  }

  motor_adc_stop_dma(adc);
  (void)HAL_TIM_Base_Stop(board_config_get_current_sample_timer());
  adc->Init.ExternalTrigConv = trigger;
  if (HAL_ADC_Init(adc) != HAL_OK)
  {
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }
  return MOTOR_ADC_STATUS_OK;
}

static motor_adc_status_t motor_adc_wait_for_dma(void)
{
  uint32_t start_tick;

  start_tick = HAL_GetTick();
  while (motor_adc_dma_complete == 0U)
  {
    if (motor_adc_dma_error != 0U)
    {
      return MOTOR_ADC_STATUS_HAL_ERROR;
    }

    if ((HAL_GetTick() - start_tick) >= MOTOR_ADC_DMA_TIMEOUT_MS)
    {
      return MOTOR_ADC_STATUS_HAL_ERROR;
    }
  }

  return MOTOR_ADC_STATUS_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == board_config_get_motor_adc())
  {
    motor_adc_dma_complete = 1U;
    if (motor_adc_synchronised != 0U)
    {
      motor_adc_latest_sample.phase_u_raw = motor_adc_dma_raw[0];
      motor_adc_latest_sample.phase_v_raw = motor_adc_dma_raw[1];
      motor_adc_latest_sample.pwm_cycle++;
      motor_adc_latest_sample.timestamp_ms = HAL_GetTick();
      motor_adc_latest_sample.saturated =
          ((motor_adc_dma_raw[0] <= MOTOR_ADC_RAIL_MARGIN_RAW) ||
           (motor_adc_dma_raw[1] <= MOTOR_ADC_RAIL_MARGIN_RAW) ||
           (motor_adc_dma_raw[0] >= (BOARD_CONFIG_ADC_FULL_SCALE_RAW - MOTOR_ADC_RAIL_MARGIN_RAW)) ||
           (motor_adc_dma_raw[1] >= (BOARD_CONFIG_ADC_FULL_SCALE_RAW - MOTOR_ADC_RAIL_MARGIN_RAW))) ? 1U : 0U;
      motor_adc_latest_sample.timeout = 0U;
      motor_adc_latest_sample.valid = (motor_adc_latest_sample.saturated == 0U) ? 1U : 0U;
      if (motor_adc_sample_callback != NULL)
      {
        motor_adc_sample_callback((const motor_current_sample_t *)&motor_adc_latest_sample);
      }
    }
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == board_config_get_motor_adc())
  {
    motor_adc_dma_error = 1U;
    motor_adc_latest_sample.valid = 0U;
    motor_adc_latest_sample.timeout = 1U;
  }
}

static void motor_adc_reset_diagnostic(void)
{
  motor_adc_diagnostic.zero.phase_u_zero_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_zero_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_min_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_max_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_min_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_max_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_peak_to_peak_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_peak_to_peak_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_raw_mean_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_raw_mean_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_median_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_median_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_mad_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_mad_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_outlier_limit_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_outlier_limit_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_outlier_count = 0U;
  motor_adc_diagnostic.zero.phase_v_outlier_count = 0U;
  motor_adc_diagnostic.zero.phase_u_valid_count = 0U;
  motor_adc_diagnostic.zero.phase_v_valid_count = 0U;
  motor_adc_diagnostic.zero.phase_u_filtered_zero_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_filtered_zero_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_filtered_min_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_filtered_max_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_filtered_min_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_filtered_max_raw = 0U;
  motor_adc_diagnostic.zero.phase_u_filtered_peak_to_peak_raw = 0U;
  motor_adc_diagnostic.zero.phase_v_filtered_peak_to_peak_raw = 0U;
  motor_adc_diagnostic.zero.sample_count = 0U;
  motor_adc_diagnostic.zero.adc_status = MOTOR_ADC_STATUS_OK;
  motor_adc_diagnostic.zero.quality = MOTOR_ADC_ZERO_QUALITY_NOT_RUN;
  motor_adc_diagnostic.state = MOTOR_ADC_CURRENT_STATE_UNINITIALIZED;
  motor_adc_diagnostic.adc_hardware_calibrated = 0U;
  motor_adc_diagnostic.scale_verified = 0U;
  motor_adc_diagnostic.direction_verified = 0U;
  motor_adc_diagnostic.sample_timing_verified = 0U;
  motor_adc_diagnostic.closed_loop_allowed = 0U;
  motor_adc_diagnostic.nominal_adc_reference_v = BOARD_CONFIG_ADC_REFERENCE_V_NOMINAL;
  motor_adc_diagnostic.nominal_shunt_ohms = BOARD_CONFIG_PHASE_CURRENT_SHUNT_OHMS;
  motor_adc_diagnostic.nominal_amplifier_gain = BOARD_CONFIG_PHASE_CURRENT_AMPLIFIER_GAIN;
  motor_adc_diagnostic.nominal_volts_per_amp = BOARD_CONFIG_PHASE_CURRENT_VOLTS_PER_AMP;
  motor_adc_synchronised = 0U;
  motor_adc_latest_sample.valid = 0U;
  motor_adc_latest_sample.saturated = 0U;
  motor_adc_latest_sample.timeout = 1U;
  motor_adc_latest_sample.pwm_cycle = 0U;
  motor_adc_latest_sample.timestamp_ms = 0U;
}

static void motor_adc_update_zero_quality(void)
{
  motor_adc_zero_calibration_t *zero;

  zero = &motor_adc_diagnostic.zero;
  if ((zero->phase_u_min_raw <= MOTOR_ADC_RAIL_MARGIN_RAW) ||
      (zero->phase_v_min_raw <= MOTOR_ADC_RAIL_MARGIN_RAW) ||
      (zero->phase_u_max_raw >= (BOARD_CONFIG_ADC_FULL_SCALE_RAW - MOTOR_ADC_RAIL_MARGIN_RAW)) ||
      (zero->phase_v_max_raw >= (BOARD_CONFIG_ADC_FULL_SCALE_RAW - MOTOR_ADC_RAIL_MARGIN_RAW)))
  {
    zero->quality = MOTOR_ADC_ZERO_QUALITY_NEAR_RAIL;
  }
  else if ((zero->phase_u_min_raw == zero->phase_u_max_raw) ||
           (zero->phase_v_min_raw == zero->phase_v_max_raw))
  {
    zero->quality = MOTOR_ADC_ZERO_QUALITY_FLATLINE;
  }
  else if ((zero->phase_u_valid_count < (MOTOR_ADC_ZERO_SAMPLE_COUNT - MOTOR_ADC_ZERO_MAX_OUTLIERS)) ||
           (zero->phase_v_valid_count < (MOTOR_ADC_ZERO_SAMPLE_COUNT - MOTOR_ADC_ZERO_MAX_OUTLIERS)) ||
           (zero->phase_u_outlier_count > MOTOR_ADC_ZERO_MAX_OUTLIERS) ||
           (zero->phase_v_outlier_count > MOTOR_ADC_ZERO_MAX_OUTLIERS) ||
           (zero->phase_u_filtered_peak_to_peak_raw > MOTOR_ADC_MAX_ZERO_NOISE_RAW) ||
           (zero->phase_v_filtered_peak_to_peak_raw > MOTOR_ADC_MAX_ZERO_NOISE_RAW))
  {
    zero->quality = MOTOR_ADC_ZERO_QUALITY_NOISY;
  }
  else
  {
    zero->quality = MOTOR_ADC_ZERO_QUALITY_OK;
  }

  motor_adc_diagnostic.state =
      ((zero->quality == MOTOR_ADC_ZERO_QUALITY_NEAR_RAIL) ||
       (zero->quality == MOTOR_ADC_ZERO_QUALITY_NOISY)) ?
      MOTOR_ADC_CURRENT_STATE_FAULT : MOTOR_ADC_CURRENT_STATE_CALIBRATION_REQUIRED;
  /* 电流增益为板级设计常量（INA240A2=50V/V、shunt=0.01Ω，已对原理图/datasheet 确认），
   * 零偏质量 OK 即 scale 确定；闭环还需电流方向(S1a)验证，故 closed=scale&&direction。
   * sample_timing(S4) 不阻塞首个电流环，避免形成循环依赖。 */
  motor_adc_diagnostic.scale_verified =
      (zero->quality == MOTOR_ADC_ZERO_QUALITY_OK) ? 1U : 0U;
  motor_adc_diagnostic.closed_loop_allowed =
      (motor_adc_diagnostic.scale_verified != 0U) &&
      (motor_adc_diagnostic.direction_verified != 0U);
}

/*
 * 静态采样使用前执行一次 ADC 自校准。调用者必须已确保 PWM 停止、驱动失能。
 * 此操作不标定板级电流零偏，后者由 motor_adc_calibrate_zero_current() 完成。
 */
motor_adc_status_t motor_adc_init_static(void)
{
  ADC_HandleTypeDef *adc;

  motor_adc_reset_diagnostic();
  adc = board_config_get_motor_adc();
  if (adc == NULL)
  {
    motor_adc_diagnostic.zero.adc_status = MOTOR_ADC_STATUS_INVALID_ARG;
    motor_adc_diagnostic.zero.quality = MOTOR_ADC_ZERO_QUALITY_ADC_ERROR;
    motor_adc_diagnostic.state = MOTOR_ADC_CURRENT_STATE_FAULT;
    return MOTOR_ADC_STATUS_INVALID_ARG;
  }

  if (motor_adc_configure_trigger(ADC_SOFTWARE_START) != MOTOR_ADC_STATUS_OK)
  {
    motor_adc_diagnostic.zero.adc_status = MOTOR_ADC_STATUS_HAL_ERROR;
    motor_adc_diagnostic.zero.quality = MOTOR_ADC_ZERO_QUALITY_ADC_ERROR;
    motor_adc_diagnostic.state = MOTOR_ADC_CURRENT_STATE_FAULT;
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }
  if (HAL_ADCEx_Calibration_Start(adc) != HAL_OK)
  {
    motor_adc_diagnostic.zero.adc_status = MOTOR_ADC_STATUS_CALIBRATION_ERROR;
    motor_adc_diagnostic.zero.quality = MOTOR_ADC_ZERO_QUALITY_ADC_ERROR;
    motor_adc_diagnostic.state = MOTOR_ADC_CURRENT_STATE_FAULT;
    return MOTOR_ADC_STATUS_CALIBRATION_ERROR;
  }

  motor_adc_diagnostic.adc_hardware_calibrated = 1U;
  return MOTOR_ADC_STATUS_OK;
}

/*
 * 读取两相电流原始值。
 * 每次软件触发扫描由 DMA 搬运两个 rank，DMA 完成后再返回结果。
 */
motor_adc_status_t motor_adc_read_raw(motor_adc_raw_sample_t *sample)
{
  ADC_HandleTypeDef *adc;

  if (sample == NULL)
  {
    return MOTOR_ADC_STATUS_INVALID_ARG;
  }

  adc = board_config_get_motor_adc();
  if (adc == NULL)
  {
    return MOTOR_ADC_STATUS_INVALID_ARG;
  }

  if (motor_adc_synchronised != 0U)
  {
    return MOTOR_ADC_STATUS_SYNC_NOT_READY;
  }

  sample->phase_u_raw = 0U;
  sample->phase_v_raw = 0U;
  sample->vbus_raw = 0U;
  /* 当前硬件基线未确认 VBUS ADC，诊断层会打印 vbus=NA。 */
  sample->vbus_available = 0U;

  if (motor_adc_dma_active == 0U)
  {
    if (motor_adc_start_dma(adc) != MOTOR_ADC_STATUS_OK)
    {
      return MOTOR_ADC_STATUS_HAL_ERROR;
    }
  }

  if (motor_adc_wait_for_dma() != MOTOR_ADC_STATUS_OK)
  {
    motor_adc_stop_dma(adc);
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }

  sample->phase_u_raw = motor_adc_dma_raw[0];
  sample->phase_v_raw = motor_adc_dma_raw[1];
  motor_adc_stop_dma(adc);
  return MOTOR_ADC_STATUS_OK;
}

motor_adc_status_t motor_adc_start_synchronized(void)
{
  motor_adc_status_t status;

  if (BOARD_CONFIG_PWM_ADC_TRIGGER_CONFIGURED == 0U)
  {
    return MOTOR_ADC_STATUS_SYNC_NOT_READY;
  }
  if (motor_adc_diagnostic.adc_hardware_calibrated == 0U)
  {
    return MOTOR_ADC_STATUS_CALIBRATION_ERROR;
  }

  /* TIM3 trigger is a DMA/firmware integration prototype, not yet a verified low-side window. */
  status = motor_adc_configure_trigger(ADC_EXTERNALTRIGCONV_T3_TRGO);
  if (status != MOTOR_ADC_STATUS_OK)
  {
    return status;
  }

  motor_adc_synchronised = 1U;
  motor_adc_latest_sample.valid = 0U;
  motor_adc_latest_sample.timeout = 1U;
  status = motor_adc_start_dma(board_config_get_motor_adc());
  if (status != MOTOR_ADC_STATUS_OK)
  {
    motor_adc_synchronised = 0U;
  }
  else if (HAL_TIM_Base_Start(board_config_get_current_sample_timer()) != HAL_OK)
  {
    motor_adc_stop_dma(board_config_get_motor_adc());
    motor_adc_synchronised = 0U;
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }
  return status;
}

void motor_adc_stop_synchronized(void)
{
  motor_adc_stop_dma(board_config_get_motor_adc());
  (void)HAL_TIM_Base_Stop(board_config_get_current_sample_timer());
  motor_adc_synchronised = 0U;
  motor_adc_latest_sample.valid = 0U;
  motor_adc_latest_sample.timeout = 1U;
}

const motor_current_sample_t *motor_adc_get_latest_sample(void)
{
  if ((motor_adc_synchronised != 0U) &&
      ((uint32_t)(HAL_GetTick() - motor_adc_latest_sample.timestamp_ms) > MOTOR_ADC_SAMPLE_MAX_AGE_MS))
  {
    motor_adc_latest_sample.timeout = 1U;
    motor_adc_latest_sample.valid = 0U;
  }
  return (const motor_current_sample_t *)&motor_adc_latest_sample;
}

uint8_t motor_adc_sample_is_valid(void)
{
  const motor_current_sample_t *sample;

  sample = motor_adc_get_latest_sample();
  return (uint8_t)((sample->valid != 0U) &&
                   (sample->saturated == 0U) &&
                   (sample->timeout == 0U));
}

void motor_adc_set_runtime_zero(uint16_t phase_u_zero_raw, uint16_t phase_v_zero_raw)
{
  motor_adc_diagnostic.zero.phase_u_zero_raw = phase_u_zero_raw;
  motor_adc_diagnostic.zero.phase_v_zero_raw = phase_v_zero_raw;
}

void motor_adc_set_sample_callback(motor_adc_sample_callback_t callback)
{
  motor_adc_sample_callback = callback;
}

void motor_adc_set_direction_verified(uint8_t verified)
{
  motor_adc_diagnostic.direction_verified = (verified != 0U) ? 1U : 0U;
  /* 闭环门禁 = 标准增益 scale 已确定 且 方向已验证。 */
  motor_adc_diagnostic.closed_loop_allowed =
      (motor_adc_diagnostic.scale_verified != 0U) &&
      (motor_adc_diagnostic.direction_verified != 0U);
}

void motor_adc_set_sample_timing_verified(uint8_t verified)
{
  motor_adc_diagnostic.sample_timing_verified = (verified != 0U) ? 1U : 0U;
}

motor_adc_status_t motor_adc_calibrate_zero_current(void)
{
  uint32_t phase_u_sum;
  uint32_t phase_v_sum;
  uint16_t index;
  motor_adc_raw_sample_t sample;
  motor_adc_status_t status;
  motor_adc_zero_calibration_t *zero;
  motor_adc_zero_stats_t u_stats;
  motor_adc_zero_stats_t v_stats;

  zero = &motor_adc_diagnostic.zero;
  zero->sample_count = 0U;
  zero->quality = MOTOR_ADC_ZERO_QUALITY_NOT_RUN;
  zero->adc_status = MOTOR_ADC_STATUS_OK;

  if ((motor_drv_is_enabled() != 0U) || (motor_pwm_is_output_started() != 0U))
  {
    zero->adc_status = MOTOR_ADC_STATUS_UNSAFE_STATE;
    zero->quality = MOTOR_ADC_ZERO_QUALITY_ADC_ERROR;
    motor_adc_diagnostic.state = MOTOR_ADC_CURRENT_STATE_FAULT;
    return zero->adc_status;
  }

  if (motor_adc_diagnostic.adc_hardware_calibrated == 0U)
  {
    zero->adc_status = MOTOR_ADC_STATUS_CALIBRATION_ERROR;
    zero->quality = MOTOR_ADC_ZERO_QUALITY_ADC_ERROR;
    motor_adc_diagnostic.state = MOTOR_ADC_CURRENT_STATE_FAULT;
    return zero->adc_status;
  }

  /* 多轮验收：单轮采样→稳健统计→过滤后 p2p 达标即用；否则丢弃本轮并重采，
   * 最多 MOTOR_ADC_ZERO_MAX_RETRIES 轮，全不达标则兜底取最干净一轮。 */
  {
    motor_adc_zero_stats_t best_u;
    motor_adc_zero_stats_t best_v;
    uint32_t best_score = 0xFFFFFFFFu;
    uint8_t accepted = 0U;
    uint8_t r;

    for (r = 0U; r < MOTOR_ADC_ZERO_MAX_RETRIES; r++)
    {
      phase_u_sum = 0U;
      phase_v_sum = 0U;
      zero->sample_count = 0U;

      for (index = 0U; index < MOTOR_ADC_ZERO_SAMPLE_COUNT; index++)
      {
        status = motor_adc_read_raw(&sample);
        if (status != MOTOR_ADC_STATUS_OK)
        {
          zero->adc_status = status;
          zero->quality = MOTOR_ADC_ZERO_QUALITY_ADC_ERROR;
          motor_adc_diagnostic.state = MOTOR_ADC_CURRENT_STATE_FAULT;
          return status;
        }
        motor_adc_zero_samples[index].phase_u_raw = sample.phase_u_raw;
        motor_adc_zero_samples[index].phase_v_raw = sample.phase_v_raw;
        motor_adc_zero_u_values[index] = sample.phase_u_raw;
        motor_adc_zero_v_values[index] = sample.phase_v_raw;
        phase_u_sum += sample.phase_u_raw;
        phase_v_sum += sample.phase_v_raw;
        zero->sample_count++;
      }

      if ((motor_adc_zero_stats_compute(motor_adc_zero_u_values,
                                        MOTOR_ADC_ZERO_SAMPLE_COUNT, &u_stats) == 0U) ||
          (motor_adc_zero_stats_compute(motor_adc_zero_v_values,
                                        MOTOR_ADC_ZERO_SAMPLE_COUNT, &v_stats) == 0U))
      {
        /* 不应发生：统计模块输入由本函数刚刚填满。保守地锁存 ADC 错误。 */
        zero->adc_status = MOTOR_ADC_STATUS_HAL_ERROR;
        zero->quality = MOTOR_ADC_ZERO_QUALITY_ADC_ERROR;
        motor_adc_diagnostic.state = MOTOR_ADC_CURRENT_STATE_FAULT;
        return zero->adc_status;
      }

      /* 达标（过滤后 p2p ≤ 噪声阈值）：采用本轮，结束重采。 */
      if ((u_stats.filtered_peak_to_peak <= MOTOR_ADC_MAX_ZERO_NOISE_RAW) &&
          (v_stats.filtered_peak_to_peak <= MOTOR_ADC_MAX_ZERO_NOISE_RAW))
      {
        accepted = 1U;
        break;
      }
      /* 未达标：记录更干净的一轮（以两相最高 filtering p2p 为评分）。 */
      {
        uint32_t score = (u_stats.filtered_peak_to_peak > v_stats.filtered_peak_to_peak)
                         ? (uint32_t)u_stats.filtered_peak_to_peak
                         : (uint32_t)v_stats.filtered_peak_to_peak;
        if (score < best_score)
        {
          best_score = score;
          best_u = u_stats;
          best_v = v_stats;
        }
      }
    }

    /* 全部未达标：兜底采用最干净一轮的统计结果。 */
    if (accepted == 0U)
    {
      u_stats = best_u;
      v_stats = best_v;
    }

    /* 用选定轮的统计结果落零位（原始 min/max 也用该轮，保证 NEAR_RAIL/诊断一致）。 */
    zero->phase_u_raw_mean_raw = u_stats.raw_mean;
    zero->phase_v_raw_mean_raw = v_stats.raw_mean;
    zero->phase_u_peak_to_peak_raw = u_stats.raw_peak_to_peak;
    zero->phase_v_peak_to_peak_raw = v_stats.raw_peak_to_peak;
    zero->phase_u_min_raw = u_stats.raw_min;
    zero->phase_u_max_raw = u_stats.raw_max;
    zero->phase_v_min_raw = v_stats.raw_min;
    zero->phase_v_max_raw = v_stats.raw_max;
    zero->phase_u_median_raw = u_stats.median;
    zero->phase_v_median_raw = v_stats.median;
    zero->phase_u_mad_raw = u_stats.mad;
    zero->phase_v_mad_raw = v_stats.mad;
    zero->phase_u_outlier_limit_raw = u_stats.outlier_limit;
    zero->phase_v_outlier_limit_raw = v_stats.outlier_limit;
    zero->phase_u_outlier_count = u_stats.outlier_count;
    zero->phase_v_outlier_count = v_stats.outlier_count;
    zero->phase_u_valid_count = u_stats.valid_count;
    zero->phase_v_valid_count = v_stats.valid_count;
    zero->phase_u_filtered_zero_raw = u_stats.filtered_mean;
    zero->phase_v_filtered_zero_raw = v_stats.filtered_mean;
    zero->phase_u_filtered_min_raw = u_stats.filtered_min;
    zero->phase_u_filtered_max_raw = u_stats.filtered_max;
    zero->phase_v_filtered_min_raw = v_stats.filtered_min;
    zero->phase_v_filtered_max_raw = v_stats.filtered_max;
    zero->phase_u_filtered_peak_to_peak_raw = u_stats.filtered_peak_to_peak;
    zero->phase_v_filtered_peak_to_peak_raw = v_stats.filtered_peak_to_peak;
    zero->phase_u_zero_raw = u_stats.filtered_mean;
    zero->phase_v_zero_raw = v_stats.filtered_mean;
  }

  motor_adc_update_zero_quality();
  return MOTOR_ADC_STATUS_OK;
}

void motor_adc_get_current_diagnostic(motor_adc_current_diagnostic_t *diagnostic)
{
  if (diagnostic != NULL)
  {
    *diagnostic = motor_adc_diagnostic;
  }
}

const motor_adc_zero_sample_t *motor_adc_get_zero_samples(void)
{
  return motor_adc_zero_samples;
}

uint16_t motor_adc_get_zero_sample_count(void)
{
  return motor_adc_diagnostic.zero.sample_count;
}

uint8_t motor_adc_is_closed_loop_allowed(void)
{
  return motor_adc_diagnostic.closed_loop_allowed;
}

const char *motor_adc_zero_quality_text(motor_adc_zero_quality_t quality)
{
  switch (quality)
  {
    case MOTOR_ADC_ZERO_QUALITY_NOT_RUN: return "NOT_RUN";
    case MOTOR_ADC_ZERO_QUALITY_OK: return "OK";
    case MOTOR_ADC_ZERO_QUALITY_ADC_ERROR: return "ADC_ERROR";
    case MOTOR_ADC_ZERO_QUALITY_NEAR_RAIL: return "NEAR_RAIL";
    case MOTOR_ADC_ZERO_QUALITY_FLATLINE: return "FLATLINE";
    case MOTOR_ADC_ZERO_QUALITY_NOISY: return "NOISY";
    default: return "UNKNOWN";
  }
}

const char *motor_adc_current_state_text(motor_adc_current_state_t state)
{
  switch (state)
  {
    case MOTOR_ADC_CURRENT_STATE_UNINITIALIZED: return "UNINITIALIZED";
    case MOTOR_ADC_CURRENT_STATE_CALIBRATION_REQUIRED: return "CALIBRATION_REQUIRED";
    case MOTOR_ADC_CURRENT_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}
