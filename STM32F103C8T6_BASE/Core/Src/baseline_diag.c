/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    baseline_diag.c
  * @brief   Baseline static diagnostic pipeline for the F103 adapter.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "baseline_diag.h"

#include "app_baseline.h"
#include "board_config.h"
#if APP_MODE_RUN
#include "app_run.h"
#else
#include "control_loop.h"
#endif
#include "debug_log.h"
#include "motor_adc.h"
#include "motor_drv.h"
#include "motor_pwm.h"
#include "mt6701.h"

#include <stdarg.h>
#include <stdio.h>

#ifndef BASELINE_DIAG_DEFAULT_MODE
#define BASELINE_DIAG_DEFAULT_MODE  (BASELINE_DIAG_MODE_FULL_STATIC_DIAG)
#endif

/* 监视日志周期。静态诊断阶段日志频率低，避免阻塞式串口输出影响过大。 */
#define BASELINE_DIAG_MONITOR_PERIOD_MS  (500U)

/* 暂停周期诊断打印；控制层的安全监测仍由 control_loop_poll() 执行。 */
#define BASELINE_DIAG_PERIODIC_LOG_ENABLE (0)

/* 单行日志缓冲区大小，覆盖 ADC/编码器快照等最长日志。 */
#define BASELINE_DIAG_LINE_SIZE          (224U)

/*
 * 基线诊断模块。
 *
 * 对应 F103 基线文档中的 BL1/BL2：
 * - BL1：确认上电安全态，MOTOR_EN 不使能、PWM compare 为 0、控制层 safe idle。
 * - BL2：静态读取 ADC1_IN8/IN9、I2C2 编码器，并通过 USART1 输出证据。
 *
 * 本模块本身不启动 PWM 或使能驱动；显式串口命令由应用层请求开环测试。
 */

/* 默认运行完整静态诊断；如只想检查 MCU 自身，可切到 MCU_ONLY。 */
static baseline_diag_mode_t baseline_diag_mode = BASELINE_DIAG_DEFAULT_MODE;

/* 最近一次流水线或监视检查的综合结果。 */
static baseline_diag_result_t baseline_diag_last_result = BASELINE_DIAG_RESULT_PASS;

/* 静默模式（2026-09-13 ROM 瘦身）：上电流水线 PASS 明细不打、只打一行汇总；
 * WARN/FAIL 明细必打（出问题统一打印）；`i` 命令经 verbose API 临时全量输出。 */
static uint8_t baseline_diag_quiet = 1U;

#if BASELINE_DIAG_PERIODIC_LOG_ENABLE
/* 上次输出周期监视日志的 HAL tick。 */
static uint32_t baseline_diag_last_monitor_tick = 0U;
#endif

#if BASELINE_DIAG_PERIODIC_LOG_ENABLE
/* 防止未初始化时 poll 误运行监视逻辑。 */
static uint8_t baseline_diag_initialized = 0U;
#endif

/* 把诊断模式转成日志字符串。 */
static const char *baseline_diag_mode_text(baseline_diag_mode_t mode)
{
  switch (mode)
  {
    case BASELINE_DIAG_MODE_MCU_ONLY:
      return "MCU_ONLY";
    case BASELINE_DIAG_MODE_FULL_STATIC_DIAG:
      return "FULL_STATIC_DIAG";
    default:
      return "UNKNOWN";
  }
}

/* 把 PASS/WARN/FAIL 转成日志字符串。 */
static const char *baseline_diag_result_text(baseline_diag_result_t result)
{
  switch (result)
  {
    case BASELINE_DIAG_RESULT_PASS:
      return "PASS";
    case BASELINE_DIAG_RESULT_WARN:
      return "WARN";
    case BASELINE_DIAG_RESULT_FAIL:
      return "FAIL";
    default:
      return "UNKNOWN";
  }
}

/* 合并多个检查结果：FAIL 优先级最高，WARN 次之，PASS 最低。 */
static baseline_diag_result_t baseline_diag_worse(baseline_diag_result_t left,
                                                  baseline_diag_result_t right)
{
  return (left > right) ? left : right;
}

/* 格式化输出一行诊断日志，底层走 debug_log -> USART1。 */
static void baseline_diag_logf(const char *fmt, ...)
{
  char line[BASELINE_DIAG_LINE_SIZE];
  int written;
  va_list args;

  va_start(args, fmt);
  written = vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  if (written > 0)
  {
    debug_log_write_line(line);
  }
}

/* 输出零偏校准汇总（大信号下零偏噪声可忽略，只打一行汇总，不打逐样本）。 */
static void baseline_diag_log_zero_samples(void)
{
  motor_adc_current_diagnostic_t current;

  motor_adc_get_current_diagnostic(&current);
  baseline_diag_logf("[ADC_ZERO] n=%u zero=%u,%u mad=%u,%u p2p=%u,%u quality=%s",
                     (unsigned int)current.zero.sample_count,
                     (unsigned int)current.zero.phase_u_raw_mean_raw,
                     (unsigned int)current.zero.phase_v_raw_mean_raw,
                     (unsigned int)current.zero.phase_u_mad_raw,
                     (unsigned int)current.zero.phase_v_mad_raw,
                     (unsigned int)current.zero.phase_u_peak_to_peak_raw,
                     (unsigned int)current.zero.phase_v_peak_to_peak_raw,
                     motor_adc_zero_quality_text(current.zero.quality));
}

/* 启动信息检查：确认当前版本、模式、PWM 周期和 VBUS 可用性（恒 PASS，静默态跳过）。 */
static baseline_diag_result_t baseline_diag_check_boot(void)
{
  if (baseline_diag_quiet == 0U)
  {
    baseline_diag_logf("[BOOT] version=%s", APP_BASELINE_VERSION);
    baseline_diag_logf("[BOOT] mode=%s", baseline_diag_mode_text(baseline_diag_mode));
    baseline_diag_logf("[BOOT] pwm_timer=TIM2 pwm_period=%lu",
                       (unsigned long)BOARD_CONFIG_PWM_PERIOD_TICKS);
    baseline_diag_logf("[BOOT] vbus_adc=unavailable");
  }
  return BASELINE_DIAG_RESULT_PASS;
}

/* 驱动使能检查：静态基线要求 MOTOR_EN 处于 disabled。 */
static baseline_diag_result_t baseline_diag_check_drv(void)
{
  baseline_diag_result_t result;
  motor_drv_state_t state;

  state = motor_drv_get_state();
  result = (state == MOTOR_DRV_STATE_DISABLED) ?
           BASELINE_DIAG_RESULT_PASS : BASELINE_DIAG_RESULT_FAIL;

  if ((baseline_diag_quiet == 0U) || (result != BASELINE_DIAG_RESULT_PASS))
  {
    baseline_diag_logf("[DRV] state=%s %s",
                       motor_drv_is_enabled() != 0U ? "enabled" : "disabled",
                       baseline_diag_result_text(result));
  }
  return result;
}

/* PWM 安全态检查：TIM2 CH1/CH2/CH3 的 compare 必须全部为 0。 */
static baseline_diag_result_t baseline_diag_check_pwm(void)
{
  TIM_HandleTypeDef *timer;
  uint32_t ch1;
  uint32_t ch2;
  uint32_t ch3;
  baseline_diag_result_t result;

  timer = board_config_get_motor_pwm_timer();
  ch1 = __HAL_TIM_GET_COMPARE(timer, TIM_CHANNEL_1);
  ch2 = __HAL_TIM_GET_COMPARE(timer, TIM_CHANNEL_2);
  ch3 = __HAL_TIM_GET_COMPARE(timer, TIM_CHANNEL_3);

  result = ((ch1 == 0U) && (ch2 == 0U) && (ch3 == 0U)) ?
           BASELINE_DIAG_RESULT_PASS : BASELINE_DIAG_RESULT_FAIL;

  if ((baseline_diag_quiet == 0U) || (result != BASELINE_DIAG_RESULT_PASS))
  {
    baseline_diag_logf("[PWM] ccr=%lu,%lu,%lu %s",
                       (unsigned long)ch1,
                       (unsigned long)ch2,
                       (unsigned long)ch3,
                       baseline_diag_result_text(result));
  }
  return result;
}

/*
 * ADC 静态采样检查。
 *
 * F103 当前只读取两相电流采样：
 * - phase_u_raw: PB0 / ADC1_IN8 / M0_OUT1_CS
 * - phase_v_raw: PB1 / ADC1_IN9 / M0_OUT2_CS
 * VBUS 未确认接入，因此日志打印 vbus=NA。
 */
static baseline_diag_result_t baseline_diag_check_adc(motor_adc_raw_sample_t *sample)
{
  motor_adc_current_diagnostic_t current;
  baseline_diag_result_t result;

  if (motor_adc_read_raw(sample) != MOTOR_ADC_STATUS_OK)
  {
    baseline_diag_logf("[ADC] read failed FAIL");
    return BASELINE_DIAG_RESULT_FAIL;
  }

  motor_adc_get_current_diagnostic(&current);
  result = BASELINE_DIAG_RESULT_PASS;
  if ((sample->phase_u_raw == 0U) && (sample->phase_v_raw == 0U))
  {
    result = BASELINE_DIAG_RESULT_WARN;
  }

  if ((sample->phase_u_raw >= 4090U) || (sample->phase_v_raw >= 4090U))
  {
    result = BASELINE_DIAG_RESULT_FAIL;
  }
  if ((current.zero.quality == MOTOR_ADC_ZERO_QUALITY_ADC_ERROR) ||
      (current.zero.quality == MOTOR_ADC_ZERO_QUALITY_NEAR_RAIL) ||
      (current.zero.quality == MOTOR_ADC_ZERO_QUALITY_NOISY))
  {
    result = BASELINE_DIAG_RESULT_FAIL;
  }
  else if (current.zero.quality != MOTOR_ADC_ZERO_QUALITY_OK)
  {
    result = baseline_diag_worse(result, BASELINE_DIAG_RESULT_WARN);
  }

  if ((baseline_diag_quiet == 0U) || (result != BASELINE_DIAG_RESULT_PASS))
  {
    baseline_diag_logf("[ADC] u=%u v=%u vbus=NA %s",
                       (unsigned int)sample->phase_u_raw,
                       (unsigned int)sample->phase_v_raw,
                       baseline_diag_result_text(result));
    baseline_diag_logf("[CUR] zero=%u,%u p2p=%u,%u n=%u quality=%s state=%s",
                       (unsigned int)current.zero.phase_u_zero_raw,
                       (unsigned int)current.zero.phase_v_zero_raw,
                       (unsigned int)current.zero.phase_u_peak_to_peak_raw,
                       (unsigned int)current.zero.phase_v_peak_to_peak_raw,
                       (unsigned int)current.zero.sample_count,
                       motor_adc_zero_quality_text(current.zero.quality),
                       motor_adc_current_state_text(current.state));
    baseline_diag_logf("[CUR] nominal=3300mV shunt=10mohm gain=50 V/A=500mV verified=%u,%u,%u closed_loop=%u",
                       (unsigned int)current.scale_verified,
                       (unsigned int)current.direction_verified,
                       (unsigned int)current.sample_timing_verified,
                       (unsigned int)current.closed_loop_allowed);
  }
  return result;
}

/*
 * 编码器检查。
 *
 * 通过 I2C2 访问 MT6701，先确认设备响应，再读取 14 bit 角度。
 * MT6701 的磁场状态只在 SSI 帧中提供；当前 I2C 基线先将角度可读作为 PASS。
 */
static baseline_diag_result_t baseline_diag_check_mt6701(mt6701_snapshot_t *snapshot)
{
  mt6701_status_t status;

  status = mt6701_is_connected();
  if (status != MT6701_STATUS_OK)
  {
    baseline_diag_logf("[ENC] device=MT6701 connected=FAIL status=%s",
                       mt6701_status_text(status));
    return BASELINE_DIAG_RESULT_FAIL;
  }

  status = mt6701_read_snapshot(snapshot);
  if (status != MT6701_STATUS_OK)
  {
    baseline_diag_logf("[ENC] device=MT6701 snapshot=FAIL status=%s",
                       mt6701_status_text(status));
    return BASELINE_DIAG_RESULT_FAIL;
  }

  if (baseline_diag_quiet == 0U)
  {
    baseline_diag_logf("[ENC] device=MT6701 raw=%u angle=%u.%02u field=%s %s",
                       (unsigned int)snapshot->raw_angle,
                       (unsigned int)(snapshot->angle_degrees_x100 / 100U),
                       (unsigned int)(snapshot->angle_degrees_x100 % 100U),
                       mt6701_field_status_text(snapshot->field_status),
                       baseline_diag_result_text(BASELINE_DIAG_RESULT_PASS));
  }
  return BASELINE_DIAG_RESULT_PASS;
}

/* 控制层检查：测试态查 control_loop 安全态；运行态查 app_run 状态
 * （FAULT=FAIL、NEED_CAL=WARN，静默规则同其它检查）。 */
#if APP_MODE_RUN
static baseline_diag_result_t baseline_diag_check_control(void)
{
  app_run_state_t state;
  baseline_diag_result_t result;

  state = app_run_get_state();
  result = (state == APP_RUN_STATE_FAULT) ? BASELINE_DIAG_RESULT_FAIL :
           (state == APP_RUN_STATE_NEED_CAL) ? BASELINE_DIAG_RESULT_WARN :
           BASELINE_DIAG_RESULT_PASS;

  if ((baseline_diag_quiet == 0U) || (result != BASELINE_DIAG_RESULT_PASS))
  {
    baseline_diag_logf("[CTRL] state=RUN_%s %s",
                       app_run_state_text(state),
                       baseline_diag_result_text(result));
  }
  return result;
}
#else
static baseline_diag_result_t baseline_diag_check_control(void)
{
  baseline_diag_result_t result;
  control_loop_state_t state;

  state = control_loop_get_state();
  result = ((state == CONTROL_LOOP_STATE_SAFE_IDLE) ||
            (state == CONTROL_LOOP_STATE_TEST_BOOT) ||
            (state == CONTROL_LOOP_STATE_TEST_READY) ||
            (state == CONTROL_LOOP_STATE_POWER_ARMED)) ?
           BASELINE_DIAG_RESULT_PASS : BASELINE_DIAG_RESULT_FAIL;

  if ((baseline_diag_quiet == 0U) || (result != BASELINE_DIAG_RESULT_PASS))
  {
    baseline_diag_logf("[CTRL] state=%s %s",
                       control_loop_state_text(state),
                       baseline_diag_result_text(result));
  }
  return result;
}
#endif

/*
 * 上电后运行一次完整静态流水线。
 * FULL_STATIC_DIAG 会访问 ADC 和编码器；MCU_ONLY 会跳过这些外部链路。
 */
static baseline_diag_result_t baseline_diag_run_static_pipeline(void)
{
  baseline_diag_result_t total;
  motor_adc_raw_sample_t adc_sample;
  mt6701_snapshot_t enc_snapshot;

  total = BASELINE_DIAG_RESULT_PASS;
  if (baseline_diag_quiet == 0U)
  {
    baseline_diag_logf("[DIAG] begin static pipeline");
  }
  total = baseline_diag_worse(total, baseline_diag_check_boot());
  total = baseline_diag_worse(total, baseline_diag_check_drv());
  total = baseline_diag_worse(total, baseline_diag_check_pwm());

  if (baseline_diag_mode == BASELINE_DIAG_MODE_FULL_STATIC_DIAG)
  {
    total = baseline_diag_worse(total, baseline_diag_check_adc(&adc_sample));
    total = baseline_diag_worse(total, baseline_diag_check_mt6701(&enc_snapshot));
  }
  else
  {
    if (baseline_diag_quiet == 0U)
    {
      baseline_diag_logf("[ADC] skipped in MCU_ONLY");
      baseline_diag_logf("[ENC] skipped in MCU_ONLY");
    }
  }

  total = baseline_diag_worse(total, baseline_diag_check_control());
  baseline_diag_logf("[DIAG] static pipeline result=%s", baseline_diag_result_text(total));
  return total;
}

#if BASELINE_DIAG_PERIODIC_LOG_ENABLE
/* 周期监视：重复读取 ADC/编码器，并汇总控制状态，方便串口持续观察。 */
static baseline_diag_result_t baseline_diag_monitor_full_static(void)
{
  motor_adc_raw_sample_t adc_sample;
  mt6701_snapshot_t enc_snapshot;
  baseline_diag_result_t adc_result;
  baseline_diag_result_t enc_result;
  baseline_diag_result_t ctrl_result;
  baseline_diag_result_t summary;

  adc_result = baseline_diag_check_adc(&adc_sample);
  enc_result = baseline_diag_check_mt6701(&enc_snapshot);
  ctrl_result = (control_loop_get_state() == CONTROL_LOOP_STATE_FAULT) ?
                BASELINE_DIAG_RESULT_FAIL : BASELINE_DIAG_RESULT_PASS;

  summary = baseline_diag_worse(adc_result, enc_result);
  summary = baseline_diag_worse(summary, ctrl_result);

  baseline_diag_logf("[MON] mode=%s drv=%s adc=%s enc=%s ctrl=%s",
                     baseline_diag_mode_text(baseline_diag_mode),
                     motor_drv_is_enabled() != 0U ? "enabled" : "disabled",
                     baseline_diag_result_text(adc_result),
                     baseline_diag_result_text(enc_result),
                     baseline_diag_result_text(ctrl_result));
  baseline_diag_last_result = summary;
  return summary;
}
#endif

/* 设置诊断模式；非法枚举直接忽略，避免误写导致未知行为。 */
void baseline_diag_set_mode(baseline_diag_mode_t mode)
{
  if ((mode != BASELINE_DIAG_MODE_MCU_ONLY) &&
      (mode != BASELINE_DIAG_MODE_FULL_STATIC_DIAG))
  {
    return;
  }

  baseline_diag_mode = mode;
}

/* 返回当前诊断模式。 */
baseline_diag_mode_t baseline_diag_get_mode(void)
{
  return baseline_diag_mode;
}

/* 返回最近一次综合诊断结果。 */
baseline_diag_result_t baseline_diag_get_last_result(void)
{
  return baseline_diag_last_result;
}

/* `i` 命令入口：临时关闭静默，全量重跑静态流水线（明细逐行输出）后恢复。 */
void baseline_diag_run_verbose_pipeline(void)
{
  baseline_diag_quiet = 0U;
  baseline_diag_last_result = baseline_diag_run_static_pipeline();
  baseline_diag_quiet = 1U;
}

/*
 * 初始化诊断模块。
 * 先强制失能驱动并清零 PWM，再初始化控制层，最后运行一次静态流水线。
 */
void baseline_diag_init(void)
{
  motor_drv_disable();
  motor_pwm_force_zero();
  motor_pwm_stop_output();
  (void)motor_adc_init_static();
  (void)motor_adc_calibrate_zero_current();
  baseline_diag_log_zero_samples();
#if APP_MODE_RUN
  /* 运行态由 app_run 自持状态机，控制层初始化/轮询不经此处。 */
#else
  control_loop_init();
#endif
#if BASELINE_DIAG_PERIODIC_LOG_ENABLE
  baseline_diag_initialized = 1U;
#endif
#if BASELINE_DIAG_PERIODIC_LOG_ENABLE
  baseline_diag_last_monitor_tick = HAL_GetTick();
#endif
  baseline_diag_last_result = baseline_diag_run_static_pipeline();
}

/*
 * 周期轮询入口。
 * 每次先让控制层 poll 一次；达到 500 ms 周期后输出监视日志。
 */
void baseline_diag_poll(void)
{
#if BASELINE_DIAG_PERIODIC_LOG_ENABLE
  uint32_t now;
#endif

#if APP_MODE_RUN
  /* 运行态主循环在 app_run_poll，此处不驱动控制层。 */
#else
  control_loop_poll();
#endif

#if BASELINE_DIAG_PERIODIC_LOG_ENABLE
  if (baseline_diag_initialized == 0U)
  {
    return;
  }

  now = HAL_GetTick();
  if ((uint32_t)(now - baseline_diag_last_monitor_tick) < BASELINE_DIAG_MONITOR_PERIOD_MS)
  {
    return;
  }

  baseline_diag_last_monitor_tick = now;

  if (baseline_diag_mode == BASELINE_DIAG_MODE_FULL_STATIC_DIAG)
  {
    baseline_diag_monitor_full_static();
  }
  else
  {
    baseline_diag_result_t summary;

    summary = (control_loop_get_state() == CONTROL_LOOP_STATE_SAFE_IDLE) ?
              BASELINE_DIAG_RESULT_PASS : BASELINE_DIAG_RESULT_FAIL;
    baseline_diag_last_result = summary;
    baseline_diag_logf("[MON] mode=%s ctrl=%s",
                       baseline_diag_mode_text(baseline_diag_mode),
                       baseline_diag_result_text(summary));
  }
#endif
}
