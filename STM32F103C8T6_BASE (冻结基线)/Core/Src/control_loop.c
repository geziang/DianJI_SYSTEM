/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    control_loop.c
  * @brief   Safe low-speed open-loop control state machine.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "control_loop.h"

#include "board_config.h"
#include "debug_log.h"
#include "motor_adc.h"
#include "motor_drv.h"
#include "motor_pwm.h"
#include "mt6701.h"
#include "open_loop.h"

#include <stdio.h>

/* Conservative first-run values for a <=10 V V3P, empty-load experiment. */
#define CONTROL_LOOP_ALIGN_TIME_MS             (300U)
#define CONTROL_LOOP_RUN_TIMEOUT_MS            (3000U)
#define CONTROL_LOOP_MONITOR_PERIOD_MS         (100U)
#define CONTROL_LOOP_ALIGN_MODULATION_PERMILLE (50U)
#define CONTROL_LOOP_RUN_MODULATION_PERMILLE   (100U)
#define CONTROL_LOOP_START_HZ_X10              (50U)
#define CONTROL_LOOP_TARGET_HZ_X10             (200U)
#define CONTROL_LOOP_RAMP_TIME_MS              (1000U)
#define CONTROL_LOOP_ADC_MAX_RAW               (4090U)
#define CONTROL_LOOP_ENCODER_FAILURE_LIMIT     (2U)

static control_loop_state_t control_loop_state = CONTROL_LOOP_STATE_SAFE_IDLE;
static control_loop_fault_t control_loop_fault = CONTROL_LOOP_FAULT_NONE;
static uint32_t control_loop_state_tick;
static uint32_t control_loop_last_step_tick;
static uint32_t control_loop_last_monitor_tick;
static uint8_t control_loop_encoder_failure_count;
/* 开环旋转时的编码器遥测节流（owner 诊断需求 2026-09-14：旋转期打印编码器，
 * 观察其是否跟随/失效；冻结原则偏离已登记 readme）。 */
static uint32_t control_loop_enc_print_ms;

static void control_loop_safe_disable(void)
{
  /* Remove gate-drive enable before clearing or stopping PWM. */
  motor_drv_disable();
  motor_pwm_force_zero();
  motor_pwm_stop_output();
}

static void control_loop_enter_fault(control_loop_fault_t fault)
{
  control_loop_safe_disable();
  control_loop_fault = fault;
  control_loop_state = CONTROL_LOOP_STATE_FAULT;
}

static uint16_t control_loop_run_frequency(uint32_t elapsed_ms)
{
  uint32_t delta;

  if (elapsed_ms >= CONTROL_LOOP_RAMP_TIME_MS)
  {
    return CONTROL_LOOP_TARGET_HZ_X10;
  }

  delta = ((uint32_t)(CONTROL_LOOP_TARGET_HZ_X10 - CONTROL_LOOP_START_HZ_X10) * elapsed_ms) /
          CONTROL_LOOP_RAMP_TIME_MS;
  return (uint16_t)(CONTROL_LOOP_START_HZ_X10 + delta);
}

static uint8_t control_loop_apply_svpwm(uint16_t modulation_permille)
{
  open_loop_pwm_t pwm;

  open_loop_make_svpwm(modulation_permille,
                       BOARD_CONFIG_PWM_PERIOD_TICKS,
                       &pwm);
  return (uint8_t)(motor_pwm_set_raw(pwm.phase_a_ticks,
                                     pwm.phase_b_ticks,
                                     pwm.phase_c_ticks) == MOTOR_PWM_STATUS_OK);
}

static void control_loop_monitor_inputs(void)
{
  motor_adc_raw_sample_t adc_sample;
  uint16_t encoder_angle;
  mt6701_status_t encoder_status;

  if (motor_adc_read_raw(&adc_sample) != MOTOR_ADC_STATUS_OK)
  {
    control_loop_enter_fault(CONTROL_LOOP_FAULT_ADC);
    return;
  }

  if ((adc_sample.phase_u_raw >= CONTROL_LOOP_ADC_MAX_RAW) ||
      (adc_sample.phase_v_raw >= CONTROL_LOOP_ADC_MAX_RAW))
  {
    control_loop_enter_fault(CONTROL_LOOP_FAULT_ADC);
    return;
  }

  encoder_status = mt6701_read_raw_angle(&encoder_angle);
  if (encoder_status == MT6701_STATUS_OK)
  {
    control_loop_encoder_failure_count = 0U;
    return;
  }

  control_loop_encoder_failure_count++;
  if (control_loop_encoder_failure_count >= CONTROL_LOOP_ENCODER_FAILURE_LIMIT)
  {
    control_loop_enter_fault(CONTROL_LOOP_FAULT_ENCODER);
  }
}

void control_loop_init(void)
{
  control_loop_safe_disable();
  open_loop_reset();
  control_loop_state = CONTROL_LOOP_STATE_SAFE_IDLE;
  control_loop_fault = CONTROL_LOOP_FAULT_NONE;
  control_loop_state_tick = HAL_GetTick();
  control_loop_last_step_tick = control_loop_state_tick;
  control_loop_last_monitor_tick = control_loop_state_tick;
  control_loop_encoder_failure_count = 0U;
}

control_loop_command_status_t control_loop_request_open_loop_start(void)
{
  if (control_loop_state == CONTROL_LOOP_STATE_FAULT)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_FAULT;
  }
  if (control_loop_state != CONTROL_LOOP_STATE_SAFE_IDLE)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }

  control_loop_state = CONTROL_LOOP_STATE_OPEN_LOOP_READY;
  control_loop_state_tick = HAL_GetTick();
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

void control_loop_request_stop(void)
{
  if ((control_loop_state != CONTROL_LOOP_STATE_SAFE_IDLE) &&
      (control_loop_state != CONTROL_LOOP_STATE_FAULT))
  {
    control_loop_state = CONTROL_LOOP_STATE_STOPPING;
  }
}

control_loop_command_status_t control_loop_request_fault_clear(void)
{
  if (control_loop_state != CONTROL_LOOP_STATE_FAULT)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }

  control_loop_safe_disable();
  control_loop_fault = CONTROL_LOOP_FAULT_NONE;
  control_loop_state = CONTROL_LOOP_STATE_SAFE_IDLE;
  control_loop_state_tick = HAL_GetTick();
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

void control_loop_poll(void)
{
  uint32_t now;
  uint32_t state_elapsed;
  uint32_t step_elapsed;

  now = HAL_GetTick();

  switch (control_loop_state)
  {
    case CONTROL_LOOP_STATE_OPEN_LOOP_READY:
      control_loop_monitor_inputs();
      if (control_loop_state == CONTROL_LOOP_STATE_FAULT)
      {
        return;
      }
      if (motor_pwm_start_test_output() != MOTOR_PWM_STATUS_OK)
      {
        control_loop_enter_fault(CONTROL_LOOP_FAULT_PWM_START);
        return;
      }
      open_loop_reset();
      if (control_loop_apply_svpwm(CONTROL_LOOP_ALIGN_MODULATION_PERMILLE) == 0U)
      {
        control_loop_enter_fault(CONTROL_LOOP_FAULT_PWM_START);
        return;
      }
      if (motor_drv_enable_for_test() != MOTOR_DRV_STATUS_OK)
      {
        control_loop_enter_fault(CONTROL_LOOP_FAULT_DRIVER_ENABLE);
        return;
      }
      control_loop_state = CONTROL_LOOP_STATE_ALIGN;
      control_loop_state_tick = now;
      control_loop_last_step_tick = now;
      control_loop_last_monitor_tick = now;
      break;

    case CONTROL_LOOP_STATE_ALIGN:
      if ((uint32_t)(now - control_loop_state_tick) >= CONTROL_LOOP_ALIGN_TIME_MS)
      {
        control_loop_state = CONTROL_LOOP_STATE_OPEN_LOOP_RUN;
        control_loop_state_tick = now;
        control_loop_last_step_tick = now;
      }
      break;

    case CONTROL_LOOP_STATE_OPEN_LOOP_RUN:
      state_elapsed = (uint32_t)(now - control_loop_state_tick);
      if (state_elapsed >= CONTROL_LOOP_RUN_TIMEOUT_MS)
      {
        control_loop_enter_fault(CONTROL_LOOP_FAULT_TIMEOUT);
        return;
      }
      step_elapsed = (uint32_t)(now - control_loop_last_step_tick);
      if (step_elapsed != 0U)
      {
        open_loop_advance(step_elapsed, control_loop_run_frequency(state_elapsed));
        if (control_loop_apply_svpwm(CONTROL_LOOP_RUN_MODULATION_PERMILLE) == 0U)
        {
          control_loop_enter_fault(CONTROL_LOOP_FAULT_PWM_START);
          return;
        }
        control_loop_last_step_tick = now;
      }
      /* 编码器遥测：旋转期间每 200ms 打印原始角（14bit/16384）与读取状态，
       * 供观察编码器是否跟随开环旋转/何时失效（owner 诊断需求 2026-09-14）。 */
      if ((now - control_loop_enc_print_ms) >= 200U)
      {
        uint16_t enc_raw;
        char enc_line[96];

        control_loop_enc_print_ms = now;
        if (mt6701_read_raw_angle(&enc_raw) == MT6701_STATUS_OK)
        {
          (void)snprintf(enc_line, sizeof(enc_line),
                         "[OL] t=%lums enc=%u (%.1fdeg) OK",
                         (unsigned long)state_elapsed,
                         (unsigned)enc_raw,
                         (double)((float)enc_raw * 360.0f / 16384.0f));
        }
        else
        {
          (void)snprintf(enc_line, sizeof(enc_line),
                         "[OL] t=%lums enc=READ FAIL",
                         (unsigned long)state_elapsed);
        }
        debug_log_write_line(enc_line);
      }
      break;

    case CONTROL_LOOP_STATE_STOPPING:
      control_loop_safe_disable();
      control_loop_state = CONTROL_LOOP_STATE_SAFE_IDLE;
      control_loop_state_tick = now;
      break;

    case CONTROL_LOOP_STATE_SAFE_IDLE:
    case CONTROL_LOOP_STATE_FAULT:
    default:
      break;
  }

  if (((control_loop_state == CONTROL_LOOP_STATE_ALIGN) ||
       (control_loop_state == CONTROL_LOOP_STATE_OPEN_LOOP_RUN)) &&
      ((uint32_t)(now - control_loop_last_monitor_tick) >= CONTROL_LOOP_MONITOR_PERIOD_MS))
  {
    control_loop_last_monitor_tick = now;
    control_loop_monitor_inputs();
  }
}

control_loop_state_t control_loop_get_state(void)
{
  return control_loop_state;
}

control_loop_fault_t control_loop_get_fault(void)
{
  return control_loop_fault;
}

const char *control_loop_state_text(control_loop_state_t state)
{
  switch (state)
  {
    case CONTROL_LOOP_STATE_SAFE_IDLE: return "SAFE_IDLE";
    case CONTROL_LOOP_STATE_OPEN_LOOP_READY: return "OPEN_LOOP_READY";
    case CONTROL_LOOP_STATE_ALIGN: return "ALIGN";
    case CONTROL_LOOP_STATE_OPEN_LOOP_RUN: return "OPEN_LOOP_RUN";
    case CONTROL_LOOP_STATE_STOPPING: return "STOPPING";
    case CONTROL_LOOP_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

const char *control_loop_fault_text(control_loop_fault_t fault)
{
  switch (fault)
  {
    case CONTROL_LOOP_FAULT_NONE: return "NONE";
    case CONTROL_LOOP_FAULT_PWM_START: return "PWM_START";
    case CONTROL_LOOP_FAULT_DRIVER_ENABLE: return "DRIVER_ENABLE";
    case CONTROL_LOOP_FAULT_ADC: return "ADC";
    case CONTROL_LOOP_FAULT_ENCODER: return "ENCODER";
    case CONTROL_LOOP_FAULT_TIMEOUT: return "TIMEOUT";
    default: return "UNKNOWN";
  }
}
