#include "safety_manager.h"

#include "control_loop.h"
#if APP_MODE_RUN
#include "app_run.h"
#endif
#include "motor_adc.h"
#include "motor_drv.h"
#include "motor_parameter_store.h"
#include "motor_pwm.h"
#include "mt6701.h"

static safety_manager_state_t safety_manager_state;
static safety_manager_fault_t safety_manager_fault;

static void safety_manager_safe_disable(void)
{
  motor_drv_disable();
  motor_pwm_force_zero();
  motor_pwm_stop_output();
}

void safety_manager_latch_fault(safety_manager_fault_t fault)
{
  safety_manager_safe_disable();
  safety_manager_fault = fault;
  safety_manager_state = SAFETY_MANAGER_STATE_FAULT;
}

void safety_manager_init(void)
{
  safety_manager_safe_disable();
  motor_parameter_store_init();
  safety_manager_fault = SAFETY_MANAGER_FAULT_NONE;
  safety_manager_state = SAFETY_MANAGER_STATE_BOOT_SAFE;
}

void safety_manager_poll(void)
{
  motor_parameter_package_t parameters;
  motor_adc_status_t adc_status;

  switch (safety_manager_state)
  {
    case SAFETY_MANAGER_STATE_BOOT_SAFE:
      safety_manager_safe_disable();
      safety_manager_state = SAFETY_MANAGER_STATE_PARAMETER_CHECK;
      break;
    case SAFETY_MANAGER_STATE_PARAMETER_CHECK:
      if (motor_parameter_store_load(&parameters) != MOTOR_PARAMETER_STORE_OK)
      {
        safety_manager_state = SAFETY_MANAGER_STATE_CALIBRATION_REQUIRED;
      }
      else
      {
        /* FR-2.4：加载成功的已固化包注入控制参数集（符号/转子/静态 R、L
         * + v2 学习值重建），日志打印来源页/revision。
         * phase_map 不注入（P3 恒等契约 2026-09-13），只存 control_loop_loaded 镜像。
         * 运行态固件改灌 app_run（map 从包直取，DD-01 契约）。 */
#if APP_MODE_RUN
        app_run_load_package(&parameters);
#else
        control_loop_load_committed_parameters(&parameters);
#endif
        safety_manager_state = SAFETY_MANAGER_STATE_ADC_ZERO_REFRESH;
      }
      break;
    case SAFETY_MANAGER_STATE_ADC_ZERO_REFRESH:
      adc_status = motor_adc_init_static();
      if (adc_status == MOTOR_ADC_STATUS_OK)
      {
        adc_status = motor_adc_calibrate_zero_current();
      }
      if (adc_status != MOTOR_ADC_STATUS_OK)
      {
        safety_manager_latch_fault(SAFETY_MANAGER_FAULT_ADC);
      }
      else
      {
        safety_manager_state = SAFETY_MANAGER_STATE_SENSOR_CHECK;
      }
      break;
    case SAFETY_MANAGER_STATE_SENSOR_CHECK:
      if (mt6701_is_connected() != MT6701_STATUS_OK)
      {
        safety_manager_latch_fault(SAFETY_MANAGER_FAULT_ENCODER);
      }
      else
      {
        safety_manager_state = SAFETY_MANAGER_STATE_READY;
      }
      break;
    case SAFETY_MANAGER_STATE_READY:
    case SAFETY_MANAGER_STATE_IDENT_PRECHECK:
    case SAFETY_MANAGER_STATE_LIMITED_FOC_READY:
    case SAFETY_MANAGER_STATE_LIMITED_FOC_RUN:
    case SAFETY_MANAGER_STATE_SAFE_STOP:
    case SAFETY_MANAGER_STATE_CALIBRATION_REQUIRED:
    case SAFETY_MANAGER_STATE_FAULT:
    default:
      break;
  }
}

void safety_manager_request_safe_stop(void)
{
  safety_manager_safe_disable();
  if (safety_manager_state != SAFETY_MANAGER_STATE_FAULT)
  {
    safety_manager_state = SAFETY_MANAGER_STATE_SAFE_STOP;
  }
}

uint8_t safety_manager_request_fault_clear(void)
{
  if (safety_manager_state != SAFETY_MANAGER_STATE_FAULT)
  {
    return 0U;
  }

  safety_manager_safe_disable();
  safety_manager_fault = SAFETY_MANAGER_FAULT_NONE;
  safety_manager_state = SAFETY_MANAGER_STATE_BOOT_SAFE;
  return 1U;
}

uint8_t safety_manager_is_limited_power_allowed(void)
{
  /* P0/P2 never enables power: current scale, polarity and sample window lack evidence. */
  return 0U;
}

safety_manager_state_t safety_manager_get_state(void)
{
  return safety_manager_state;
}

safety_manager_fault_t safety_manager_get_fault(void)
{
  return safety_manager_fault;
}

const char *safety_manager_state_text(safety_manager_state_t state)
{
  switch (state)
  {
    case SAFETY_MANAGER_STATE_BOOT_SAFE: return "BOOT_SAFE";
    case SAFETY_MANAGER_STATE_PARAMETER_CHECK: return "PARAMETER_CHECK";
    case SAFETY_MANAGER_STATE_ADC_ZERO_REFRESH: return "ADC_ZERO_REFRESH";
    case SAFETY_MANAGER_STATE_SENSOR_CHECK: return "SENSOR_CHECK";
    case SAFETY_MANAGER_STATE_READY: return "READY";
    case SAFETY_MANAGER_STATE_IDENT_PRECHECK: return "IDENT_PRECHECK";
    case SAFETY_MANAGER_STATE_LIMITED_FOC_READY: return "LIMITED_FOC_READY";
    case SAFETY_MANAGER_STATE_LIMITED_FOC_RUN: return "LIMITED_FOC_RUN";
    case SAFETY_MANAGER_STATE_SAFE_STOP: return "SAFE_STOP";
    case SAFETY_MANAGER_STATE_CALIBRATION_REQUIRED: return "CALIBRATION_REQUIRED";
    case SAFETY_MANAGER_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

const char *safety_manager_fault_text(safety_manager_fault_t fault)
{
  switch (fault)
  {
    case SAFETY_MANAGER_FAULT_NONE: return "NONE";
    case SAFETY_MANAGER_FAULT_ADC: return "ADC";
    case SAFETY_MANAGER_FAULT_ENCODER: return "ENCODER";
    case SAFETY_MANAGER_FAULT_PARAMETER: return "PARAMETER";
    case SAFETY_MANAGER_FAULT_SAMPLE_TIMEOUT: return "SAMPLE_TIMEOUT";
    case SAFETY_MANAGER_FAULT_SAMPLE_SATURATED: return "SAMPLE_SATURATED";
    case SAFETY_MANAGER_FAULT_CONTROL_TIMEOUT: return "CONTROL_TIMEOUT";
    default: return "UNKNOWN";
  }
}
