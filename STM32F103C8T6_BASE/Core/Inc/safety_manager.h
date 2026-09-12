#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  SAFETY_MANAGER_STATE_BOOT_SAFE = 0,
  SAFETY_MANAGER_STATE_PARAMETER_CHECK,
  SAFETY_MANAGER_STATE_ADC_ZERO_REFRESH,
  SAFETY_MANAGER_STATE_SENSOR_CHECK,
  SAFETY_MANAGER_STATE_READY,
  SAFETY_MANAGER_STATE_IDENT_PRECHECK,
  SAFETY_MANAGER_STATE_LIMITED_FOC_READY,
  SAFETY_MANAGER_STATE_LIMITED_FOC_RUN,
  SAFETY_MANAGER_STATE_SAFE_STOP,
  SAFETY_MANAGER_STATE_CALIBRATION_REQUIRED,
  SAFETY_MANAGER_STATE_FAULT
} safety_manager_state_t;

typedef enum
{
  SAFETY_MANAGER_FAULT_NONE = 0,
  SAFETY_MANAGER_FAULT_ADC,
  SAFETY_MANAGER_FAULT_ENCODER,
  SAFETY_MANAGER_FAULT_PARAMETER,
  SAFETY_MANAGER_FAULT_SAMPLE_TIMEOUT,
  SAFETY_MANAGER_FAULT_SAMPLE_SATURATED,
  SAFETY_MANAGER_FAULT_CONTROL_TIMEOUT
} safety_manager_fault_t;

void safety_manager_init(void);
void safety_manager_poll(void);
void safety_manager_request_safe_stop(void);
void safety_manager_latch_fault(safety_manager_fault_t fault);
uint8_t safety_manager_request_fault_clear(void);
uint8_t safety_manager_is_limited_power_allowed(void);
safety_manager_state_t safety_manager_get_state(void);
safety_manager_fault_t safety_manager_get_fault(void);
const char *safety_manager_state_text(safety_manager_state_t state);
const char *safety_manager_fault_text(safety_manager_fault_t fault);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MANAGER_H */
