#include "foc_experiment_config.h"

static float foc_experiment_min(float left, float right)
{
  return (left < right) ? left : right;
}

foc_status_t foc_experiment_derive_current_limit(const foc_current_limit_sources_t *sources,
                                                  float *current_limit_a)
{
  float limit;

  if ((sources == 0) || (current_limit_a == 0))
  {
    return FOC_STATUS_INVALID_ARG;
  }
  if ((sources->motor_continuous_current_a <= 0.0f) ||
      (sources->driver_confirmed_current_a <= 0.0f) ||
      (sources->supply_limit_current_a <= 0.0f) ||
      (sources->sampling_valid_current_a <= 0.0f) ||
      (sources->experiment_current_limit_a <= 0.0f))
  {
    return FOC_STATUS_NOT_READY;
  }
  limit = foc_experiment_min(sources->motor_continuous_current_a,
                             sources->driver_confirmed_current_a);
  limit = foc_experiment_min(limit, sources->supply_limit_current_a);
  limit = foc_experiment_min(limit, sources->sampling_valid_current_a);
  *current_limit_a = foc_experiment_min(limit, sources->experiment_current_limit_a);
  return FOC_STATUS_OK;
}

void foc_experiment_load_safe_defaults(foc_experiment_config_t *config)
{
  if (config == 0)
  {
    return;
  }
  /* Zero is deliberately a disabled output limit until measurements supply evidence. */
  config->max_current_a = 0.0f;
  config->max_voltage_v = 0.0f;
  config->max_modulation = 0.0f;
  config->r_identification_current_a = 0.0f;
  config->l_pulse_voltage_v = 0.0f;
  config->l_pulse_duration_us = 0U;
}
