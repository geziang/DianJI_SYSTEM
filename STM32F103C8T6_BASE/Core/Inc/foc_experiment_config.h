#ifndef FOC_EXPERIMENT_CONFIG_H
#define FOC_EXPERIMENT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

typedef struct
{
  float motor_continuous_current_a;
  float driver_confirmed_current_a;
  float supply_limit_current_a;
  float sampling_valid_current_a;
  float experiment_current_limit_a;
} foc_current_limit_sources_t;

typedef struct
{
  float max_current_a;
  float max_voltage_v;
  float max_modulation;
  float r_identification_current_a;
  float l_pulse_voltage_v;
  uint32_t l_pulse_duration_us;
} foc_experiment_config_t;

foc_status_t foc_experiment_derive_current_limit(const foc_current_limit_sources_t *sources,
                                                  float *current_limit_a);
void foc_experiment_load_safe_defaults(foc_experiment_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* FOC_EXPERIMENT_CONFIG_H */
