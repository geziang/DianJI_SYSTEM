#ifndef FOC_IDENTIFICATION_H
#define FOC_IDENTIFICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

typedef struct
{
  float phase_resistance_ohm;
  float phase_inductance_h;
  float ld_h;
  float lq_h;
  uint8_t ld_lq_valid;
} foc_identification_result_t;

typedef struct
{
  foc_pi_config_t id_pi;
  foc_pi_config_t iq_pi;
  float control_time_constant_s;
} foc_current_pi_result_t;

foc_status_t foc_identification_estimate_resistance(float steady_voltage_v,
                                                     float steady_current_a,
                                                     float minimum_current_a,
                                                     float *resistance_ohm);
foc_status_t foc_identification_estimate_inductance(float pulse_voltage_v,
                                                     float resistance_ohm,
                                                     float current_a,
                                                     float current_slope_a_per_s,
                                                     float *inductance_h);
foc_status_t foc_identification_derive_current_pi(float resistance_ohm,
                                                   float inductance_h,
                                                   float control_time_constant_s,
                                                   float voltage_limit_v,
                                                   foc_current_pi_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* FOC_IDENTIFICATION_H */
