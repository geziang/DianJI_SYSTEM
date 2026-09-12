#include "foc_identification.h"

static float foc_identification_abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

foc_status_t foc_identification_estimate_resistance(float steady_voltage_v,
                                                     float steady_current_a,
                                                     float minimum_current_a,
                                                     float *resistance_ohm)
{
  if (resistance_ohm == 0)
  {
    return FOC_STATUS_INVALID_ARG;
  }
  if ((minimum_current_a <= 0.0f) ||
      (foc_identification_abs(steady_current_a) < minimum_current_a))
  {
    return FOC_STATUS_INVALID_SAMPLE;
  }
  *resistance_ohm = foc_identification_abs(steady_voltage_v / steady_current_a);
  return (*resistance_ohm > 0.0f) ? FOC_STATUS_OK : FOC_STATUS_OUT_OF_RANGE;
}

foc_status_t foc_identification_estimate_inductance(float pulse_voltage_v,
                                                     float resistance_ohm,
                                                     float current_a,
                                                     float current_slope_a_per_s,
                                                     float *inductance_h)
{
  float effective_voltage_v;

  if (inductance_h == 0)
  {
    return FOC_STATUS_INVALID_ARG;
  }
  if ((resistance_ohm <= 0.0f) || (current_slope_a_per_s == 0.0f))
  {
    return FOC_STATUS_INVALID_SAMPLE;
  }
  effective_voltage_v = pulse_voltage_v - (resistance_ohm * current_a);
  *inductance_h = foc_identification_abs(effective_voltage_v / current_slope_a_per_s);
  return (*inductance_h > 0.0f) ? FOC_STATUS_OK : FOC_STATUS_OUT_OF_RANGE;
}

foc_status_t foc_identification_derive_current_pi(float resistance_ohm,
                                                   float inductance_h,
                                                   float control_time_constant_s,
                                                   float voltage_limit_v,
                                                   foc_current_pi_result_t *result)
{
  if (result == 0)
  {
    return FOC_STATUS_INVALID_ARG;
  }
  if ((resistance_ohm <= 0.0f) || (inductance_h <= 0.0f) ||
      (control_time_constant_s <= 0.0f) || (voltage_limit_v <= 0.0f))
  {
    return FOC_STATUS_OUT_OF_RANGE;
  }

  result->id_pi.kp = inductance_h / control_time_constant_s;
  result->id_pi.ki = resistance_ohm / control_time_constant_s;
  result->id_pi.integrator_min = -voltage_limit_v;
  result->id_pi.integrator_max = voltage_limit_v;
  result->id_pi.output_min = -voltage_limit_v;
  result->id_pi.output_max = voltage_limit_v;
  result->iq_pi = result->id_pi;
  result->control_time_constant_s = control_time_constant_s;
  return FOC_STATUS_OK;
}
