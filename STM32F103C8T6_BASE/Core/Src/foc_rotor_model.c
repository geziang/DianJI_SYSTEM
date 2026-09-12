/*
 * FOC 转子角度系统数学库实现。
 *
 * 计算链路：MT6701 原始角 -> 机械 rad -> 方向和极对数 -> 电角 rad。
 * 方向、极对数和零位偏移都是待 FOC-PRE 实验确认的参数。
 */

#include "foc_rotor_model.h"
#include "foc_angle_math.h"

foc_status_t foc_rotor_model_convert(const foc_rotor_config_t *config,
                                     uint16_t mechanical_raw,
                                     foc_rotor_sample_t *sample)
{
  if ((config == 0) || (sample == 0))
  {
    return FOC_STATUS_INVALID_ARG;
  }
  if ((config->pole_pairs == 0U) ||
      ((config->encoder_direction != 1) &&
       (config->encoder_direction != -1)))
  {
    sample->valid = 0U;
    return FOC_STATUS_OUT_OF_RANGE;
  }

  sample->mechanical_raw = (uint16_t)(mechanical_raw & 0x3FFFU);
  sample->mechanical_angle_rad = foc_mechanical_raw_to_rad(sample->mechanical_raw);
  sample->electrical_angle_rad = foc_mechanical_to_electrical_rad(
      sample->mechanical_angle_rad,
      config->pole_pairs,
      config->encoder_direction,
      config->electrical_offset_rad);
  sample->valid = 1U;
  return FOC_STATUS_OK;
}
