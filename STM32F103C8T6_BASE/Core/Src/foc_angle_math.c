/*
 * FOC 角度基础数学库实现。
 *
 * 角度统一使用 rad；MT6701 原始角度为 0..16383，对应一个机械周期。
 * 方向和零位属于参数，不在本文件中猜测或修改。
 */

#include "foc_angle_math.h"

float foc_angle_wrap_rad(float angle_rad)
{
  while (angle_rad >= FOC_TWO_PI_F)
  {
    angle_rad -= FOC_TWO_PI_F;
  }
  while (angle_rad < 0.0f)
  {
    angle_rad += FOC_TWO_PI_F;
  }
  return angle_rad;
}

float foc_angle_wrap_signed_rad(float angle_rad)
{
  angle_rad = foc_angle_wrap_rad(angle_rad + FOC_PI_F) - FOC_PI_F;
  return angle_rad;
}

float foc_mechanical_raw_to_rad(uint16_t raw_angle)
{
  return ((float)(raw_angle & 0x3FFFU) * FOC_TWO_PI_F) / 16384.0f;
}

float foc_mechanical_to_electrical_rad(float mechanical_angle_rad,
                                       uint8_t pole_pairs,
                                       int8_t direction,
                                       float electrical_offset_rad)
{
  float signed_angle;

  if (direction < 0)
  {
    signed_angle = -mechanical_angle_rad;
  }
  else
  {
    signed_angle = mechanical_angle_rad;
  }

  return foc_angle_wrap_rad((signed_angle * (float)pole_pairs) +
                            electrical_offset_rad);
}
