/*
 * 电机参数包纯逻辑核心实现。零 HAL 依赖，PC 与 MCU 同一份编译（决策 D6）。
 */
#include "motor_parameter_core.h"

#include <stddef.h>

static uint32_t motor_parameter_crc_update(uint32_t crc, uint8_t byte)
{
  uint8_t bit;

  crc ^= byte;
  for (bit = 0U; bit < 8U; bit++)
  {
    crc = (crc & 1UL) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
  }
  return crc;
}

uint32_t motor_parameter_core_calculate_crc(const motor_parameter_package_t *package)
{
  const uint8_t *bytes;
  uint32_t crc;
  size_t index;

  if (package == NULL)
  {
    return 0U;
  }

  bytes = (const uint8_t *)package;
  crc = 0xFFFFFFFFUL;
  for (index = 0U; index < offsetof(motor_parameter_package_t, crc); index++)
  {
    crc = motor_parameter_crc_update(crc, bytes[index]);
  }
  for (index = offsetof(motor_parameter_package_t, current_gain_u_a_per_raw);
       index < sizeof(*package); index++)
  {
    crc = motor_parameter_crc_update(crc, bytes[index]);
  }
  return crc ^ 0xFFFFFFFFUL;
}

motor_parameter_core_status_t motor_parameter_core_validate(
    const motor_parameter_package_t *package,
    uint32_t hardware_id,
    uint32_t motor_id,
    uint32_t wiring_revision,
    float max_bus_voltage_v)
{
  if (package == NULL)
  {
    return MOTOR_PARAMETER_CORE_INVALID_ARG;
  }
  if ((package->format_version != MOTOR_PARAMETER_FORMAT_VERSION_1) &&
      (package->format_version != MOTOR_PARAMETER_FORMAT_VERSION_2))
  {
    return MOTOR_PARAMETER_CORE_FORMAT_ERROR;
  }
  if ((package->hardware_id != hardware_id) ||
      (package->motor_id != motor_id) ||
      (package->wiring_revision != wiring_revision))
  {
    return MOTOR_PARAMETER_CORE_ID_MISMATCH;
  }
  if (package->crc != motor_parameter_core_calculate_crc(package))
  {
    return MOTOR_PARAMETER_CORE_CRC_ERROR;
  }
  if ((package->current_gain_u_a_per_raw <= 0.0f) ||
      (package->current_gain_v_a_per_raw <= 0.0f) ||
      ((package->current_sign_u != 1) && (package->current_sign_u != -1)) ||
      ((package->current_sign_v != 1) && (package->current_sign_v != -1)) ||
      (package->phase_resistance_ohm <= 0.0f) ||
      (package->phase_inductance_h <= 0.0f) ||
      (package->current_limit_a <= 0.0f) ||
      (package->current_limit_a > 5.0f) ||
      (package->voltage_limit_v <= 0.0f) ||
      (package->voltage_limit_v > max_bus_voltage_v) ||
      (package->pole_pairs == 0U) ||
      ((package->encoder_direction != 1) && (package->encoder_direction != -1)) ||
      ((package->phase_map_a > 2U) || (package->phase_map_b > 2U) ||
       (package->phase_map_c > 2U)) ||
      ((package->phase_map_a == package->phase_map_b) ||
       (package->phase_map_a == package->phase_map_c) ||
       (package->phase_map_b == package->phase_map_c)))
  {
    return MOTOR_PARAMETER_CORE_RANGE_ERROR;
  }
  if ((package->state != MOTOR_PARAMETER_STATE_COMMITTED) &&
      (package->state != MOTOR_PARAMETER_STATE_FROZEN) &&
      (package->state != MOTOR_PARAMETER_STATE_STATIC_VALIDATED))
  {
    return MOTOR_PARAMETER_CORE_STATE_ERROR;
  }
  return MOTOR_PARAMETER_CORE_OK;
}

void motor_parameter_core_migrate_v1(motor_parameter_package_t *package)
{
  uint8_t i;

  if (package == NULL)
  {
    return;
  }
  package->format_version = MOTOR_PARAMETER_FORMAT_VERSION_2;
  package->phase_map_a = 0U;
  package->phase_map_b = 1U;
  package->phase_map_c = 2U;
  package->phase_seq_verified = 0U;
  package->learned_r_valid = 0U;
  package->learned_r_count = 0U;
  package->learned_phase_resistance_ohm = 0.0f;
  package->learned_r_revision = 0UL;
  for (i = 0U; i < MOTOR_PARAMETER_LEARN_HISTORY_N; i++)
  {
    package->learned_r_history_ohm[i] = 0.0f;
  }
  package->speed_kp = 0.0f;
  package->speed_ki = 0.0f;
  package->speed_limit_rad_s = 0.0f;
  package->accel_limit_rad_s2 = 0.0f;
  package->speed_estim_window_ms = 0.0f;
}

motor_parameter_page_select_t motor_parameter_core_select_page(
    const motor_parameter_package_t *page_a, uint8_t a_valid,
    const motor_parameter_package_t *page_b, uint8_t b_valid,
    motor_parameter_package_t *out)
{
  if ((a_valid != 0U) && (b_valid != 0U))
  {
    /* 双新：取 parameter_revision 新者；相等取 A（确定性，避免随机回退）。 */
    if (page_b->parameter_revision > page_a->parameter_revision)
    {
      if (out != NULL)
      {
        *out = *page_b;
      }
      return MOTOR_PARAMETER_PAGE_SELECT_B;
    }
    if (out != NULL)
    {
      *out = *page_a;
    }
    return MOTOR_PARAMETER_PAGE_SELECT_A;
  }
  if (a_valid != 0U)
  {
    if (out != NULL)
    {
      *out = *page_a;
    }
    return MOTOR_PARAMETER_PAGE_SELECT_A;
  }
  if (b_valid != 0U)
  {
    if (out != NULL)
    {
      *out = *page_b;
    }
    return MOTOR_PARAMETER_PAGE_SELECT_B;
  }
  return MOTOR_PARAMETER_PAGE_SELECT_NONE;
}

const char *motor_parameter_state_text(motor_parameter_state_t state)
{
  switch (state)
  {
    case MOTOR_PARAMETER_STATE_NOMINAL: return "NOMINAL";
    case MOTOR_PARAMETER_STATE_STATIC_VALIDATED: return "STATIC_VALIDATED";
    case MOTOR_PARAMETER_STATE_DYNAMIC_CANDIDATE: return "DYNAMIC_CANDIDATE";
    case MOTOR_PARAMETER_STATE_COMMITTED: return "COMMITTED";
    case MOTOR_PARAMETER_STATE_FROZEN: return "FROZEN";
    case MOTOR_PARAMETER_STATE_INVALID: return "INVALID";
    default: return "UNKNOWN";
  }
}
