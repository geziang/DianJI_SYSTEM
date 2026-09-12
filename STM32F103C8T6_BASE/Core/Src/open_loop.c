/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    open_loop.c
  * @brief   Low-speed open-loop voltage vector and SVPWM helper.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "open_loop.h"

#include <stddef.h>

#define OPEN_LOOP_TABLE_POINTS       (24U)
#define OPEN_LOOP_SECTOR_FRACTIONS   (256U)
#define OPEN_LOOP_PHASE_OFFSET        (2048U)
#define OPEN_LOOP_Q15_ONE             (32767L)

/* One electrical sine period in Q15, sampled every 15 degrees. */
static const int16_t open_loop_sine_table[OPEN_LOOP_TABLE_POINTS] =
{
      0,  8481, 16384, 23170, 28377, 31650,
  32767, 31650, 28377, 23170, 16384,  8481,
      0, -8481,-16384,-23170,-28377,-31650,
 -32767,-31650,-28377,-23170,-16384, -8481
};

static uint16_t open_loop_angle;

static int32_t open_loop_sine_q15(uint16_t angle)
{
  uint16_t wrapped;
  uint16_t index;
  uint16_t fraction;
  int32_t left;
  int32_t right;

  wrapped = (uint16_t)(angle % OPEN_LOOP_ANGLE_FULL_SCALE);
  index = (uint16_t)(wrapped / OPEN_LOOP_SECTOR_FRACTIONS);
  fraction = (uint16_t)(wrapped % OPEN_LOOP_SECTOR_FRACTIONS);
  left = open_loop_sine_table[index];
  right = open_loop_sine_table[(index + 1U) % OPEN_LOOP_TABLE_POINTS];

  return left + (((right - left) * (int32_t)fraction) /
                 (int32_t)OPEN_LOOP_SECTOR_FRACTIONS);
}

void open_loop_reset(void)
{
  open_loop_angle = 0U;
}

void open_loop_advance(uint32_t elapsed_ms, uint16_t electrical_hz_x10)
{
  uint32_t increment;

  increment = ((uint32_t)electrical_hz_x10 * OPEN_LOOP_ANGLE_FULL_SCALE * elapsed_ms) / 10000UL;
  open_loop_angle = (uint16_t)((open_loop_angle + increment) % OPEN_LOOP_ANGLE_FULL_SCALE);
}

void open_loop_set_angle(uint16_t electrical_angle)
{
  open_loop_angle = (uint16_t)(electrical_angle % OPEN_LOOP_ANGLE_FULL_SCALE);
}

uint16_t open_loop_get_angle(void)
{
  return open_loop_angle;
}

void open_loop_make_svpwm(uint16_t modulation_permille,
                          uint32_t pwm_period_ticks,
                          open_loop_pwm_t *pwm)
{
  int32_t phase_a;
  int32_t phase_b;
  int32_t phase_c;
  int32_t maximum;
  int32_t minimum;
  int32_t common_mode;
  int32_t half_period;

  if (pwm == NULL)
  {
    return;
  }

  /* Keep this helper safe even if a future caller supplies an out-of-range value. */
  if (modulation_permille > 1000U)
  {
    modulation_permille = 1000U;
  }

  phase_a = open_loop_sine_q15(open_loop_angle);
  phase_b = open_loop_sine_q15((uint16_t)((open_loop_angle + OPEN_LOOP_PHASE_OFFSET) %
                                           OPEN_LOOP_ANGLE_FULL_SCALE));
  phase_c = open_loop_sine_q15((uint16_t)((open_loop_angle + (2U * OPEN_LOOP_PHASE_OFFSET)) %
                                           OPEN_LOOP_ANGLE_FULL_SCALE));

  maximum = phase_a;
  if (phase_b > maximum) maximum = phase_b;
  if (phase_c > maximum) maximum = phase_c;
  minimum = phase_a;
  if (phase_b < minimum) minimum = phase_b;
  if (phase_c < minimum) minimum = phase_c;
  common_mode = (maximum + minimum) / 2L;
  half_period = (int32_t)(pwm_period_ticks / 2U);

  /* Divide before multiplying by the PWM period to keep the 32-bit path bounded. */
  phase_a = ((phase_a - common_mode) * (int32_t)modulation_permille) / 1000L;
  phase_b = ((phase_b - common_mode) * (int32_t)modulation_permille) / 1000L;
  phase_c = ((phase_c - common_mode) * (int32_t)modulation_permille) / 1000L;

  phase_a = half_period + ((phase_a * half_period) / OPEN_LOOP_Q15_ONE);
  phase_b = half_period + ((phase_b * half_period) / OPEN_LOOP_Q15_ONE);
  phase_c = half_period + ((phase_c * half_period) / OPEN_LOOP_Q15_ONE);

  if (phase_a < 0L) phase_a = 0L;
  if (phase_b < 0L) phase_b = 0L;
  if (phase_c < 0L) phase_c = 0L;
  if (phase_a > (int32_t)pwm_period_ticks) phase_a = (int32_t)pwm_period_ticks;
  if (phase_b > (int32_t)pwm_period_ticks) phase_b = (int32_t)pwm_period_ticks;
  if (phase_c > (int32_t)pwm_period_ticks) phase_c = (int32_t)pwm_period_ticks;

  pwm->phase_a_ticks = (uint32_t)phase_a;
  pwm->phase_b_ticks = (uint32_t)phase_b;
  pwm->phase_c_ticks = (uint32_t)phase_c;
}
