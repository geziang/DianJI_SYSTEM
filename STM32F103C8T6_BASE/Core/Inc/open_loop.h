/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    open_loop.h
  * @brief   Low-speed open-loop voltage vector and SVPWM helper.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef OPEN_LOOP_H
#define OPEN_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Electrical angle uses 24 sine-table sectors, each split into 256 fractions. */
#define OPEN_LOOP_ANGLE_FULL_SCALE  (6144U)

typedef struct
{
  uint32_t phase_a_ticks;
  uint32_t phase_b_ticks;
  uint32_t phase_c_ticks;
} open_loop_pwm_t;

/* Reset the electrical-angle accumulator to zero. */
void open_loop_reset(void);

/* Advance electrical angle by elapsed milliseconds at 0.1 Hz resolution. */
void open_loop_advance(uint32_t elapsed_ms, uint16_t electrical_hz_x10);

/* Set the electrical angle explicitly for the alignment stage. */
void open_loop_set_angle(uint16_t electrical_angle);

/* Return the current electrical angle in OPEN_LOOP_ANGLE_FULL_SCALE units. */
uint16_t open_loop_get_angle(void);

/*
 * Generate centered SVPWM duties. modulation_permille is limited by the caller
 * to a conservative range; 100 means 10 percent of the available modulation.
 */
void open_loop_make_svpwm(uint16_t modulation_permille,
                          uint32_t pwm_period_ticks,
                          open_loop_pwm_t *pwm);

#ifdef __cplusplus
}
#endif

#endif /* OPEN_LOOP_H */
