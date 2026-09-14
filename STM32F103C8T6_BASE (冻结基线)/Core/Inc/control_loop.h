/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    control_loop.h
  * @brief   Safe low-speed open-loop control state machine.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  CONTROL_LOOP_STATE_SAFE_IDLE = 0,
  CONTROL_LOOP_STATE_OPEN_LOOP_READY,
  CONTROL_LOOP_STATE_ALIGN,
  CONTROL_LOOP_STATE_OPEN_LOOP_RUN,
  CONTROL_LOOP_STATE_STOPPING,
  CONTROL_LOOP_STATE_FAULT
} control_loop_state_t;

typedef enum
{
  CONTROL_LOOP_FAULT_NONE = 0,
  CONTROL_LOOP_FAULT_PWM_START,
  CONTROL_LOOP_FAULT_DRIVER_ENABLE,
  CONTROL_LOOP_FAULT_ADC,
  CONTROL_LOOP_FAULT_ENCODER,
  CONTROL_LOOP_FAULT_TIMEOUT
} control_loop_fault_t;

typedef enum
{
  CONTROL_LOOP_COMMAND_ACCEPTED = 0,
  CONTROL_LOOP_COMMAND_REJECTED_STATE,
  CONTROL_LOOP_COMMAND_REJECTED_FAULT
} control_loop_command_status_t;

void control_loop_init(void);
void control_loop_poll(void);

/* Motion can only begin through this explicit command. */
control_loop_command_status_t control_loop_request_open_loop_start(void);
void control_loop_request_stop(void);
control_loop_command_status_t control_loop_request_fault_clear(void);

control_loop_state_t control_loop_get_state(void);
control_loop_fault_t control_loop_get_fault(void);
const char *control_loop_state_text(control_loop_state_t state);
const char *control_loop_fault_text(control_loop_fault_t fault);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H */
