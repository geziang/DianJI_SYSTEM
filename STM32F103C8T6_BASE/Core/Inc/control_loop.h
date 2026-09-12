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

/* FR-2.4 注入 API 需要参数包类型（纯类型定义，零 HAL）。 */
#include "motor_parameter_store.h"

typedef enum
{
  CONTROL_LOOP_STATE_SAFE_IDLE = 0,
  CONTROL_LOOP_STATE_TEST_BOOT,
  CONTROL_LOOP_STATE_TEST_READY,
  CONTROL_LOOP_STATE_POWER_ARMED,
  CONTROL_LOOP_STATE_TEST_ALIGN,
  CONTROL_LOOP_STATE_SIGN_CALIBRATION,
  CONTROL_LOOP_STATE_R_IDENTIFICATION,
  CONTROL_LOOP_STATE_L_IDENTIFICATION,
  CONTROL_LOOP_STATE_TEST_CURRENT_VALIDATE,
  CONTROL_LOOP_STATE_PHASE_SEQ_CALIBRATION,
  CONTROL_LOOP_STATE_ENCODER_DIR_CALIBRATION,
  CONTROL_LOOP_STATE_SENSOR_CALIBRATION,
  CONTROL_LOOP_STATE_LOW_SPEED_CAPTURE,
  CONTROL_LOOP_STATE_CANDIDATE_REVIEW,
  CONTROL_LOOP_STATE_OPEN_LOOP_READY,
  CONTROL_LOOP_STATE_ALIGN,
  CONTROL_LOOP_STATE_OPEN_LOOP_RUN,
  CONTROL_LOOP_STATE_IDENT_PRECHECK,
  CONTROL_LOOP_STATE_FORCED_CURRENT,
  CONTROL_LOOP_STATE_PI_VALIDATE,
  CONTROL_LOOP_STATE_LIMITED_FOC_READY,
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
  CONTROL_LOOP_FAULT_TIMEOUT,
  CONTROL_LOOP_FAULT_CURRENT_LOOP,
  CONTROL_LOOP_FAULT_SIGN_MISMATCH,
  CONTROL_LOOP_FAULT_CHANNEL_MISMATCH,
  CONTROL_LOOP_FAULT_NO_CONVERGE,
  CONTROL_LOOP_FAULT_STEP_VALIDATE
} control_loop_fault_t;

typedef enum
{
  CONTROL_LOOP_COMMAND_ACCEPTED = 0,
  CONTROL_LOOP_COMMAND_REJECTED_STATE,
  CONTROL_LOOP_COMMAND_REJECTED_FAULT,
  CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY,
  /* SPEC-RUN FR-2.5：c=clear 擦参数页失败等存储侧拒绝。 */
  CONTROL_LOOP_COMMAND_REJECTED_STORAGE
} control_loop_command_status_t;

void control_loop_init(void);
void control_loop_poll(void);

/* Legacy open-loop entry; test identification uses p/r/a/d/x below. */
control_loop_command_status_t control_loop_request_open_loop_start(void);
control_loop_command_status_t control_loop_request_identification_start(void);
control_loop_command_status_t control_loop_request_power_confirm(void);
control_loop_command_status_t control_loop_request_candidate_apply(void);
control_loop_command_status_t control_loop_request_candidate_discard(void);
void control_loop_request_stop(void);
control_loop_command_status_t control_loop_request_fault_clear(void);
/* c=clear（非 FAULT 态）：失效 RAM 学习值/活动包并擦除两个参数页，
 * 要求功率已撤（SAFE_IDLE 等静默态）；清除后等同未标定（FR-2.5）。 */
control_loop_command_status_t control_loop_request_clear_storage(void);
/* 上电参数注入（FR-2.4）：safety_manager PARAMETER_CHECK 加载成功后调用，
 * 把 R/L/PI/符号/方向/offset/换相灌入控制参数集、重建 R 学习状态，
 * 并打印参数来源（页/revision/关键值）。运行中调用无效。 */
void control_loop_load_committed_parameters(const motor_parameter_package_t *package);

control_loop_state_t control_loop_get_state(void);
control_loop_fault_t control_loop_get_fault(void);
const char *control_loop_state_text(control_loop_state_t state);
const char *control_loop_fault_text(control_loop_fault_t fault);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H */
