/*
 * FOC 单步控制器中间库。
 *
 * 所属层：面向系统的数学中间库。
 * 组合电流变换、Park、PI、反 Park 和 SVPWM，返回一次控制计算结果。
 * 本模块不访问硬件，不决定 MOTOR_EN，也不决定状态机何时调用它。
 */

#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_params.h"

typedef struct
{
  foc_parameter_set_t parameters;
  foc_pi_state_t id_pi;
  foc_pi_state_t iq_pi;
  uint8_t initialized;
} foc_controller_t;

void foc_controller_init(foc_controller_t *controller,
                         const foc_parameter_set_t *parameters);
void foc_controller_reset(foc_controller_t *controller);
foc_status_t foc_controller_step(foc_controller_t *controller,
                                 const foc_control_input_t *input,
                                 foc_control_output_t *output);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROLLER_H */
