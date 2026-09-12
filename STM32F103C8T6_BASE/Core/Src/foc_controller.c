/*
 * FOC 单步控制器中间库实现。
 *
 * 单次计算顺序：
 * 1. 三相电流 -> alpha/beta；
 * 2. alpha/beta + 电角度 -> Id/Iq；
 * 3. Id/Iq 误差 -> Vd/Vq PI 输出；
 * 4. Vd/Vq -> Valpha/Vbeta；
 * 5. Valpha/Vbeta -> 三相 PWM compare。
 *
 * 上层系统逻辑必须在调用前完成采样有效性、角度有效性、使能和故障检查。
 */

#include "foc_controller.h"
#include "foc_pi.h"
#include "foc_svpwm.h"
#include "foc_transform.h"
#include <math.h>

static float foc_controller_clamp(float value, float limit)
{
  if (limit <= 0.0f)
  {
    return 0.0f;
  }
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static float foc_controller_limit_target(float target, float limit)
{
  if (limit <= 0.0f)
  {
    return 0.0f;
  }
  if (target > limit) return limit;
  if (target < -limit) return -limit;
  return target;
}

static void foc_controller_limit_voltage_vector(foc_dq_t *voltage, float limit)
{
  float square_sum;
  float magnitude;
  float scale;

  if ((voltage == 0) || (limit <= 0.0f))
  {
    return;
  }
  square_sum = (voltage->d * voltage->d) + (voltage->q * voltage->q);
  /* 计算重构（惰性求值）：先用平方比较，绝大多数拍不超限，直接返回，
   * 只有真正超限时才做一次 sqrtf（无 FPU 时约 200 cycles）。限幅语义不变。 */
  if (square_sum > (limit * limit))
  {
    magnitude = sqrtf(square_sum);
    scale = limit / magnitude;
    voltage->d *= scale;
    voltage->q *= scale;
  }
}

void foc_controller_init(foc_controller_t *controller,
                         const foc_parameter_set_t *parameters)
{
  if ((controller == 0) || (parameters == 0))
  {
    return;
  }

  controller->parameters = *parameters;
  foc_pi_init(&controller->id_pi, &parameters->id_pi);
  foc_pi_init(&controller->iq_pi, &parameters->iq_pi);
  controller->initialized = 1U;
}

void foc_controller_reset(foc_controller_t *controller)
{
  if (controller == 0)
  {
    return;
  }

  foc_pi_reset(&controller->id_pi);
  foc_pi_reset(&controller->iq_pi);
}

foc_status_t foc_controller_step(foc_controller_t *controller,
                                 const foc_control_input_t *input,
                                 foc_control_output_t *output)
{
  foc_alpha_beta_t current_alpha_beta;
  foc_alpha_beta_t voltage_alpha_beta;
  foc_dq_t measured_current;
  foc_dq_t voltage_command;
  foc_status_t svpwm_status;
  float trig_sine;
  float trig_cosine;

  if ((controller == 0) || (input == 0) || (output == 0))
  {
    return FOC_STATUS_INVALID_ARG;
  }
  output->status = FOC_STATUS_INVALID_ARG;
  if ((controller->initialized == 0U) ||
      (input->control_period_s <= 0.0f) ||
      (controller->parameters.bus_voltage_v <= 0.0f) ||
      (controller->parameters.max_voltage_v <= 0.0f) ||
      (controller->parameters.max_current_a <= 0.0f) ||
      (controller->parameters.pwm_period_ticks == 0U) ||
      (controller->parameters.inv_bus_voltage_v <= 0.0f) ||
      (controller->parameters.pwm_period_f <= 0.0f) ||
      (controller->parameters.current.code_to_amp <= 0.0f))
  {
    output->status = FOC_STATUS_NOT_READY;
    return output->status;
  }

  foc_clarke_transform(&input->phase_current_a, &current_alpha_beta);
  /* Park 与反 Park 同一电角度，sin/cos 只算一次，降低 10kHz 快路径负载。 */
  foc_sin_cos(input->electrical_angle_rad, &trig_sine, &trig_cosine);
  foc_park_transform_sc(&current_alpha_beta, trig_sine, trig_cosine,
                        &measured_current);

  if (input->mode == FOC_CONTROL_VOLTAGE)
  {
    /* 开环电压注入：不经过 PI，直接采用给定 Vd/Vq（S1a 定符号/相序用）。
     * 仍做分量与矢量限幅，保证 max_voltage 安全上限在开环下也不失守；
     * 持续清空 PI 积分，使切回 CURRENT 时无积分残留。 */
    voltage_command.d = foc_controller_clamp(input->voltage_command_v.d,
                                             controller->parameters.max_voltage_v);
    voltage_command.q = foc_controller_clamp(input->voltage_command_v.q,
                                             controller->parameters.max_voltage_v);
    foc_pi_reset(&controller->id_pi);
    foc_pi_reset(&controller->iq_pi);
  }
  else
  {
    voltage_command.d = foc_pi_update(&controller->id_pi,
                                      foc_controller_limit_target(input->current_target_a.d,
                                                                 controller->parameters.max_current_a) -
                                      measured_current.d);
    voltage_command.q = foc_pi_update(&controller->iq_pi,
                                      foc_controller_limit_target(input->current_target_a.q,
                                                                 controller->parameters.max_current_a) -
                                      measured_current.q);
    voltage_command.d = foc_controller_clamp(voltage_command.d,
                                             controller->parameters.max_voltage_v);
    voltage_command.q = foc_controller_clamp(voltage_command.q,
                                             controller->parameters.max_voltage_v);
  }
  foc_controller_limit_voltage_vector(&voltage_command,
                                      controller->parameters.max_voltage_v);

  foc_inverse_park_transform_sc(&voltage_command, trig_sine, trig_cosine,
                                &voltage_alpha_beta);
  svpwm_status = foc_svpwm_calculate(&voltage_alpha_beta,
                                     controller->parameters.inv_bus_voltage_v,
                                     controller->parameters.pwm_period_f,
                                     &output->pwm);

  output->measured_current_a = measured_current;
  output->phase_current_a = input->phase_current_a;
  output->voltage_command_v = voltage_command;
  output->voltage_alpha_beta_v = voltage_alpha_beta;
  output->status = svpwm_status;
  return output->status;
}
