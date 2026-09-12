#include "foc_runtime.h"

#include "board_config.h"
#include "foc_angle_math.h"
#include "foc_current_model.h"
#include "motor_drv.h"
#include "motor_pwm.h"

static foc_controller_t foc_runtime_controller;
static foc_parameter_set_t foc_runtime_parameters;
static foc_dq_t foc_runtime_target_a;
static foc_dq_t foc_runtime_voltage_target_v;
static foc_control_mode_t foc_runtime_control_mode;
static foc_control_output_t foc_runtime_last_output;
static float foc_runtime_forced_angle_rad;
static foc_runtime_state_t foc_runtime_state;
static foc_runtime_fault_t foc_runtime_fault;
static uint8_t foc_runtime_test_mode;
static foc_runtime_output_observer_fn foc_runtime_output_observer;
/* 临时诊断探针计数（ISR 写、主循环读）。 */
static volatile uint32_t foc_runtime_probe_enter;
static volatile uint32_t foc_runtime_probe_bad;
static volatile uint32_t foc_runtime_probe_faults;
static volatile foc_runtime_fault_t foc_runtime_probe_last_fault;

/* 坏拍去抖：单拍坏样本（多为开关噪声/采样毛刺）只丢弃不 FAULT，连续坏拍
 * 才判真实失效。10kHz 下 3 拍=0.3ms，热时间常数为秒级，无累积损伤风险；
 * 该窗口不放宽、不替代任何持续过流保护。 */
#define FOC_RUNTIME_BAD_SAMPLE_LIMIT (3U)
static uint8_t foc_runtime_bad_streak;

/* 拍均值累加器（FR1 抗单拍噪声）：快路径 10kHz ISR 每个成功控制步累加一次
 * measured Id/Iq，慢路径 1ms 调 consume 取均值并清零。ISR 只做加法，O(1)。 */
static volatile float foc_runtime_tick_id_sum;
static volatile float foc_runtime_tick_iq_sum;
static volatile uint32_t foc_runtime_tick_n;

static void foc_runtime_fail(foc_runtime_fault_t fault)
{
  foc_runtime_probe_faults++;
  foc_runtime_probe_last_fault = fault;
  motor_drv_disable();
  motor_pwm_force_zero();
  motor_pwm_stop_output();
  foc_controller_reset(&foc_runtime_controller);
  foc_runtime_fault = fault;
  foc_runtime_state = FOC_RUNTIME_FAULT;
}

void foc_runtime_init(void)
{
  foc_runtime_target_a.d = 0.0f;
  foc_runtime_target_a.q = 0.0f;
  foc_runtime_voltage_target_v.d = 0.0f;
  foc_runtime_voltage_target_v.q = 0.0f;
  foc_runtime_control_mode = FOC_CONTROL_CURRENT;
  foc_runtime_forced_angle_rad = 0.0f;
  foc_runtime_fault = FOC_RUNTIME_FAULT_NONE;
  foc_runtime_state = FOC_RUNTIME_STOPPED;
  foc_runtime_test_mode = 0U;
  motor_adc_set_sample_callback(foc_runtime_on_current_sample);
}

foc_status_t foc_runtime_configure(const foc_parameter_set_t *parameters)
{
  if (parameters == 0)
  {
    return FOC_STATUS_INVALID_ARG;
  }
  if ((parameters->current.calibrated == 0U) ||
      (parameters->bus_voltage_v <= 0.0f) ||
      (parameters->max_voltage_v <= 0.0f) ||
      (parameters->max_current_a <= 0.0f) ||
      (parameters->pwm_period_ticks == 0U))
  {
    return FOC_STATUS_NOT_READY;
  }
  foc_runtime_parameters = *parameters;
  foc_controller_init(&foc_runtime_controller, &foc_runtime_parameters);
  /* 每次（重）配置回到安全默认：电流模式、零给定；实际模式由后续 setter 指定。 */
  foc_runtime_control_mode = FOC_CONTROL_CURRENT;
  foc_runtime_voltage_target_v.d = 0.0f;
  foc_runtime_voltage_target_v.q = 0.0f;
  foc_runtime_target_a.d = 0.0f;
  foc_runtime_target_a.q = 0.0f;
  foc_runtime_fault = FOC_RUNTIME_FAULT_NONE;
  foc_runtime_state = FOC_RUNTIME_READY;
  return FOC_STATUS_OK;
}

void foc_runtime_set_test_mode(uint8_t enabled)
{
  foc_runtime_test_mode = (enabled != 0U) ? 1U : 0U;
}

void foc_runtime_set_output_observer(foc_runtime_output_observer_fn observer)
{
  foc_runtime_output_observer = observer;
}

void foc_runtime_reset_controller(void)
{
  foc_controller_reset(&foc_runtime_controller);
}

foc_status_t foc_runtime_start(void)
{
  if (foc_runtime_state != FOC_RUNTIME_READY)
  {
    return FOC_STATUS_NOT_READY;
  }
  if (foc_runtime_control_mode == FOC_CONTROL_CURRENT)
  {
    /* 电流闭环要求反馈方向已验证（scale+direction）；test_mode 不再无差别
     * 放行闭环，避免在符号未定时用错误方向建环造成正反馈。 */
    if (motor_adc_is_closed_loop_allowed() == 0U)
    {
      return FOC_STATUS_NOT_READY;
    }
  }
  else
  {
    /* 开环电压注入只允许在标定 test_mode 下进行（上层负责电压/时长限制）。 */
    if (foc_runtime_test_mode == 0U)
    {
      return FOC_STATUS_NOT_READY;
    }
  }
  /* PWM 由上层在使能驱动前提前启动（见 control_loop start_power）；此处幂等：
   * 已启动则跳过，避免重复 motor_pwm_start_test_output() 再次 force_zero 闪断。
   * 未启动时仍兜底启动，保持 foc_runtime_start 的独立可用性。 */
  if (motor_pwm_is_output_started() == 0U)
  {
    if (motor_pwm_start_test_output() != MOTOR_PWM_STATUS_OK)
    {
      foc_runtime_fail(FOC_RUNTIME_FAULT_PWM);
      return FOC_STATUS_NOT_READY;
    }
  }
  foc_runtime_probe_enter = 0U;
  foc_runtime_probe_bad = 0U;
  foc_runtime_probe_faults = 0U;
  foc_runtime_probe_last_fault = FOC_RUNTIME_FAULT_NONE;
  foc_runtime_bad_streak = 0U;
  foc_runtime_tick_id_sum = 0.0f;
  foc_runtime_tick_iq_sum = 0.0f;
  foc_runtime_tick_n = 0U;
  foc_runtime_state = FOC_RUNTIME_RUNNING;
  return FOC_STATUS_OK;
}

void foc_runtime_stop(void)
{
  motor_drv_disable();
  motor_pwm_force_zero();
  motor_pwm_stop_output();
  foc_controller_reset(&foc_runtime_controller);
  foc_runtime_tick_id_sum = 0.0f;
  foc_runtime_tick_iq_sum = 0.0f;
  foc_runtime_tick_n = 0U;
  if (foc_runtime_state != FOC_RUNTIME_FAULT)
  {
    foc_runtime_state = FOC_RUNTIME_STOPPED;
  }
}

void foc_runtime_set_current_target(float id_a, float iq_a)
{
  foc_runtime_target_a.d = id_a;
  foc_runtime_target_a.q = iq_a;
  foc_runtime_control_mode = FOC_CONTROL_CURRENT;
}

void foc_runtime_set_voltage_target(float vd_v, float vq_v)
{
  foc_runtime_voltage_target_v.d = vd_v;
  foc_runtime_voltage_target_v.q = vq_v;
  foc_runtime_control_mode = FOC_CONTROL_VOLTAGE;
}

foc_control_mode_t foc_runtime_get_control_mode(void)
{
  return foc_runtime_control_mode;
}

void foc_runtime_set_forced_angle(float electrical_angle_rad)
{
  foc_runtime_forced_angle_rad = foc_angle_wrap_rad(electrical_angle_rad);
}

void foc_runtime_advance_forced_angle(float electrical_angle_step_rad)
{
  foc_runtime_set_forced_angle(foc_runtime_forced_angle_rad + electrical_angle_step_rad);
}

void foc_runtime_on_current_sample(const motor_current_sample_t *sample)
{
  foc_raw_current_sample_t raw;
  foc_current_sample_t current;
  foc_control_input_t input;

  if (foc_runtime_state != FOC_RUNTIME_RUNNING)
  {
    return;
  }
  foc_runtime_probe_enter++;
  if ((sample == 0) || (sample->valid == 0U) ||
      (sample->saturated != 0U) || (sample->timeout != 0U))
  {
    /* 去抖：单拍坏样本丢弃并保持上一拍输出，连续达限才 FAULT。 */
    foc_runtime_probe_bad++;
    foc_runtime_bad_streak++;
    if (foc_runtime_bad_streak >= FOC_RUNTIME_BAD_SAMPLE_LIMIT)
    {
      foc_runtime_fail(FOC_RUNTIME_FAULT_INVALID_SAMPLE);
    }
    return;
  }
  foc_runtime_bad_streak = 0U;
  raw.channel_u_raw = sample->phase_u_raw;
  raw.channel_v_raw = sample->phase_v_raw;
  /* 测量侧同样按 phase_map 路由（与下方 PWM 输出侧对称）：
   * 通道 -> 逻辑相的重排在 foc_current_model_convert 内完成。 */
  if (foc_current_model_convert(&foc_runtime_parameters.current, &raw,
                                &foc_runtime_parameters.phase_map,
                                &current) != FOC_STATUS_OK)
  {
    foc_runtime_fail(FOC_RUNTIME_FAULT_CURRENT_MODEL);
    return;
  }

  input.phase_current_a = current.phase_current_a;
  input.electrical_angle_rad = foc_runtime_forced_angle_rad;
  input.current_target_a = foc_runtime_target_a;
  input.voltage_command_v = foc_runtime_voltage_target_v;
  input.mode = foc_runtime_control_mode;
  input.control_period_s = BOARD_CONFIG_CURRENT_LOOP_PERIOD_S;
  if (foc_controller_step(&foc_runtime_controller, &input, &foc_runtime_last_output) != FOC_STATUS_OK)
  {
    foc_runtime_fail(FOC_RUNTIME_FAULT_CONTROL);
    return;
  }
  /* 线序映射（输出侧）：把控制器 A/B/C 输出的 tick 按 phase_map 重排到硬件
   * PWM 通道（TIM2 CH1/CH2/CH3），使软件里"电角度=0 对应 A 相轴"的约定与
   * 真实绕组轴一致。phase_map 由标定序列写入 foc_runtime_parameters
   * （configure 时拷入本静态副本），快路径只读；默认恒等映射，行为与历史
   * 完全一致。【对称性契约】测量侧（上方 foc_current_model_convert）必须
   * 应用同一映射，否则电流环反馈错位（见 20260912 调试日志）。 */
  {
    uint32_t ch[3];
    ch[foc_runtime_parameters.phase_map.phase_a_output] =
        foc_runtime_last_output.pwm.phase_a_ticks;
    ch[foc_runtime_parameters.phase_map.phase_b_output] =
        foc_runtime_last_output.pwm.phase_b_ticks;
    ch[foc_runtime_parameters.phase_map.phase_c_output] =
        foc_runtime_last_output.pwm.phase_c_ticks;
    if (motor_pwm_set_raw(ch[0], ch[1], ch[2]) != MOTOR_PWM_STATUS_OK)
    {
      foc_runtime_fail(FOC_RUNTIME_FAULT_PWM);
      return;
    }
  }
  /* 成功控制步累加拍均值（FR1）：仅加法，慢路径 consume 取均值；保护/PI 用的仍是上面的瞬时值。 */
  foc_runtime_tick_id_sum += foc_runtime_last_output.measured_current_a.d;
  foc_runtime_tick_iq_sum += foc_runtime_last_output.measured_current_a.q;
  foc_runtime_tick_n++;
  /* PWM 更新成功后再喂观察钩子；未注册时仅一次判空，零额外开销。 */
  if (foc_runtime_output_observer != 0)
  {
    foc_runtime_output_observer(&foc_runtime_last_output, sample->pwm_cycle);
  }
}

foc_runtime_state_t foc_runtime_get_state(void)
{
  return foc_runtime_state;
}

foc_runtime_fault_t foc_runtime_get_fault(void)
{
  return foc_runtime_fault;
}

const foc_control_output_t *foc_runtime_get_last_output(void)
{
  return &foc_runtime_last_output;
}

uint8_t foc_runtime_consume_tick_mean(foc_dq_t *out, uint16_t *samples)
{
  uint32_t primask;
  uint32_t n;
  float id_sum;
  float iq_sum;

  if (out == 0)
  {
    return 0U;
  }
  /* 读-清零必须对 ISR 原子：保存 PRIMASK、关中断、拷出并清零、恢复原状态。 */
  primask = __get_PRIMASK();
  __disable_irq();
  n = foc_runtime_tick_n;
  id_sum = foc_runtime_tick_id_sum;
  iq_sum = foc_runtime_tick_iq_sum;
  foc_runtime_tick_n = 0U;
  foc_runtime_tick_id_sum = 0.0f;
  foc_runtime_tick_iq_sum = 0.0f;
  __set_PRIMASK(primask);

  if (samples != 0)
  {
    *samples = (uint16_t)n;
  }
  if (n == 0U)
  {
    return 0U;
  }
  out->d = id_sum / (float)n;
  out->q = iq_sum / (float)n;
  return 1U;
}

void foc_runtime_get_probe(uint32_t *enter, uint32_t *bad,
                           uint32_t *faults, foc_runtime_fault_t *last_fault)
{
  if (enter != 0) { *enter = foc_runtime_probe_enter; }
  if (bad != 0) { *bad = foc_runtime_probe_bad; }
  if (faults != 0) { *faults = foc_runtime_probe_faults; }
  if (last_fault != 0) { *last_fault = foc_runtime_probe_last_fault; }
}

const char *foc_runtime_fault_text(foc_runtime_fault_t fault)
{
  switch (fault)
  {
    case FOC_RUNTIME_FAULT_NONE:          return "NONE";
    case FOC_RUNTIME_FAULT_INVALID_SAMPLE:return "INVALID_SAMPLE";
    case FOC_RUNTIME_FAULT_CURRENT_MODEL: return "CURRENT_MODEL";
    case FOC_RUNTIME_FAULT_CONTROL:       return "CONTROL";
    case FOC_RUNTIME_FAULT_PWM:           return "PWM";
    default:                              return "UNKNOWN";
  }
}
