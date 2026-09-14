/*
 * 运行态最小应用骨架（app_run.h 见模块定位）。
 *
 * 安全基线：上电默认功率失能；使能仅走 READY→p 准入门（FR-3.2 骨架：
 * 有效固化包 + ADC 零偏 OK + 编码器在线 + 无锁存故障）；任何失败先
 * MOTOR_EN 拉低再 PWM 归零（对齐 FR-3.4 顺序）；故障不自动重启，
 * c=clear 后重走 BOOT 准入评估。
 */

#include "app_run.h"

#include "baseline_diag.h"
#include "board_config.h"
#include "debug_log.h"
#include "encoder_cache.h"
#include "foc_angle_math.h"
#include "foc_params.h"
#include "foc_pi.h"
#include "foc_precompute.h"
#include "foc_rotor_model.h"
#include "foc_runtime.h"
#include "motor_adc.h"
#include "motor_drv.h"
#include "motor_pwm.h"
#include "safety_manager.h"
#include "speed_estimate.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

/* 运行态限值（与测试态 TEST_* 同源同值：10V 红线/电压饱和裕量/1A 硬限）。 */
#define RUN_BUS_VOLTAGE_V   (10.0f)
#define RUN_MAX_VOLTAGE_V   (5.5f)
#define RUN_MAX_CURRENT_A   (1.0f)

#define RUN_TLM_PERIOD_MS   (500U)  /* 使能态遥测周期（R3 正式设计再定频） */
#define RUN_ENC_VALID_MS    (20U)   /* 编码器新鲜度门（同 PSEQ 判决口径） */
#define RUN_IQ_LIMIT_A      (0.5f)  /* iq 指令限幅：守工程窗口 0.2-0.6A 上沿 */

/* ---- 速度环首版（SPEC-RUN FR-4.2/4.3/4.4；FR-4.5 阈值待整定回填） ---- */
#define RUN_SPEED_LIMIT_RPM    (300.0f) /* 目标限幅：避开 400rpm 电压饱和与编码器高速掉线区 */
#define RUN_SPEED_SLEW_RPM_S   (500.0f) /* 目标斜坡（FR-4.3 加速度限首版值） */
#define RUN_OVERSPEED_RPM      (450.0f) /* 超速保护（FR-4.4） */
#define RUN_SPEED_KP_INIT      (0.005f) /* A/rpm：按 ~2000rpm/A 开环增益与 0.5A 饱和估，待整定 */
#define RUN_SPEED_KI_INIT      (0.025f) /* A/(rpm·s)，待整定 */
#define RUN_SPEED_STEP_RPM     (20.0f)
#define RUN_SPEED_TICK_S       (0.001f) /* 速度外环拍周期=主循环 1ms */
#define RUN_RPS_TO_RPM         (9.5493f)
#define RUN_GAIN_RATIO         (1.25f)  /* 1..4 键增益调节倍率 */

typedef enum
{
  APP_RUN_MODE_IQ = 0,    /* 手动 iq 阶梯（诊断用，FOC-4 已验证） */
  APP_RUN_MODE_SPEED      /* 速度闭环 */
} app_run_mode_t;

static app_run_state_t app_run_state = APP_RUN_STATE_BOOT;
static foc_parameter_set_t app_run_parameters;
static motor_parameter_package_t app_run_package;
static uint8_t app_run_has_package = 0U;
static uint32_t app_run_tlm_ms = 0U;
static float app_run_iq_target_a = 0.0f;
static app_run_mode_t app_run_mode = APP_RUN_MODE_IQ;
static speed_estimator_t app_run_speed_est;
static foc_pi_state_t app_run_speed_pi;
static float app_run_speed_kp = RUN_SPEED_KP_INIT;
static float app_run_speed_ki = RUN_SPEED_KI_INIT;
static float app_run_speed_target_rpm = 0.0f;
static float app_run_speed_cmd_rpm = 0.0f;    /* 斜坡后的实际指令 */
static float app_run_speed_iq_ref = 0.0f;     /* 速度 PI 输出 */
static uint32_t app_run_stall_ms = 0U;
static uint32_t app_run_wrong_dir_ms = 0U;
static uint8_t app_run_zero_retries = 0U;   /* BOOT 零偏自愈重试计数 */
static uint32_t app_run_enc_revive_ms = 0U; /* BOOT 编码器无效起点/复活节流 */
static uint8_t app_run_enc_revives = 0U;    /* I2C 总线复活尝试计数 */
static uint32_t app_run_hint_ms = 0U;

static void app_run_speed_pi_configure(void);

/* 格式化日志（debug_log_write_line 只收成品串，同 baseline_diag_logf 做法）。 */
static void app_run_logf(const char *fmt, ...)
{
  char line[160];
  int written;
  va_list args;

  va_start(args, fmt);
  written = vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  if (written > 0)
  {
    debug_log_write_line(line);
  }
}

static void app_run_safe_disable(void)
{
  motor_drv_disable();
  motor_pwm_force_zero();
  motor_pwm_stop_output();
}

static void app_run_set_state(app_run_state_t next)
{
  if (app_run_state != next)
  {
    app_run_state = next;
    app_run_logf("[HBT] state=RUN_%s", app_run_state_text(next));
  }
}

const char *app_run_state_text(app_run_state_t state)
{
  switch (state)
  {
    case APP_RUN_STATE_BOOT: return "BOOT";
    case APP_RUN_STATE_NEED_CAL: return "NEED_CAL";
    case APP_RUN_STATE_READY: return "READY";
    case APP_RUN_STATE_ENABLED: return "ENABLED";
    case APP_RUN_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

app_run_state_t app_run_get_state(void)
{
  return app_run_state;
}

void app_run_load_package(const motor_parameter_package_t *package)
{
  char line[160];

  if (package == (const motor_parameter_package_t *)0)
  {
    return;
  }
  app_run_package = *package;
  app_run_has_package = 1U;

  /* 只存包副本；参数集构建在 BOOT→READY 准入点做——本函数运行于
   * PARAMETER_CHECK 阶段，ADC_ZERO_REFRESH 尚未执行，此刻读零偏是
   * 复位垃圾值（2026-09-13 上板实测：错误零偏→幻觉电流→电机带载鸣叫）。
   * PI 直取包内标定值（ki 由固化时的生效 R 驱动，含 learned_R 链）。 */
  (void)snprintf(line, sizeof(line),
                 "[LOAD] run package (rev=%lu): R=%.3fohm L=%.2fuH offset=%+.4frad "
                 "dir=%+d map=%u,%u,%u learned_R=%s",
                 (unsigned long)app_run_package.parameter_revision,
                 (double)app_run_package.phase_resistance_ohm,
                 (double)(app_run_package.phase_inductance_h * 1e6f),
                 (double)app_run_package.electrical_offset_rad,
                 (int)app_run_package.encoder_direction,
                 (unsigned)app_run_package.phase_map_a,
                 (unsigned)app_run_package.phase_map_b,
                 (unsigned)app_run_package.phase_map_c,
                 (app_run_package.learned_r_valid != 0U) ? "yes" : "no");
  debug_log_write_line(line);
}

/* 参数集构建：只在零偏已新鲜（准入点）时执行——零偏取当拍实测，
 * 其余标量/PI/换相取包（map 按 DD-01 契约从包直取）。 */
static uint8_t app_run_build_parameters(void)
{
  motor_adc_current_diagnostic_t diagnostic;

  motor_adc_get_current_diagnostic(&diagnostic);
  foc_parameter_set_load_default(&app_run_parameters);
  app_run_parameters.current.channel_u_zero_raw =
      (float)diagnostic.zero.phase_u_zero_raw;
  app_run_parameters.current.channel_v_zero_raw =
      (float)diagnostic.zero.phase_v_zero_raw;
  app_run_parameters.current.calibrated = 1U;
  app_run_parameters.current.channel_u_sign =
      (float)app_run_package.current_sign_u;
  app_run_parameters.current.channel_v_sign =
      (float)app_run_package.current_sign_v;
  app_run_parameters.bus_voltage_v = RUN_BUS_VOLTAGE_V;
  app_run_parameters.max_voltage_v = RUN_MAX_VOLTAGE_V;
  app_run_parameters.max_current_a = RUN_MAX_CURRENT_A;
  app_run_parameters.pwm_period_ticks = BOARD_CONFIG_PWM_PERIOD_TICKS;
  app_run_parameters.rotor.pole_pairs = app_run_package.pole_pairs;
  app_run_parameters.rotor.encoder_direction = app_run_package.encoder_direction;
  app_run_parameters.rotor.electrical_offset_rad =
      app_run_package.electrical_offset_rad;
  app_run_parameters.id_pi.kp = app_run_package.id_kp;
  app_run_parameters.id_pi.ki = app_run_package.id_ki;
  app_run_parameters.iq_pi.kp = app_run_package.iq_kp;
  app_run_parameters.iq_pi.ki = app_run_package.iq_ki;
  app_run_parameters.phase_map.phase_a_output = app_run_package.phase_map_a;
  app_run_parameters.phase_map.phase_b_output = app_run_package.phase_map_b;
  app_run_parameters.phase_map.phase_c_output = app_run_package.phase_map_c;

  return (uint8_t)(foc_parameter_set_precompute(&app_run_parameters,
                                                BOARD_CONFIG_CURRENT_LOOP_PERIOD_S) ==
                   FOC_STATUS_OK);
}

/* 使能：FR-3.2 准入门 + start_power 同序（cfg→sync→角度→目标→pwm→drv→rt）。 */
static uint8_t app_run_enable(void)
{
  motor_adc_current_diagnostic_t diagnostic;
  foc_rotor_sample_t rotor;
  const encoder_cache_sample_t *enc;

  if (app_run_has_package == 0U)
  {
    debug_log_write_line("[CMD] enable rejected: no parameter package");
    return 0U;
  }
  motor_adc_get_current_diagnostic(&diagnostic);
  if (diagnostic.zero.quality != MOTOR_ADC_ZERO_QUALITY_OK)
  {
    debug_log_write_line("[CMD] enable rejected: ADC zero not OK");
    return 0U;
  }
  if (encoder_cache_is_valid(RUN_ENC_VALID_MS) == 0U)
  {
    debug_log_write_line("[CMD] enable rejected: encoder not online");
    return 0U;
  }

  if (foc_runtime_configure(&app_run_parameters) != FOC_STATUS_OK)
  {
    debug_log_write_line("[FAULT] run cfg failed");
    app_run_safe_disable();
    app_run_set_state(APP_RUN_STATE_FAULT);
    return 0U;
  }
  if (motor_adc_start_synchronized() != MOTOR_ADC_STATUS_OK)
  {
    debug_log_write_line("[FAULT] run adc sync failed");
    app_run_safe_disable();
    app_run_set_state(APP_RUN_STATE_FAULT);
    return 0U;
  }
  enc = encoder_cache_get_latest();
  (void)foc_rotor_model_convert(&app_run_parameters.rotor, enc->raw_angle, &rotor);
  foc_runtime_set_forced_angle(rotor.electrical_angle_rad);
  foc_runtime_set_current_target(0.0f, 0.0f); /* 零流目标：FOC-4 第一步 */
  if (motor_pwm_start_test_output() != MOTOR_PWM_STATUS_OK)
  {
    debug_log_write_line("[FAULT] run pwm start failed");
    app_run_safe_disable();
    app_run_set_state(APP_RUN_STATE_FAULT);
    return 0U;
  }
  if (motor_drv_enable_for_test() != MOTOR_DRV_STATUS_OK)
  {
    debug_log_write_line("[FAULT] run drv enable failed");
    app_run_safe_disable();
    app_run_set_state(APP_RUN_STATE_FAULT);
    return 0U;
  }
  if (foc_runtime_start() != FOC_STATUS_OK)
  {
    /* 闭环门禁三标志自证：拒绝时直接暴露 closed_loop/scale/dir 状态。 */
    motor_adc_current_diagnostic_t gate;
    motor_adc_get_current_diagnostic(&gate);
    app_run_logf("[FAULT] run runtime start rejected: closed_loop=%u scale=%u dir=%u",
                 (unsigned)gate.closed_loop_allowed,
                 (unsigned)gate.scale_verified,
                 (unsigned)gate.direction_verified);
    app_run_safe_disable();
    app_run_set_state(APP_RUN_STATE_FAULT);
    return 0U;
  }
  app_run_iq_target_a = 0.0f;
  speed_estimator_init(&app_run_speed_est);
  app_run_speed_target_rpm = 0.0f;
  app_run_speed_cmd_rpm = 0.0f;
  app_run_speed_iq_ref = 0.0f;
  app_run_stall_ms = 0U;
  app_run_wrong_dir_ms = 0U;
  app_run_speed_pi_configure();
  app_run_tlm_ms = HAL_GetTick();
  debug_log_write_line("[RUN] enabled: zero-current hold (id=0 iq=0)");
  app_run_set_state(APP_RUN_STATE_ENABLED);
  return 1U;
}

/* 速度 PI 装载：在线改增益必须整装（ki_times_t 重预算 + 积分清零，
 * 后者本就是增益调整的标准做法）。 */
static void app_run_speed_pi_configure(void)
{
  foc_pi_config_t cfg;

  cfg.kp = app_run_speed_kp;
  cfg.ki = app_run_speed_ki;
  cfg.ki_times_t = app_run_speed_ki * RUN_SPEED_TICK_S;
  cfg.integrator_min = -RUN_IQ_LIMIT_A;
  cfg.integrator_max = RUN_IQ_LIMIT_A;
  cfg.output_min = -RUN_IQ_LIMIT_A;
  cfg.output_max = RUN_IQ_LIMIT_A;
  foc_pi_init(&app_run_speed_pi, &cfg);
}

static void app_run_speed_step(float delta_rpm)
{
  if (app_run_state != APP_RUN_STATE_ENABLED)
  {
    debug_log_write_line("[CMD] speed command only while ENABLED");
    return;
  }
  app_run_speed_target_rpm += delta_rpm;
  if (app_run_speed_target_rpm > RUN_SPEED_LIMIT_RPM)
  {
    app_run_speed_target_rpm = RUN_SPEED_LIMIT_RPM;
  }
  if (app_run_speed_target_rpm < -RUN_SPEED_LIMIT_RPM)
  {
    app_run_speed_target_rpm = -RUN_SPEED_LIMIT_RPM;
  }
  app_run_logf("[CMD] n*=%+.0frpm (slew %.0frpm/s)",
               (double)app_run_speed_target_rpm,
               (double)RUN_SPEED_SLEW_RPM_S);
}

/* which: 0=kp 1=ki；增益在线调节（1..4 键），改后整装 PI。 */
static void app_run_gain_step(uint8_t which, uint8_t up)
{
  float ratio = (up != 0U) ? RUN_GAIN_RATIO : (1.0f / RUN_GAIN_RATIO);

  if (which == 0U)
  {
    app_run_speed_kp *= ratio;
    if (app_run_speed_kp < 1e-4f) { app_run_speed_kp = 1e-4f; }
    if (app_run_speed_kp > 1.0f) { app_run_speed_kp = 1.0f; }
  }
  else
  {
    app_run_speed_ki *= ratio;
    if (app_run_speed_ki < 1e-4f) { app_run_speed_ki = 1e-4f; }
    if (app_run_speed_ki > 10.0f) { app_run_speed_ki = 10.0f; }
  }
  app_run_speed_pi_configure();
  app_run_logf("[CMD] speed PI kp=%.4f ki=%.3f",
               (double)app_run_speed_kp, (double)app_run_speed_ki);
}

/* FOC-4 第 2-5 步：iq 步进指令（工程窗口内大激励，id 恒 0）。 */
static void app_run_iq_step(float delta_a)
{
  if (app_run_state != APP_RUN_STATE_ENABLED)
  {
    debug_log_write_line("[CMD] current command only while ENABLED");
    return;
  }
  app_run_iq_target_a += delta_a;
  if (app_run_iq_target_a > RUN_IQ_LIMIT_A)
  {
    app_run_iq_target_a = RUN_IQ_LIMIT_A;
  }
  if (app_run_iq_target_a < -RUN_IQ_LIMIT_A)
  {
    app_run_iq_target_a = -RUN_IQ_LIMIT_A;
  }
  foc_runtime_set_current_target(0.0f, app_run_iq_target_a);
  app_run_logf("[CMD] target id=0.00A iq=%+.2fA", (double)app_run_iq_target_a);
}

static void app_run_handle_command(uint8_t command)
{
  switch (command)
  {
    case 'p':
    case 'P':
      if (app_run_state == APP_RUN_STATE_READY)
      {
        (void)app_run_enable();
      }
      else
      {
        app_run_logf("[CMD] enable rejected: state=%s (need READY)",
                     app_run_state_text(app_run_state));
      }
      break;
    case 'x':
    case 'X':
      if (app_run_state == APP_RUN_STATE_ENABLED)
      {
        app_run_safe_disable();
        debug_log_write_line("[RUN] disabled");
        app_run_set_state(APP_RUN_STATE_READY);
      }
      else if (app_run_state == APP_RUN_STATE_FAULT)
      {
        debug_log_write_line("[CMD] use c to clear fault");
      }
      else
      {
        debug_log_write_line("[CMD] not enabled");
      }
      break;
    case 'c':
    case 'C':
      if (app_run_state == APP_RUN_STATE_FAULT)
      {
        (void)safety_manager_request_fault_clear();
        debug_log_write_line("[CMD] fault cleared; re-evaluating admission");
        app_run_set_state(APP_RUN_STATE_BOOT); /* 重走准入（FR-3.4） */
      }
      else
      {
        debug_log_write_line("[CMD] nothing to clear");
      }
      break;
    case 'i':
    case 'I':
    case '?':
      baseline_diag_run_verbose_pipeline();
      break;
    case '+':
    case '=':
      if (app_run_mode == APP_RUN_MODE_SPEED)
      {
        app_run_speed_step(RUN_SPEED_STEP_RPM);
      }
      else
      {
        app_run_iq_step(0.05f);
      }
      break;
    case '-':
    case '_':
      if (app_run_mode == APP_RUN_MODE_SPEED)
      {
        app_run_speed_step(-RUN_SPEED_STEP_RPM);
      }
      else
      {
        app_run_iq_step(-0.05f);
      }
      break;
    case '0':
      if (app_run_state != APP_RUN_STATE_ENABLED)
      {
        debug_log_write_line("[CMD] current command only while ENABLED");
      }
      else if (app_run_mode == APP_RUN_MODE_SPEED)
      {
        app_run_speed_target_rpm = 0.0f;
        debug_log_write_line("[CMD] n*=+0rpm");
      }
      else
      {
        app_run_iq_target_a = 0.0f;
        foc_runtime_set_current_target(0.0f, 0.0f);
        debug_log_write_line("[CMD] target id=0.00A iq=+0.00A");
      }
      break;
    case 's':
    case 'S':
      if (app_run_mode == APP_RUN_MODE_SPEED)
      {
        app_run_mode = APP_RUN_MODE_IQ;
        app_run_iq_target_a = 0.0f;
        if (app_run_state == APP_RUN_STATE_ENABLED)
        {
          foc_runtime_set_current_target(0.0f, 0.0f);
        }
        debug_log_write_line("[CMD] mode=IQ (+/- step 0.05A)");
      }
      else
      {
        app_run_mode = APP_RUN_MODE_SPEED;
        app_run_speed_target_rpm = 0.0f;
        app_run_speed_cmd_rpm = 0.0f;
        app_run_speed_iq_ref = 0.0f;
        app_run_speed_pi_configure();
        if (app_run_state == APP_RUN_STATE_ENABLED)
        {
          foc_runtime_set_current_target(0.0f, 0.0f);
        }
        debug_log_write_line("[CMD] mode=SPEED (+/- step 20rpm, ramp to target)");
      }
      break;
    case '1':
      app_run_gain_step(0U, 1U);
      break;
    case '2':
      app_run_gain_step(0U, 0U);
      break;
    case '3':
      app_run_gain_step(1U, 1U);
      break;
    case '4':
      app_run_gain_step(1U, 0U);
      break;
    default:
      break;
  }
}

static void app_run_tick(uint32_t now_ms)
{
  motor_adc_current_diagnostic_t diagnostic;
  foc_rotor_sample_t rotor;
  const encoder_cache_sample_t *enc;
  const foc_control_output_t *output;
  float mech_rad;
  float mech_rpm;

  switch (app_run_state)
  {
    case APP_RUN_STATE_BOOT:
      /* 准入评估（FR-3.2 骨架）：包 + 零偏 + 编码器。等待必须可见、
       * 可自愈、有上限（2026-09-13 上板教训：无声等待=只能靠猜）。 */
      if (app_run_has_package == 0U)
      {
        if (safety_manager_get_state() > SAFETY_MANAGER_STATE_PARAMETER_CHECK)
        {
          app_run_set_state(APP_RUN_STATE_NEED_CAL);
        }
        break;
      }
      motor_adc_get_current_diagnostic(&diagnostic);
      if (diagnostic.zero.quality != MOTOR_ADC_ZERO_QUALITY_OK)
      {
        /* 零偏自愈：安全链刷新已过而质量不 OK -> 每 2s 重校（功率关闭态，
         * 安全无副作用），10 次不过转 FAULT。 */
        if ((safety_manager_get_state() > SAFETY_MANAGER_STATE_ADC_ZERO_REFRESH) &&
            ((uint32_t)(now_ms - app_run_hint_ms) >= 2000U))
        {
          app_run_hint_ms = now_ms;
          (void)motor_adc_calibrate_zero_current();
          app_run_zero_retries++;
          app_run_logf("[RUN] waiting: zero quality not OK, retry %u/10",
                       (unsigned)app_run_zero_retries);
          if (app_run_zero_retries >= 10U)
          {
            debug_log_write_line("[FAULT] zero calibration failed after 10 retries; check ADC/supply noise");
            app_run_set_state(APP_RUN_STATE_FAULT);
          }
        }
        break;
      }
      if (encoder_cache_is_valid(RUN_ENC_VALID_MS) == 0U)
      {
        /* 编码器自愈：无效满 2s 尝试 I2C 总线复活（DeInit/Init），
         * 3 次不过转 FAULT（从机死锁拉 SDA 只能断电，明确提示）。 */
        if (app_run_enc_revive_ms == 0U)
        {
          app_run_enc_revive_ms = now_ms;
          debug_log_write_line("[RUN] waiting: encoder invalid");
        }
        else if ((uint32_t)(now_ms - app_run_enc_revive_ms) >= 2000U)
        {
          app_run_enc_revive_ms = now_ms;
          app_run_enc_revives++;
          encoder_cache_restart_bus();
          app_run_logf("[RUN] waiting: encoder invalid, I2C revive %u/3",
                       (unsigned)app_run_enc_revives);
          if (app_run_enc_revives >= 3U)
          {
            debug_log_write_line("[FAULT] encoder bus dead; full power cycle required");
            app_run_set_state(APP_RUN_STATE_FAULT);
          }
        }
        break;
      }
      app_run_enc_revive_ms = 0U;
      app_run_enc_revives = 0U;
      /* 零偏刷新已结束（quality 刚确认 OK）：此刻构建参数集（零偏取当拍
       * 实测）并置 direction（包的存在=该板 S1a 已绑定符号）——两个标志
       * 都不会被后续流程洗掉。 */
      motor_adc_set_direction_verified(1U);
      if (app_run_build_parameters() == 0U)
      {
        debug_log_write_line("[FAULT] run parameter precompute failed");
        app_run_set_state(APP_RUN_STATE_FAULT);
        break;
      }
      app_run_set_state(APP_RUN_STATE_READY);
      debug_log_write_line("[READY] p=enable (zero-current) x=stop i=diag");
      break;

    case APP_RUN_STATE_NEED_CAL:
      if ((uint32_t)(now_ms - app_run_hint_ms) >= 5000U)
      {
        app_run_hint_ms = now_ms;
        debug_log_write_line(
            "[RUN] no parameter package; flash identification firmware to calibrate");
      }
      break;

    case APP_RUN_STATE_ENABLED:
      if (encoder_cache_is_valid(RUN_ENC_VALID_MS) == 0U)
      {
        app_run_safe_disable();
        debug_log_write_line("[FAULT] encoder lost while enabled");
        app_run_set_state(APP_RUN_STATE_FAULT);
        break;
      }
      enc = encoder_cache_get_latest();
      (void)foc_rotor_model_convert(&app_run_parameters.rotor, enc->raw_angle, &rotor);
      /* 骨架限界：1ms poll 喂 forced angle（同 S7 口径，~6°@130rad/s），
       * R3 正式实现改 ISR 内编码器连续出角。 */
      foc_runtime_set_forced_angle(rotor.electrical_angle_rad);

      /* 速度估计（FR-4.1，16ms 滑窗回绕差分） */
      mech_rad = foc_mechanical_raw_to_rad(enc->raw_angle);
      speed_estimator_update(&app_run_speed_est, mech_rad, RUN_SPEED_TICK_S);
      mech_rpm = speed_estimator_get_rad_s(&app_run_speed_est, RUN_SPEED_TICK_S) *
                 RUN_RPS_TO_RPM;

      if (app_run_mode == APP_RUN_MODE_SPEED)
      {
        /* 目标斜坡（FR-4.3）：阶跃经 500rpm/s 爬坡，禁止直接反打。 */
        float step_rpm = RUN_SPEED_SLEW_RPM_S * RUN_SPEED_TICK_S;
        uint8_t est_ok = speed_estimator_valid(&app_run_speed_est);

        /* 超速保护（FR-4.4）——速度模式专属：IQ 手动模式顶空载电压天花板
         * （~455rpm@小电流）是正常物理，不该被拦（2026-09-13 上板教训）。 */
        if ((est_ok != 0U) && (fabsf(mech_rpm) > RUN_OVERSPEED_RPM))
        {
          app_run_safe_disable();
          app_run_logf("[FAULT] overspeed %.0frpm (limit %.0f)",
                       (double)mech_rpm, (double)RUN_OVERSPEED_RPM);
          app_run_set_state(APP_RUN_STATE_FAULT);
          break;
        }

        if (app_run_speed_cmd_rpm < (app_run_speed_target_rpm - step_rpm))
        {
          app_run_speed_cmd_rpm += step_rpm;
        }
        else if (app_run_speed_cmd_rpm > (app_run_speed_target_rpm + step_rpm))
        {
          app_run_speed_cmd_rpm -= step_rpm;
        }
        else
        {
          app_run_speed_cmd_rpm = app_run_speed_target_rpm;
        }

        if (est_ok != 0U)
        {
          app_run_speed_iq_ref =
              foc_pi_update(&app_run_speed_pi, app_run_speed_cmd_rpm - mech_rpm);
          foc_runtime_set_current_target(0.0f, app_run_speed_iq_ref);
        }

        /* 失速保护（FR-4.4）：iq 顶限 1s 不动而目标在动。 */
        if ((est_ok != 0U) && (fabsf(app_run_speed_iq_ref) >= 0.45f) &&
            (fabsf(mech_rpm) < 10.0f) &&
            (fabsf(app_run_speed_cmd_rpm) > 50.0f))
        {
          app_run_stall_ms += 1U;
        }
        else
        {
          app_run_stall_ms = 0U;
        }
        if (app_run_stall_ms >= 1000U)
        {
          app_run_safe_disable();
          debug_log_write_line("[FAULT] speed stall: iq at limit, no motion");
          app_run_set_state(APP_RUN_STATE_FAULT);
          break;
        }

        /* 方向矛盾保护（FR-4.4）。 */
        if ((est_ok != 0U) && (fabsf(app_run_speed_cmd_rpm) > 50.0f) &&
            (fabsf(mech_rpm) > 30.0f) &&
            ((mech_rpm * app_run_speed_cmd_rpm) < 0.0f))
        {
          app_run_wrong_dir_ms += 1U;
        }
        else
        {
          app_run_wrong_dir_ms = 0U;
        }
        if (app_run_wrong_dir_ms >= 500U)
        {
          app_run_safe_disable();
          debug_log_write_line("[FAULT] speed direction mismatch");
          app_run_set_state(APP_RUN_STATE_FAULT);
          break;
        }
      }

      if ((uint32_t)(now_ms - app_run_tlm_ms) >= RUN_TLM_PERIOD_MS)
      {
        app_run_tlm_ms = now_ms;
        output = foc_runtime_get_last_output();
        if (app_run_mode == APP_RUN_MODE_SPEED)
        {
          app_run_logf("[RUN] id=%.3fA iq=%.3fA n=%.0f n*=%.0frpm",
                       (double)output->measured_current_a.d,
                       (double)output->measured_current_a.q,
                       (double)mech_rpm,
                       (double)app_run_speed_cmd_rpm);
        }
        else
        {
          app_run_logf("[RUN] id=%.3fA iq=%.3fA iq*=%+.2fA n=%.0frpm",
                       (double)output->measured_current_a.d,
                       (double)output->measured_current_a.q,
                       (double)app_run_iq_target_a,
                       (double)mech_rpm);
        }
      }
      break;

    case APP_RUN_STATE_READY:
    case APP_RUN_STATE_FAULT:
    default:
      break;
  }
}

void app_run_init(void)
{
  /* 初始化序对齐测试态 app_baseline_init（BL 安全基线）：
   * safe outputs → drv/pwm safe → 日志 → safety（内含 store init）
   * → encoder_cache → foc_runtime（注册 ADC 采样回调，缺它电流环死）。 */
  board_config_apply_safe_outputs();
  motor_drv_init_safe();
  motor_pwm_init_safe();

  debug_log_init(board_config_get_debug_uart());
  debug_log_write_line("STM32F103C8T6 motor run " APP_RUN_VERSION);
  debug_log_write_line("build: " __DATE__ " " __TIME__);
  debug_log_write_line("safety: V3P supply <=10V (12V PROHIBITED); motor output disabled at boot");
  debug_log_write_line("cmds: p=enable x=stop c=clear(fault) i/?=diag s=mode +/-=step 0=zero 1..4=speed-PI-gain");

  safety_manager_init();
  encoder_cache_init();
  foc_runtime_init();
  debug_log_write_line("[RUN] boot; evaluating admission (package/ADC zero/encoder)");
}

void app_run_poll(void)
{
  uint8_t command;

  if (HAL_UART_Receive(board_config_get_debug_uart(), &command, 1U, 0U) == HAL_OK)
  {
    app_run_handle_command(command);
  }
  encoder_cache_poll();
  safety_manager_poll();
  app_run_tick(HAL_GetTick());
  HAL_Delay(1U);
}
