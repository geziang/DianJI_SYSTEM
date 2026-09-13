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
#include "foc_precompute.h"
#include "foc_rotor_model.h"
#include "foc_runtime.h"
#include "motor_adc.h"
#include "motor_drv.h"
#include "motor_pwm.h"
#include "safety_manager.h"

#include <stdarg.h>
#include <stdio.h>

/* 运行态限值（与测试态 TEST_* 同源同值：10V 红线/电压饱和裕量/1A 硬限）。 */
#define RUN_BUS_VOLTAGE_V   (10.0f)
#define RUN_MAX_VOLTAGE_V   (5.5f)
#define RUN_MAX_CURRENT_A   (1.0f)

#define RUN_TLM_PERIOD_MS   (500U)  /* 使能态遥测周期（R3 正式设计再定频） */
#define RUN_ENC_VALID_MS    (20U)   /* 编码器新鲜度门（同 PSEQ 判决口径） */

static app_run_state_t app_run_state = APP_RUN_STATE_BOOT;
static foc_parameter_set_t app_run_parameters;
static motor_parameter_package_t app_run_package;
static uint8_t app_run_has_package = 0U;
static uint32_t app_run_state_tick = 0U;
static uint32_t app_run_tlm_ms = 0U;
static float app_run_last_mech_rad = 0.0f;
static uint8_t app_run_have_last_mech = 0U;
static uint32_t app_run_hint_ms = 0U;

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
    app_run_state_tick = HAL_GetTick();
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
  motor_adc_current_diagnostic_t diagnostic;
  char line[160];

  if (package == (const motor_parameter_package_t *)0)
  {
    return;
  }
  app_run_package = *package;
  app_run_has_package = 1U;

  /* 包在=该板测试固件已走完 S1a 符号绑定，方向验证置位联动 closed_loop 门。
   * 零偏仍取本拍实测（同测试态 prepare 原则）。 */
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
  /* PI 直取包内标定值（ki 由固化时的生效 R 驱动，含 learned_R 链）。 */
  app_run_parameters.id_pi.kp = app_run_package.id_kp;
  app_run_parameters.id_pi.ki = app_run_package.id_ki;
  app_run_parameters.iq_pi.kp = app_run_package.iq_kp;
  app_run_parameters.iq_pi.ki = app_run_package.iq_ki;
  /* DD-01 契约：运行态 phase_map 从固化包取，不经测试参数集。
   * 注意 direction_verified 不在此处置位——本函数运行于 safety_manager
   * PARAMETER_CHECK 阶段，随后的 ADC_ZERO_REFRESH 会 reset_diagnostic()
   * 把它洗掉（2026-09-13 上板实测）；改在 BOOT→READY 准入通过时置位。 */
  app_run_parameters.phase_map.phase_a_output = app_run_package.phase_map_a;
  app_run_parameters.phase_map.phase_b_output = app_run_package.phase_map_b;
  app_run_parameters.phase_map.phase_c_output = app_run_package.phase_map_c;

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

  if (foc_parameter_set_precompute(&app_run_parameters,
                                   BOARD_CONFIG_CURRENT_LOOP_PERIOD_S) !=
      FOC_STATUS_OK)
  {
    app_run_has_package = 0U;
    debug_log_write_line("[FAULT] run parameter precompute failed");
  }
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
  app_run_have_last_mech = 0U;
  app_run_tlm_ms = HAL_GetTick();
  debug_log_write_line("[RUN] enabled: zero-current hold (id=0 iq=0)");
  app_run_set_state(APP_RUN_STATE_ENABLED);
  return 1U;
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

  switch (app_run_state)
  {
    case APP_RUN_STATE_BOOT:
      /* 准入评估（FR-3.2 骨架）：包 + 零偏 + 编码器。safety_manager
       * PARAMETER_CHECK 异步完成装载，此处逐拍等其结果。 */
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
        break; /* 等零偏刷新完成 */
      }
      if (encoder_cache_is_valid(RUN_ENC_VALID_MS) == 0U)
      {
        break;
      }
      /* 零偏刷新已结束（quality 刚确认 OK），此刻置位才不会被洗掉；
       * 包的存在=该板 S1a 已绑定符号（direction 证明）。 */
      motor_adc_set_direction_verified(1U);
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

      if ((uint32_t)(now_ms - app_run_tlm_ms) >= RUN_TLM_PERIOD_MS)
      {
        float dt_s;
        float rpm;

        app_run_tlm_ms = now_ms;
        output = foc_runtime_get_last_output();
        mech_rad = foc_mechanical_raw_to_rad(enc->raw_angle);
        rpm = 0.0f;
        if (app_run_have_last_mech != 0U)
        {
          dt_s = (float)(now_ms - app_run_state_tick) / 1000.0f;
          rpm = (dt_s > 0.0f) ? ((mech_rad - app_run_last_mech_rad) / dt_s) *
                                (60.0f / 6.28318530718f) : 0.0f;
        }
        app_run_last_mech_rad = mech_rad;
        app_run_have_last_mech = 1U;
        app_run_logf("[RUN] id=%.3fA iq=%.3fA n=%.1frpm",
                     (double)output->measured_current_a.d,
                     (double)output->measured_current_a.q,
                     (double)rpm);
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
  debug_log_write_line("cmds: p=enable(zero-current) x=stop c=clear(fault) i/?=diag");

  safety_manager_init();
  encoder_cache_init();
  foc_runtime_init();
  app_run_state_tick = HAL_GetTick();
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
