/*
 * ISR 计算重构 PC 端等价性/误差测试。不依赖任何 HAL/STM32 头，仅用标准库 +
 * 被重构的纯函数层（foc_*），用于量化"预算/查表/惰性求值"相对原教科书公式的误差。
 *
 * 编译运行（在 STM32F103C8T6_BASE 目录下）：
 *   gcc -std=c99 -I Core/Inc \
 *     Core/Src/foc_precompute.c Core/Src/foc_current_model.c Core/Src/foc_trig_table.c \
 *     Core/Src/foc_transform.c Core/Src/foc_angle_math.c Core/Src/foc_pi.c \
 *     Core/Src/foc_svpwm.c Core/Src/foc_controller.c \
 *     tests/pc/test_foc_isr_refactor.c -lm -o tests/pc/trefactor
 *   ./tests/pc/trefactor        (Linux/macOS)
 *   tests\pc\trefactor.exe      (Windows)
 * 全部用例通过返回 0，否则返回非 0。
 *
 * 说明：本机开发环境无 C 编译器时，本文件只做静态交付，需在 MinGW / Linux 跑。
 */
#include <stdio.h>
#include <math.h>

#include "foc_precompute.h"
#include "foc_current_model.h"
#include "foc_trig_table.h"
#include "foc_transform.h"
#include "foc_pi.h"
#include "foc_svpwm.h"
#include "foc_controller.h"

static int g_failures = 0;
static int g_checks = 0;

static void check_close(const char *name, float got, float expect, float tol)
{
  float err = fabsf(got - expect);
  ++g_checks;
  if (err <= tol)
  {
    printf("[PASS] %-26s got=%.7f expect=%.7f\n", name, got, expect);
  }
  else
  {
    printf("[FAIL] %-26s got=%.7f expect=%.7f err=%.3e tol=%.1e\n",
           name, got, expect, err, tol);
    ++g_failures;
  }
}

static void check_true(const char *name, int cond)
{
  ++g_checks;
  if (cond)
  {
    printf("[PASS] %-26s\n", name);
  }
  else
  {
    printf("[FAIL] %-26s\n", name);
    ++g_failures;
  }
}

/* ---- 板级事实（与 board_config.h 一致，测试不包含板级头） ---- */
#define T_REF_V       (3.3f)
#define T_FULLSCALE   (4095.0f)
#define T_SHUNT       (0.01f)
#define T_GAIN        (50.0f)
#define T_BUS_V       (10.0f)
#define T_TICKS       (7199U)
#define T_PERIOD_S    (0.0001f)     /* 10kHz */

static void fill_base_params(foc_parameter_set_t *p)
{
  p->rotor.pole_pairs = 7U;
  p->rotor.encoder_direction = 1;
  p->rotor.electrical_offset_rad = 0.0f;
  p->current.adc_reference_v = T_REF_V;
  p->current.adc_full_scale_raw = T_FULLSCALE;
  p->current.shunt_resistance_ohm = T_SHUNT;
  p->current.amplifier_gain = T_GAIN;
  p->current.code_to_amp = 0.0f;
  p->current.phase_a_zero_raw = 2048.0f;
  p->current.phase_b_zero_raw = 2048.0f;
  p->current.phase_a_sign = 1.0f;
  p->current.phase_b_sign = 1.0f;
  p->current.calibrated = 1U;
  p->id_pi.kp = 2.0f;
  p->id_pi.ki = 500.0f;
  p->id_pi.ki_times_t = 0.0f;
  p->id_pi.integrator_min = -5.0f;
  p->id_pi.integrator_max = 5.0f;
  p->id_pi.output_min = -5.0f;
  p->id_pi.output_max = 5.0f;
  p->iq_pi = p->id_pi;
  p->bus_voltage_v = T_BUS_V;
  p->max_voltage_v = 5.0f;
  p->max_current_a = 1.0f;
  p->pwm_period_ticks = T_TICKS;
  p->inv_bus_voltage_v = 0.0f;
  p->pwm_period_f = 0.0f;
}

/* 旧教科书公式：电流换算（每拍含两次除法），用作等价参考。 */
static float legacy_raw_to_amp(float raw, float zero, float sign,
                               float ref, float full, float shunt, float gain)
{
  float v = ((raw - zero) * ref) / full;
  return (v * sign) / (shunt * gain);
}

/* 旧教科书公式：SVPWM（除母线、int->float），用作等价参考。 */
static void legacy_svpwm(float alpha, float beta, float bus, uint32_t ticks,
                         uint32_t *ta, uint32_t *tb, uint32_t *tc)
{
  const float sq32 = 0.86602540378443864676f;
  float va = alpha;
  float vb = -0.5f * alpha + sq32 * beta;
  float vc = -0.5f * alpha - sq32 * beta;
  float mx = va; if (vb > mx) mx = vb; if (vc > mx) mx = vc;
  float mn = va; if (vb < mn) mn = vb; if (vc < mn) mn = vc;
  float cm = 0.5f * (mx + mn);
  float da, db, dc;
  da = 0.5f + (va - cm) / bus; if (da < 0) da = 0; if (da > 1) da = 1;
  db = 0.5f + (vb - cm) / bus; if (db < 0) db = 0; if (db > 1) db = 1;
  dc = 0.5f + (vc - cm) / bus; if (dc < 0) dc = 0; if (dc > 1) dc = 1;
  *ta = (uint32_t)(da * (float)ticks);
  *tb = (uint32_t)(db * (float)ticks);
  *tc = (uint32_t)(dc * (float)ticks);
}

/* 旧 PI（ki*e*T 每拍现算，抗饱和逻辑与新版一致），用作等价参考。 */
typedef struct
{
  float integ;
  float kp, ki, imin, imax, omin, omax;
} legacy_pi_t;

static float legacy_pi_update(legacy_pi_t *s, float e, float T)
{
  float p = s->kp * e;
  float ni = s->integ + s->ki * e * T;
  if (ni < s->imin) ni = s->imin;
  if (ni > s->imax) ni = s->imax;
  float out = p + ni;
  if (((out > s->omax) && (e > 0.0f)) || ((out < s->omin) && (e < 0.0f)))
  {
    out = p + s->integ;
  }
  else
  {
    s->integ = ni;
  }
  if (out < s->omin) out = s->omin;
  if (out > s->omax) out = s->omax;
  return out;
}

static void test_precompute(void)
{
  foc_parameter_set_t p;
  fill_base_params(&p);
  check_true("precompute.ok",
             foc_parameter_set_precompute(&p, T_PERIOD_S) == FOC_STATUS_OK);
  check_close("precompute.code_to_amp", p.current.code_to_amp,
              T_REF_V / T_FULLSCALE / (T_SHUNT * T_GAIN), 1e-9f);
  check_close("precompute.inv_bus", p.inv_bus_voltage_v, 0.1f, 1e-9f);
  check_close("precompute.period_f", p.pwm_period_f, 7199.0f, 1e-6f);
  check_close("precompute.ki_t", p.id_pi.ki_times_t,
              500.0f * T_PERIOD_S, 1e-9f);

  /* 非法：bus<=0 必须拒绝且不产出半成品 */
  {
    foc_parameter_set_t q;
    fill_base_params(&q);
    q.bus_voltage_v = 0.0f;
    check_true("precompute.reject_bad_bus",
               foc_parameter_set_precompute(&q, T_PERIOD_S) != FOC_STATUS_OK);
  }
  /* 非法：周期<=0 */
  {
    foc_parameter_set_t q;
    fill_base_params(&q);
    check_true("precompute.reject_bad_T",
               foc_parameter_set_precompute(&q, 0.0f) != FOC_STATUS_OK);
  }
}

static void test_current_model(void)
{
  foc_parameter_set_t p;
  foc_raw_current_sample_t raw;
  foc_current_sample_t out;
  float raws[4] = {1900.0f, 2048.0f, 2200.0f, 2600.0f};
  int i;

  fill_base_params(&p);
  foc_parameter_set_precompute(&p, T_PERIOD_S);

  for (i = 0; i < 4; ++i)
  {
    float expect_pos = legacy_raw_to_amp(raws[i], 2048.0f, 1.0f,
                                         T_REF_V, T_FULLSCALE, T_SHUNT, T_GAIN);
    char nm[40];
    raw.phase_a_raw = (uint16_t)raws[i];
    raw.phase_b_raw = (uint16_t)raws[i];
    p.current.phase_a_sign = 1.0f;
    p.current.phase_b_sign = 1.0f;
    check_true("model.convert.ok",
               foc_current_model_convert(&p.current, &raw, &out) == FOC_STATUS_OK);
    snprintf(nm, sizeof(nm), "model.pos[%d]", i);
    check_close(nm, out.phase_current_a.phase_a, expect_pos, 1e-6f);

    /* sign=-1：条件取反结果等于旧公式乘 -1 */
    p.current.phase_a_sign = -1.0f;
    foc_current_model_convert(&p.current, &raw, &out);
    snprintf(nm, sizeof(nm), "model.neg[%d]", i);
    check_close(nm, out.phase_current_a.phase_a, -expect_pos, 1e-6f);
  }

  /* 缺预算（code_to_amp=0）必须 NOT_READY，防止忘了 precompute */
  {
    foc_current_calibration_t bad = p.current;
    bad.code_to_amp = 0.0f;
    raw.phase_a_raw = 2048; raw.phase_b_raw = 2048;
    check_true("model.reject_no_budget",
               foc_current_model_convert(&bad, &raw, &out) == FOC_STATUS_NOT_READY);
  }
  /* ic = -ia-ib 约束 */
  {
    raw.phase_a_raw = 2100; raw.phase_b_raw = 2000;
    p.current.phase_a_sign = p.current.phase_b_sign = 1.0f;
    foc_current_model_convert(&p.current, &raw, &out);
    check_close("model.ic_sum_zero",
                out.phase_current_a.phase_a + out.phase_current_a.phase_b +
                out.phase_current_a.phase_c, 0.0f, 1e-6f);
  }
}

static void test_pi_equivalence(void)
{
  foc_pi_config_t cfg;
  foc_pi_state_t st;
  legacy_pi_t leg;
  float errors[12] = {0.05f, -0.02f, 0.1f, 0.3f, 0.6f, 1.2f, -1.0f,
                      0.4f, 0.0f, -0.5f, 0.2f, 0.05f};
  int k;

  cfg.kp = 2.0f; cfg.ki = 500.0f; cfg.ki_times_t = 500.0f * T_PERIOD_S;
  cfg.integrator_min = cfg.output_min = -5.0f;
  cfg.integrator_max = cfg.output_max = 5.0f;
  foc_pi_init(&st, &cfg);

  leg.kp = 2.0f; leg.ki = 500.0f; leg.integ = 0.0f;
  leg.imin = leg.omin = -5.0f; leg.imax = leg.omax = 5.0f;

  for (k = 0; k < 12; ++k)
  {
    float got = foc_pi_update(&st, errors[k]);
    float exp = legacy_pi_update(&leg, errors[k], T_PERIOD_S);
    char nm[32];
    snprintf(nm, sizeof(nm), "pi.eq[%d]", k);
    check_close(nm, got, exp, 1e-5f);
  }

  /* 限幅：持续大正误差，输出不得超过 output_max */
  {
    foc_pi_init(&st, &cfg);
    float v = 0.0f;
    for (k = 0; k < 200; ++k) v = foc_pi_update(&st, 5.0f);
    check_true("pi.clamp_max", v <= 5.0f + 1e-6f);
    check_close("pi.clamp_max_val", v, 5.0f, 1e-5f);
  }
}

static void test_svpwm_equivalence(void)
{
  foc_alpha_beta_t ab;
  foc_pwm_output_t out;
  float cases[6][2] = {
    {0.0f, 0.0f}, {1.0f, 0.0f}, {-1.0f, 0.5f}, {0.3f, -0.8f},
    {2.0f, 2.0f}, {-0.2f, -0.1f}
  };
  int k;
  for (k = 0; k < 6; ++k)
  {
    uint32_t la, lb, lc;
    char nm[32];
    ab.alpha = cases[k][0];
    ab.beta = cases[k][1];
    legacy_svpwm(ab.alpha, ab.beta, T_BUS_V, T_TICKS, &la, &lb, &lc);
    check_true("svpwm.ok",
               foc_svpwm_calculate(&ab, 1.0f / T_BUS_V, (float)T_TICKS, &out)
               == FOC_STATUS_OK);
    snprintf(nm, sizeof(nm), "svpwm.a[%d]", k);
    check_true(nm, (out.phase_a_ticks <= la + 1U) && (out.phase_a_ticks + 1U >= la));
    snprintf(nm, sizeof(nm), "svpwm.b[%d]", k);
    check_true(nm, (out.phase_b_ticks <= lb + 1U) && (out.phase_b_ticks + 1U >= lb));
    snprintf(nm, sizeof(nm), "svpwm.c[%d]", k);
    check_true(nm, (out.phase_c_ticks <= lc + 1U) && (out.phase_c_ticks + 1U >= lc));
    check_true("svpwm.in_range",
               (out.phase_a_ticks <= T_TICKS) && (out.phase_b_ticks <= T_TICKS) &&
               (out.phase_c_ticks <= T_TICKS));
  }
  /* 非法预算拒绝 */
  check_true("svpwm.reject_bad_inv",
             foc_svpwm_calculate(&ab, 0.0f, 7199.0f, &out) == FOC_STATUS_NOT_READY);
}

static void test_trig_table(void)
{
  const float pi = 3.14159265358979f;
  float max_sin_err = 0.0f, max_cos_err = 0.0f;
  int n;

  /* 全角度（含负角、超 2pi）密集对比 libm，线性插值误差上限 1.2e-4 */
  for (n = 0; n < 20000; ++n)
  {
    float a = -pi + (4.0f * pi) * ((float)n / 19999.0f);
    float s, c, es, ec;
    foc_trig_sin_cos(a, &s, &c);
    es = fabsf(s - sinf(a));
    ec = fabsf(c - cosf(a));
    if (es > max_sin_err) max_sin_err = es;
    if (ec > max_cos_err) max_cos_err = ec;
    if (fabsf(s * s + c * c - 1.0f) > 3e-4f)
    {
      printf("[FAIL] trig.unit_circle at %.4f s=%.6f c=%.6f\n", a, s, c);
      ++g_failures;
      break;
    }
  }
  printf("[INFO] trig max |sin err|=%.3e  |cos err|=%.3e (budget 1.2e-4)\n",
         max_sin_err, max_cos_err);
  check_true("trig.sin_within_budget", max_sin_err < 1.2e-4f);
  check_true("trig.cos_within_budget", max_cos_err < 1.2e-4f);

  /* 四整格点精确（自检恒角 theta=0 无量化残差） */
  {
    float s, c;
    foc_trig_sin_cos(0.0f, &s, &c);
    check_close("trig.angle0.sin", s, 0.0f, 1e-9f);
    check_close("trig.angle0.cos", c, 1.0f, 1e-9f);
    foc_trig_sin_cos(pi * 0.5f, &s, &c);
    check_close("trig.angle90.sin", s, 1.0f, 1e-9f);
    check_close("trig.angle90.cos", c, 0.0f, 1e-9f);
    foc_trig_sin_cos(pi, &s, &c);
    check_close("trig.angle180.sin", s, 0.0f, 1e-9f);
    check_close("trig.angle180.cos", c, -1.0f, 1e-9f);
    foc_trig_sin_cos(pi * 1.5f, &s, &c);
    check_close("trig.angle270.sin", s, -1.0f, 1e-9f);
    check_close("trig.angle270.cos", c, 0.0f, 1e-9f);
  }

  /* 周期一致：theta 与 theta+2pi 结果相同（折叠正确） */
  {
    float s1, c1, s2, c2;
    foc_trig_sin_cos(0.7f, &s1, &c1);
    foc_trig_sin_cos(0.7f + 2.0f * pi, &s2, &c2);
    check_close("trig.periodic_sin", s1, s2, 1e-7f);
    check_close("trig.periodic_cos", c1, c2, 1e-7f);
  }
}

static void test_controller_smoke(void)
{
  /* 集成冒烟：已预算参数集跑一个 CURRENT 步，链路接通、输出合法。 */
  foc_parameter_set_t p;
  foc_controller_t ctrl;
  foc_control_input_t in;
  foc_control_output_t out;

  fill_base_params(&p);
  foc_parameter_set_precompute(&p, T_PERIOD_S);
  foc_controller_init(&ctrl, &p);

  in.phase_current_a.phase_a = 0.05f;
  in.phase_current_a.phase_b = -0.02f;
  in.phase_current_a.phase_c = -0.03f;
  in.electrical_angle_rad = 0.0f;
  in.current_target_a.d = 0.0f;
  in.current_target_a.q = 0.1f;
  in.voltage_command_v.d = 0.0f;
  in.voltage_command_v.q = 0.0f;
  in.mode = FOC_CONTROL_CURRENT;
  in.control_period_s = T_PERIOD_S;

  check_true("smoke.current_step_ok",
             foc_controller_step(&ctrl, &in, &out) == FOC_STATUS_OK);
  check_true("smoke.ticks_in_range",
             (out.pwm.phase_a_ticks <= T_TICKS) &&
             (out.pwm.phase_b_ticks <= T_TICKS) &&
             (out.pwm.phase_c_ticks <= T_TICKS));

  /* 未预算（inv_bus=0）必须 NOT_READY */
  {
    foc_parameter_set_t q;
    foc_controller_t c2;
    fill_base_params(&q); /* 故意不 precompute */
    foc_controller_init(&c2, &q);
    check_true("smoke.reject_no_budget",
               foc_controller_step(&c2, &in, &out) == FOC_STATUS_NOT_READY);
  }
}

int main(void)
{
  test_precompute();
  test_current_model();
  test_pi_equivalence();
  test_svpwm_equivalence();
  test_trig_table();
  test_controller_smoke();

  printf("\n==== %d checks, %d failures ====\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}
