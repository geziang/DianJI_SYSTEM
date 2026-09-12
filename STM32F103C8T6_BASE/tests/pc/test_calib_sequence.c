/*
 * calib_sequence / calib_stats 的 PC 端解析解单测（批次 A）。
 * 不依赖任何 HAL / STM32 头，仅编译纯算法：
 *
 *   gcc -std=c99 -I../../Core/Inc ../../Core/Src/calib_sequence.c \
 *       ../../Core/Src/calib_stats.c test_calib_sequence.c -lm -o test_seq
 *   ./test_seq        # 全部用例通过返回 0，否则返回失败用例数
 *
 * 本机无 gcc 时随源码交付，在装有 MinGW / w64devkit 的环境执行上述命令。
 */
#include <stdio.h>
#include <math.h>

#include "calib_sequence.h"
#include "calib_config.h"
#include "calib_stats.h"

static int g_fail;

static void check(int cond, const char *name)
{
  if (cond)
  {
    printf("PASS %s\n", name);
  }
  else
  {
    printf("FAIL %s\n", name);
    g_fail++;
  }
}

static int nearf(float a, float b, float tol)
{
  return fabsf(a - b) <= tol;
}

static void test_s1a(void)
{
  calib_s1_result_t r;
  uint8_t ok, i;
  /* 标准增益：1 code = 3.3/4095/(0.01*50) A；R=2.3Ω 对应斜率 ku=1/(R*a_per_raw) code/V。 */
  const float a_per_raw = 3.3f / 4095.0f / (0.01f * 50.0f);
  const float rnom = 2.3f, tlo = 40.0f;
  const float ku = 1.0f / (rnom * a_per_raw);
  /* 5 个爬坡档，最低档 0.14V→37.7code 已高于最小可用门限(30)，全部参与拟合。 */
  float vdp[5] = {0.14f, 0.17f, 0.20f, 0.23f, 0.26f};
  float vdn[5] = {-0.14f, -0.17f, -0.20f, -0.23f, -0.26f};
  float dup[5], dvp[5], dun[5], dvn[5];

  /* 正常：du=ku*Vd、dv=-0.5du，正/负对称 -> sign +1/-1、ratio=-0.5、R≈2.3、进带、PASS */
  for (i = 0U; i < 5U; i++)
  {
    dup[i] = ku * vdp[i]; dvp[i] = -0.5f * dup[i];
    dun[i] = ku * vdn[i]; dvn[i] = -0.5f * dun[i];
  }
  ok = calib_sequence_s1a_fit(vdp, dup, dvp, 5U, vdn, dun, dvn, 5U,
                              a_per_raw, rnom, tlo, 0.20f, &r);
  check(ok && r.sign_u == 1 && r.sign_v == -1 &&
        nearf(r.ratio_vu, -0.5f, 1e-3f) && r.reached_band && r.bipolar_ok &&
        r.symmetry_ok && nearf(r.r_est_ohm, 2.3f, 0.05f) &&
        r.pos_pts == 5U && r.neg_pts == 5U,
        "s1a sweep normal ratio -0.5 R~2.3");

  /* 整体反接：U/V 同时反号 -> sign -1/+1，斜率比仍 -0.5，PASS */
  for (i = 0U; i < 5U; i++)
  {
    dup[i] = -ku * vdp[i]; dvp[i] = -0.5f * dup[i];
    dun[i] = -ku * vdn[i]; dvn[i] = -0.5f * dun[i];
  }
  ok = calib_sequence_s1a_fit(vdp, dup, dvp, 5U, vdn, dun, dvn, 5U,
                              a_per_raw, rnom, tlo, 0.20f, &r);
  check(ok && r.sign_u == -1 && r.sign_v == 1 && nearf(r.ratio_vu, -0.5f, 1e-3f),
        "s1a sweep reversed still ratio -0.5");

  /* 通道错位：dv=du（斜率比 +1）-> channel_ratio_ok=0，NOGO */
  for (i = 0U; i < 5U; i++)
  {
    dup[i] = ku * vdp[i]; dvp[i] = dup[i];
    dun[i] = ku * vdn[i]; dvn[i] = dun[i];
  }
  ok = calib_sequence_s1a_fit(vdp, dup, dvp, 5U, vdn, dun, dvn, 5U,
                              a_per_raw, rnom, tlo, 0.20f, &r);
  check(!ok && r.channel_ratio_ok == 0U, "s1a sweep channel mismatch nogo");

  /* 弱信号：斜率仅 50code/V，最大档才 13code<门限30，全被筛、有效点不足 -> linear_ok=0 */
  for (i = 0U; i < 5U; i++)
  {
    dup[i] = 50.0f * vdp[i]; dvp[i] = -0.5f * dup[i];
    dun[i] = 50.0f * vdn[i]; dvn[i] = -0.5f * dun[i];
  }
  ok = calib_sequence_s1a_fit(vdp, dup, dvp, 5U, vdn, dun, dvn, 5U,
                              a_per_raw, rnom, tlo, 0.20f, &r);
  check(!ok && r.linear_ok == 0U, "s1a sweep weak signal all filtered nogo");

  /* 达最小可用但未进目标带：slope=180，三档 30.6/33.3/36code 均∈[30,40) -> 线性成立、reached=0 */
  {
    float wp[3] = {0.17f, 0.185f, 0.20f};
    float wn[3] = {-0.17f, -0.185f, -0.20f};
    float up[3], vp[3], un[3], vn[3];
    for (i = 0U; i < 3U; i++)
    {
      up[i] = 180.0f * wp[i]; vp[i] = -0.5f * up[i];
      un[i] = 180.0f * wn[i]; vn[i] = -0.5f * un[i];
    }
    ok = calib_sequence_s1a_fit(wp, up, vp, 3U, wn, un, vn, 3U,
                                a_per_raw, rnom, tlo, 0.20f, &r);
    check(!ok && r.linear_ok && r.reached_band == 0U, "s1a usable but below target band nogo");
  }

  /* 正负不对称：正向 ku、负向 0.75ku（筛后各≥3点、同号但幅值差大）-> bipolar 成立、symmetry_ok=0 */
  for (i = 0U; i < 5U; i++)
  {
    dup[i] = ku * vdp[i]; dvp[i] = -0.5f * dup[i];
    dun[i] = 0.75f * ku * vdn[i]; dvn[i] = -0.5f * dun[i];
  }
  ok = calib_sequence_s1a_fit(vdp, dup, dvp, 5U, vdn, dun, dvn, 5U,
                              a_per_raw, rnom, tlo, 0.20f, &r);
  check(!ok && r.bipolar_ok && r.symmetry_ok == 0U, "s1a sweep asymmetry nogo");
}

static void test_probe3(void)
{
  calib_probe3_result_t r;
  uint8_t ok;
  /* 正常 s++：0° U 独大、120° V 独大，余者 -0.5；第三相 -(U+V) 独大在 240°。 */
  float du1[3] = { 80.0f, -40.0f, -40.0f};
  float dv1[3] = {-40.0f,  80.0f, -40.0f};
  ok = calib_sequence_s1_probe3(du1, dv1, &r);
  check(ok && r.sign_u == 1 && r.sign_v == 1 &&
        r.peak_u_angle == 0U && r.peak_v_angle == 1U &&
        r.channel_map_ok && r.symmetry_ok && r.third_phase_ok && r.amplitude_ok &&
        nearf(r.ratio_mean, -0.5f, 1e-3f), "probe3 normal sign++ passes");

  /* 整体反接 s--：符号同时为负，比值判据不变仍 PASS。 */
  {
    float du[3] = {-80.0f,  40.0f,  40.0f};
    float dv[3] = { 40.0f, -80.0f,  40.0f};
    ok = calib_sequence_s1_probe3(du, dv, &r);
    check(ok && r.sign_u == -1 && r.sign_v == -1 && nearf(r.ratio_mean, -0.5f, 1e-3f),
          "probe3 reversed sign-- still passes");
  }

  /* 通道交叉(硬门)：U 独大在120、V 独大在0 -> channel_map=0、hard bit1，仍 NOGO 不放水。 */
  {
    float du[3] = {-40.0f,  80.0f, -40.0f};
    float dv[3] = { 80.0f, -40.0f, -40.0f};
    ok = calib_sequence_s1_probe3(du, dv, &r);
    check(!ok && r.channel_map_ok == 0U &&
          (r.fail_mask & 0x02U) && (r.hard_mask & 0x02U),
          "probe3 channel cross hard nogo");
  }

  /* 弱信号(硬门)：独大仅 10code < 门限30 -> weak、hard bit0，NOGO。 */
  {
    float du[3] = { 10.0f, -5.0f, -5.0f};
    float dv[3] = {-5.0f,  10.0f, -5.0f};
    ok = calib_sequence_s1_probe3(du, dv, &r);
    check(!ok && (r.fail_mask & 0x01U) && (r.hard_mask & 0x01U),
          "probe3 weak excitation hard nogo");
  }

  /* 不对称(软门)：V 在240° 应 -40 却 -10（比值 -0.125）-> symmetry=0、soft bit2，
   * 但通道对、不弱、等幅，硬门全清，仍 ok=1 可绑定（低端硬件不被精度卡住）。 */
  {
    float du[3] = { 80.0f, -40.0f, -40.0f};
    float dv[3] = {-40.0f,  80.0f, -10.0f};
    ok = calib_sequence_s1_probe3(du, dv, &r);
    check(ok && r.channel_map_ok && r.amplitude_ok &&
          r.symmetry_ok == 0U && (r.fail_mask & 0x04U) && (r.hard_mask == 0U),
          "probe3 asymmetry soft-warn but still bindable");
  }
}

static void test_s2(void)
{
  /* V = 2.3*I + 0.01，三档 */
  float id[3] = {0.04f, 0.06f, 0.08f};
  float vd[3];
  calib_s2_result_t r;
  uint8_t ok;
  int i;
  for (i = 0; i < 3; i++) vd[i] = 2.3f * id[i] + 0.01f;
  ok = calib_sequence_s2_evaluate(id, vd, 2.3f, &r);
  check(ok && nearf(r.resistance_ohm, 2.3f, 1e-4f) &&
        nearf(r.v_offset_v, 0.01f, 1e-4f) && r.r2 > 0.999f,
        "s2 least squares recover R=2.3 offset=0.01");
}

static void test_s3(void)
{
  /* 造线性上升：id=i*did，vd=R*id+L*slope；则每点 L 恒为 0.00086 */
  calib_sample_t s[24];
  calib_s3_result_t r;
  uint8_t ok;
  int i;
  const float ts = 50.0e-6f, slope = 100.0f, R = 2.3f, L = 0.00086f;
  for (i = 0; i < 24; i++)
  {
    s[i].id = (float)i * slope * ts;
    s[i].vd = R * s[i].id + L * slope;
    s[i].cycle = (uint32_t)i;
  }
  ok = calib_sequence_s3_evaluate(s, 24U, 2U, R, ts, &r);
  check(ok && nearf(r.inductance_h, L, L * 0.05f),
        "s3 window recover L=0.86mH");
}

static void run_s4_curve(float (*id_fn)(uint32_t, void *), void *arg,
                         uint32_t observe_ms, calib_s4_result_t *out)
{
  calib_s4_tracker_t t;
  uint32_t e;
  calib_sequence_s4_begin(&t, 0.06f, 0.010f, 0.50f, 0.02f, 5U,
                          CALIB_S4_SETTLE_WINDOW_CYCLES, 5U, 8U);
  for (e = 1U; e <= observe_ms; e++)
  {
    float id = id_fn(e, arg);
    calib_sequence_s4_update(&t, id, 0.10f, e);
  }
  /* spread 门限 2%：对称振荡被滑窗均值隐藏时，由 spread_ok 兜底拒绝；
   * 穿出占比 35%、最长连续穿出 2 个窗口（噪声孤立穿出豁免）。 */
  calib_sequence_s4_evaluate(&t, 0.02f, 0.10f, 0.02f, 0.35f, 2U, out);
}

static float good_curve(uint32_t e, void *arg)
{
  (void)arg;
  /* 时间常数 3.33ms 指数趋近，无超调无振荡 */
  return 0.06f * (1.0f - expf(-(float)e / 3.333f));
}

static float oscillating_curve(uint32_t e, void *arg)
{
  (void)arg;
  if (e < 20U) return 0.06f * (1.0f - expf(-(float)e / 3.333f));
  /* 建立后在 ±3% 交替，反复穿出 2% 稳态带 */
  return (e & 1U) ? 0.06f * 1.03f : 0.06f * 0.97f;
}

static void test_s4(void)
{
  calib_s4_result_t out;
  run_s4_curve(good_curve, NULL, 80U, &out);
  check(out.settle_ok && out.steady_err_ok && out.no_osc_ok &&
        out.saturation_ok && out.spread_ok,
        "s4 good step passes all five criteria");
  check(out.settle_time_ms <= 50.0f, "s4 settle within 5*Tc=50ms");

  run_s4_curve(oscillating_curve, NULL, 80U, &out);
  check(!out.no_osc_ok || !out.spread_ok,
        "s4 post-settle oscillation rejected (band-exit or spread)");
}

int main(void)
{
  g_fail = 0;
  test_s1a();
  test_probe3();
  test_s2();
  test_s3();
  test_s4();
  if (g_fail == 0)
  {
    printf("ALL calib_sequence TESTS PASSED\n");
  }
  else
  {
    printf("%d TEST(S) FAILED\n", g_fail);
  }
  return g_fail;
}
