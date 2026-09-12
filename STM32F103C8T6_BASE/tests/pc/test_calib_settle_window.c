/*
 * 情况 A（收敛判定抗单拍噪声）PC 端回放单测。
 *
 * 覆盖：
 *  - calib_step：滑动窗均值在带判定（FR2）、COLLECT spread 纹波门限（FR3）；
 *  - calib_sequence S4：16 拍滑窗建立、建立后穿出占比/连续长度判据、
 *    建立后窗内 spread 门限（FR5）；
 *  - 用 2026-09 实测串口 boot2 的 V 通道 64 个零偏码值做相关噪声回放，
 *    叠加快路径 10 次过采样（FR1 的主机侧模型），证明：
 *    W=8 在实测相关噪声下进不了 S4 的 2% 带，W=16 可 64/64 在 50ms 内建立。
 *
 * 不依赖 HAL/STM32 头，仅编译纯算法（在 STM32F103C8T6_BASE 目录下）：
 *   gcc -std=c99 -I Core/Inc Core/Src/calib_step.c Core/Src/calib_sequence.c \
 *       Core/Src/calib_stats.c tests/pc/test_calib_settle_window.c -lm \
 *       -o tests/pc/tw
 *   ./tests/pc/tw        # 全部用例通过返回 0，否则返回失败用例数
 */
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "calib_step.h"
#include "calib_sequence.h"
#include "calib_config.h"

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

/* 确定性 LCG，保证回放可复现（不依赖 rand 实现）。 */
static uint32_t lcg_next(uint32_t x)
{
  return (uint32_t)(1103515245UL * x + 12345UL) & 0x7fffffffUL;
}

static float lcg_noise(uint32_t *seed, float amp)
{
  *seed = lcg_next(*seed);
  return (((float)(*seed) / 2147483647.0f) * 2.0f - 1.0f) * amp;
}

static void make_cfg(calib_step_cfg_t *cfg, float target, float band,
                     uint16_t collect_n, uint16_t window, float spread_max)
{
  cfg->target = target;
  cfg->settle_band = band;
  cfg->settle_hold_cycles = CALIB_SETTLE_HOLD_CYCLES;
  cfg->collect_n = collect_n;
  cfg->timeout_ms = CALIB_STEP_TIMEOUT_MS;
  cfg->settle_window_cycles = window;
  cfg->spread_max = spread_max;
}

/* ---------------- calib_step 用例 ---------------- */

static void test_step_clean(void)
{
  calib_step_t s;
  calib_step_cfg_t cfg;
  calib_step_phase_t ph = CALIB_STEP_EXCITE;
  uint16_t t;
  make_cfg(&cfg, 0.04f, 0.010f, CALIB_COLLECT_N, CALIB_SETTLE_WINDOW_CYCLES,
           CALIB_S1B_SPREAD_MAX_A);
  calib_step_begin(&s, &cfg, 0U);
  /* 干净恒定输入：W=8 填满(8) + hold5 + collect8 = 第 20 拍 EVAL（首拍 EXCITE）。 */
  for (t = 1U; t <= 25U; t++)
  {
    ph = calib_step_tick(&s, 0.04f, (uint32_t)t);
  }
  check(ph == CALIB_STEP_EVALUATE && fabsf(s.mean - 0.04f) < 1e-6f &&
        s.spread == 0.0f, "step clean constant reaches EVALUATE, mean exact");
}

static void test_step_fr1_residual(void)
{
  calib_step_t s;
  calib_step_cfg_t cfg;
  calib_step_phase_t ph = CALIB_STEP_EXCITE;
  uint32_t seed = 7U;
  uint16_t t;
  /* 模拟 FR1 拍均值残余：±6mA 均匀噪声（原始单采样 ±20mA 经 10 次过采样量级）。 */
  make_cfg(&cfg, 0.04f, 0.010f, CALIB_COLLECT_N, CALIB_SETTLE_WINDOW_CYCLES,
           CALIB_S1B_SPREAD_MAX_A);
  calib_step_begin(&s, &cfg, 0U);
  for (t = 1U; t <= 40U; t++)
  {
    ph = calib_step_tick(&s, 0.04f + lcg_noise(&seed, 0.006f), (uint32_t)t);
    if (ph == CALIB_STEP_EVALUATE || ph == CALIB_STEP_NOGO) break;
  }
  check(ph == CALIB_STEP_EVALUATE && t <= 30U && fabsf(s.mean - 0.04f) < 0.004f,
        "step FR1-residual noise settles within 30 ticks, mean centered");
}

static void test_step_window_beats_legacy(void)
{
  /* 同一条 ±20mA 原始噪声序列（固定种子）：W=8 应快速 EVAL，W=0 第 40 拍仍在 SETTLE。 */
  calib_step_t sw, sl;
  calib_step_cfg_t cw, cl;
  calib_step_phase_t pw = CALIB_STEP_EXCITE, pl = CALIB_STEP_EXCITE;
  uint32_t seedw = 1U, seedl = 1U;
  uint16_t t;
  make_cfg(&cw, 0.04f, 0.010f, CALIB_COLLECT_N, CALIB_SETTLE_WINDOW_CYCLES, 0.0f);
  make_cfg(&cl, 0.04f, 0.010f, CALIB_COLLECT_N, 0U, 0.0f);
  calib_step_begin(&sw, &cw, 0U);
  calib_step_begin(&sl, &cl, 0U);
  for (t = 1U; t <= 40U; t++)
  {
    pw = calib_step_tick(&sw, 0.04f + lcg_noise(&seedw, 0.020f), (uint32_t)t);
    pl = calib_step_tick(&sl, 0.04f + lcg_noise(&seedl, 0.020f), (uint32_t)t);
  }
  check(pw == CALIB_STEP_EVALUATE, "windowed judge reaches EVALUATE by 40 ticks");
  check(pl == CALIB_STEP_SETTLE, "legacy per-tick judge still settling at 40 ticks");
}

static void test_step_offset_nogo(void)
{
  calib_step_t s;
  calib_step_cfg_t cfg;
  calib_step_phase_t ph = CALIB_STEP_EXCITE;
  uint32_t seed = 3U;
  uint32_t t;
  /* 持续偏置 +15mA（超出 ±10mA 带），必须在 2000ms 超时后 NOGO。 */
  make_cfg(&cfg, 0.04f, 0.010f, CALIB_COLLECT_N, CALIB_SETTLE_WINDOW_CYCLES,
           CALIB_S1B_SPREAD_MAX_A);
  calib_step_begin(&s, &cfg, 0U);
  for (t = 1U; t <= 2001U; t++)
  {
    ph = calib_step_tick(&s, 0.055f + lcg_noise(&seed, 0.001f), t);
    if (ph == CALIB_STEP_NOGO) break;
  }
  check(ph == CALIB_STEP_NOGO && t > 2000U, "persistent 15mA offset -> timeout NOGO");
}

static void test_step_spread_gate(void)
{
  calib_step_t s;
  calib_step_cfg_t cfg;
  calib_step_phase_t ph = CALIB_STEP_EXCITE;
  uint16_t t;
  /* 均值严格居中但对称振荡 ±16mA：窗均值在带内能 SETTLE，COLLECT 的
   * 样本标准差约 17mA > 15mA 门限，必须 NOGO（防止均值掩盖抖动）。 */
  make_cfg(&cfg, 0.04f, 0.010f, CALIB_COLLECT_N, CALIB_SETTLE_WINDOW_CYCLES,
           CALIB_S1B_SPREAD_MAX_A);
  calib_step_begin(&s, &cfg, 0U);
  for (t = 1U; t <= 40U; t++)
  {
    float m = 0.04f + ((t & 1U) ? 0.016f : -0.016f);
    ph = calib_step_tick(&s, m, (uint32_t)t);
    if (ph == CALIB_STEP_EVALUATE || ph == CALIB_STEP_NOGO) break;
  }
  check(ph == CALIB_STEP_NOGO && s.spread > CALIB_S1B_SPREAD_MAX_A,
        "centered but +-16mA ripple -> spread NOGO");
}

/* ---------------- S4 用例 ---------------- */

/* 实测 boot2 V 通道 64 个零偏码值（2026-09 串口记录，p2p=35、质量 NOISY 那次）。 */
static const int16_t boot2_v[64] =
{
  1650,1657,1659,1660,1669,1667,1669,1669,1663,1659,
  1661,1665,1653,1654,1671,1663,1661,1660,1653,1650,
  1662,1661,1657,1647,1647,1655,1669,1670,1675,1678,
  1666,1661,1665,1671,1670,1668,1668,1670,1667,1666,
  1668,1668,1669,1677,1677,1675,1679,1675,1667,1668,
  1654,1661,1661,1656,1659,1666,1658,1661,1649,1649,
  1655,1655,1654,1644
};
#define BOOT_N 64
static float boot_noise_a[BOOT_N];
static float boot_mean_code;

static void boot_prepare(void)
{
  int i;
  float sum = 0.0f;
  const float a_per_raw = 3.3f / 4095.0f / (0.01f * 50.0f); /* 1 code = 1.612mA */
  for (i = 0; i < BOOT_N; i++) sum += (float)boot2_v[i];
  boot_mean_code = sum / (float)BOOT_N;
  for (i = 0; i < BOOT_N; i++)
  {
    boot_noise_a[i] = ((float)boot2_v[i] - boot_mean_code) * a_per_raw;
  }
}

/* 跑一遍 80ms S4：每拍 10 个连续实测噪声平均（FR1 模型）+ 指数趋近曲线，
 * fast_pct=建立后对称交替振荡；slow_pct/period=慢正弦振荡。返回 evaluate 结果。 */
static void run_s4_boot(uint16_t window, uint16_t start, float fast_pct,
                        float slow_pct, float period_ms,
                        calib_s4_result_t *out, uint8_t *settled)
{
  calib_s4_tracker_t t;
  uint16_t e;
  uint16_t k = start;
  calib_sequence_s4_begin(&t, 0.06f, 0.010f, 0.55f, CALIB_S4_SETTLING_BAND_RATIO,
                          CALIB_SETTLE_HOLD_CYCLES, window,
                          CALIB_S4_SETTLING_TIME_MAX_TC, CALIB_S4_OBSERVE_TC);
  for (e = 1U; e <= 80U; e++)
  {
    float acc = 0.0f;
    float id;
    uint8_t j;
    for (j = 0U; j < 10U; j++)
    {
      acc += boot_noise_a[(uint16_t)(k + j) % BOOT_N];
    }
    k = (uint16_t)((k + 10U) % BOOT_N);
    id = 0.06f * (1.0f - expf(-(float)e / 3.333f)) + acc / 10.0f;
    if ((fast_pct > 0.0f) && (e >= 20U))
    {
      id += 0.06f * fast_pct * ((e & 1U) ? 1.0f : -1.0f);
    }
    if (slow_pct > 0.0f)
    {
      id += 0.06f * slow_pct * sinf(2.0f * 3.14159265f * (float)e / period_ms);
    }
    (void)calib_sequence_s4_update(&t, id, 0.10f, (uint32_t)e);
  }
  *settled = t.settled;
  (void)calib_sequence_s4_evaluate(&t, CALIB_S4_STEADY_ERROR_RATIO,
                                   CALIB_S4_SAT_DUTY_MAX, CALIB_S4_SPREAD_RATIO_MAX,
                                   CALIB_S4_EXIT_RATIO_MAX, CALIB_S4_EXIT_RUN_MAX, out);
}

static void test_s4_clean(void)
{
  calib_s4_tracker_t t;
  calib_s4_result_t r;
  uint16_t e;
  uint8_t ok;
  calib_sequence_s4_begin(&t, 0.06f, 0.010f, 0.55f, 0.02f, 5U,
                          CALIB_S4_SETTLE_WINDOW_CYCLES, 5U, 8U);
  for (e = 1U; e <= 80U; e++)
  {
    (void)calib_sequence_s4_update(&t, 0.06f * (1.0f - expf(-(float)e / 3.333f)),
                                   0.10f, (uint32_t)e);
  }
  ok = calib_sequence_s4_evaluate(&t, 0.02f, 0.10f, CALIB_S4_SPREAD_RATIO_MAX,
                                  CALIB_S4_EXIT_RATIO_MAX, CALIB_S4_EXIT_RUN_MAX, &r);
  check(ok && r.settle_time_ms <= 50.0f, "s4 clean exponential passes all criteria");
}

static void test_s4_boot_replay(void)
{
  uint16_t start;
  uint8_t all_pass = 1U;
  uint8_t settled;
  float max_settle = 0.0f, max_spread = 0.0f, max_exit = 0.0f;
  uint16_t max_run = 0U;
  for (start = 0U; start < BOOT_N; start++)
  {
    calib_s4_result_t r;
    run_s4_boot(CALIB_S4_SETTLE_WINDOW_CYCLES, start, 0.0f, 0.0f, 32.0f, &r, &settled);
    if ((settled == 0U) || (r.settle_ok == 0U) || (r.steady_err_ok == 0U) ||
        (r.no_osc_ok == 0U) || (r.spread_ok == 0U) || (r.saturation_ok == 0U))
    {
      all_pass = 0U;
    }
    if (r.settle_time_ms > max_settle) max_settle = r.settle_time_ms;
    if (r.window_spread_ratio > max_spread) max_spread = r.window_spread_ratio;
    if (r.exit_ratio > max_exit) max_exit = r.exit_ratio;
    if (r.max_exit_run > max_run) max_run = r.max_exit_run;
  }
  printf("     [stat] boot replay: max_settle=%.1fms max_spread=%.3f max_exit=%.3f max_run=%u\n",
         (double)max_settle, (double)max_spread, (double)max_exit, (unsigned)max_run);
  check(all_pass && max_settle <= 50.0f,
        "s4 W=16 boot correlated-noise replay 64/64 pass within 5*Tc");
}

static void test_s4_w8_cannot_settle(void)
{
  /* 对照：实测相关噪声下 W=8 无法稳定进入 2% 带（这正是选 W=16 的证据）。 */
  uint16_t start;
  uint16_t settled_count = 0U;
  for (start = 0U; start < BOOT_N; start++)
  {
    calib_s4_result_t r;
    uint8_t settled;
    run_s4_boot(8U, start, 0.0f, 0.0f, 32.0f, &r, &settled);
    if (settled != 0U) settled_count++;
  }
  printf("     [stat] W=8 settled %u/64 (expect 0)\n", (unsigned)settled_count);
  check(settled_count == 0U, "s4 W=8 cannot enter 2% band under measured correlated noise");
}

static void test_s4_fast_oscillation(void)
{
  calib_s4_result_t r;
  uint8_t settled;
  run_s4_boot(CALIB_S4_SETTLE_WINDOW_CYCLES, 0U, 0.15f, 0.0f, 32.0f, &r, &settled);
  printf("     [stat] fast +-15%% osc: spread_ratio=%.3f\n",
         (double)r.window_spread_ratio);
  check(r.spread_ok == 0U, "s4 fast symmetric oscillation rejected by spread gate");
}

static void test_s4_slow_oscillation(void)
{
  calib_s4_result_t r5, r3;
  uint8_t s5, s3;
  run_s4_boot(CALIB_S4_SETTLE_WINDOW_CYCLES, 0U, 0.0f, 0.05f, 32.0f, &r5, &s5);
  run_s4_boot(CALIB_S4_SETTLE_WINDOW_CYCLES, 0U, 0.0f, 0.03f, 48.0f, &r3, &s3);
  printf("     [stat] slow osc: 5%%/32ms exit=%.3f run=%u ; 3%%/48ms exit=%.3f run=%u\n",
         (double)r5.exit_ratio, (unsigned)r5.max_exit_run,
         (double)r3.exit_ratio, (unsigned)r3.max_exit_run);
  check(r5.no_osc_ok == 0U && r3.no_osc_ok == 0U,
        "s4 slow limit-cycle rejected by exit run/ratio");
}

int main(void)
{
  g_fail = 0;
  boot_prepare();
  test_step_clean();
  test_step_fr1_residual();
  test_step_window_beats_legacy();
  test_step_offset_nogo();
  test_step_spread_gate();
  test_s4_clean();
  test_s4_boot_replay();
  test_s4_w8_cannot_settle();
  test_s4_fast_oscillation();
  test_s4_slow_oscillation();
  if (g_fail == 0)
  {
    printf("\nALL TESTS PASSED\n");
    return 0;
  }
  printf("\n%d TEST(S) FAILED\n", g_fail);
  return 1;
}
