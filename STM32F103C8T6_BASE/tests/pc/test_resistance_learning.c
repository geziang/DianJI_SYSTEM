/*
 * 相电阻 R 自学习/自裁决 PC 端单元测试（SPEC-RUN FR-1.5）。不依赖任何 HAL 头。
 *
 * 覆盖行为：中值/MAD 工具、单次可信门、滑窗历史、跨启动重复门、
 * 双源交叉门（含无 S7 跳过语义）、失控护栏、effective_r 兜底、
 * load/store 往返、reset 清空。
 *
 * 编译运行（在 STM32F103C8T6_BASE 目录下）：
 *   gcc tests/pc/test_resistance_learning.c Core/Src/resistance_learning.c -I Core/Inc -lm -o tests/pc/t9
 *   ./tests/pc/t9            (Linux/macOS)
 *   tests\pc\t9.exe          (Windows)
 * 全部用例通过返回 0，否则返回非 0。
 */
#include <stdio.h>
#include <math.h>

#include "resistance_learning.h"

static int g_failures = 0;

static void check_int(const char *name, long got, long expect)
{
  if (got == expect)
  {
    printf("[PASS] %-26s got=%ld expect=%ld\n", name, got, expect);
  }
  else
  {
    printf("[FAIL] %-26s got=%ld expect=%ld\n", name, got, expect);
    ++g_failures;
  }
}

static void check_close(const char *name, float got, float expect, float tol)
{
  float err = fabsf(got - expect);
  if (err <= tol)
  {
    printf("[PASS] %-26s got=%.6f expect=%.6f\n", name, got, expect);
  }
  else
  {
    printf("[FAIL] %-26s got=%.6f expect=%.6f err=%.6f\n", name, got, expect, err);
    ++g_failures;
  }
}

static void check_true(const char *name, int cond)
{
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

int main(void)
{
  resistance_learning_cfg_t cfg;
  resistance_learning_state_t st;
  uint8_t flags;

  cfg.history_n = 3U;
  cfg.repeat_dev_max = 0.10f;
  cfg.cross_dev_max = 0.25f;
  cfg.guard_ratio = 2.0f;
  cfg.nominal_r_ohm = 2.3f;

  /* 1. 工具：中值与 MAD/中值。 */
  check_close("util.median3", resistance_learning_median3(2.0f, 2.2f, 2.1f), 2.1f, 1e-6f);
  check_close("util.mad_ratio",
              resistance_learning_mad_ratio3(2.0f, 2.0f, 2.2f, 1.8f), 0.1f, 1e-6f);

  /* 2. 单次可信门：s2_trusted=0 一律不采纳。 */
  resistance_learning_reset(&st);
  flags = resistance_learning_update(&cfg, &st, 2.0f, 0U, 2.0f, 1U);
  check_int("gate1.no_trust_flags", (long)flags, 0L);
  check_int("gate1.history", (long)st.history_count, 0L);

  /* 3. 历史累积 + 样本不足：两次可信更新 -> ADOPTED|REPEAT_FAIL。 */
  resistance_learning_reset(&st);
  flags = resistance_learning_update(&cfg, &st, 2.00f, 1U, 0.0f, 0U);
  check_int("hist2.first_flags",
            (long)(flags & (RESISTANCE_LEARN_F_ADOPTED | RESISTANCE_LEARN_F_REPEAT_FAIL)),
            (long)(RESISTANCE_LEARN_F_ADOPTED | RESISTANCE_LEARN_F_REPEAT_FAIL));
  flags = resistance_learning_update(&cfg, &st, 2.10f, 1U, 0.0f, 0U);
  check_int("hist2.count", (long)st.history_count, 2L);
  check_int("hist2.still_repeat_fail",
            (long)(flags & RESISTANCE_LEARN_F_REPEAT_FAIL),
            (long)RESISTANCE_LEARN_F_REPEAT_FAIL);

  /* 4. 样本够但无 S7：交叉门标记 CROSS_FAIL，learned 不形成。 */
  flags = resistance_learning_update(&cfg, &st, 2.05f, 1U, 0.0f, 0U);
  check_int("cross3.no_s7_flags",
            (long)(flags & RESISTANCE_LEARN_F_CROSS_FAIL),
            (long)RESISTANCE_LEARN_F_CROSS_FAIL);
  check_int("cross3.learned_invalid", (long)st.learned_valid, 0L);

  /* 5. 第四次带一致 S7：滑窗丢最旧，中值通过 -> LEARNED 形成。 */
  flags = resistance_learning_update(&cfg, &st, 2.04f, 1U, 2.06f, 1U);
  check_int("learn4.flags",
            (long)(flags & RESISTANCE_LEARN_F_LEARNED),
            (long)RESISTANCE_LEARN_F_LEARNED);
  check_int("learn4.count_capped", (long)st.history_count, 3L);
  check_close("learn4.median", st.learned_r_ohm, 2.05f, 1e-6f);
  check_int("learn4.revision", (long)st.learned_revision, 1L);
  check_close("learn4.history_latest", st.history_r_ohm[2], 2.04f, 1e-6f);

  /* 6. 双源交叉门失败：S7 与 S2 偏差 25% 以上 -> CROSS_FAIL，learned 不变。 */
  flags = resistance_learning_update(&cfg, &st, 2.00f, 1U, 3.00f, 1U);
  check_int("cross6.fail",
            (long)(flags & RESISTANCE_LEARN_F_CROSS_FAIL),
            (long)RESISTANCE_LEARN_F_CROSS_FAIL);
  check_close("cross6.learned_unchanged", st.learned_r_ohm, 2.05f, 1e-6f);

  /* 7. 跨启动重复门失败：离散大 -> REPEAT_FAIL。 */
  resistance_learning_reset(&st);
  (void)resistance_learning_update(&cfg, &st, 2.00f, 1U, 0.0f, 0U);
  (void)resistance_learning_update(&cfg, &st, 2.50f, 1U, 0.0f, 0U);
  flags = resistance_learning_update(&cfg, &st, 3.00f, 1U, 3.05f, 1U);
  check_int("repeat7.fail",
            (long)(flags & RESISTANCE_LEARN_F_REPEAT_FAIL),
            (long)RESISTANCE_LEARN_F_REPEAT_FAIL);
  check_int("repeat7.learned_invalid", (long)st.learned_valid, 0L);

  /* 8. 失控护栏：历史一致但远离标称（>2 倍）-> GUARD_FAIL，不采纳。 */
  resistance_learning_reset(&st);
  (void)resistance_learning_update(&cfg, &st, 6.00f, 1U, 0.0f, 0U);
  (void)resistance_learning_update(&cfg, &st, 6.10f, 1U, 0.0f, 0U);
  flags = resistance_learning_update(&cfg, &st, 6.05f, 1U, 6.04f, 1U);
  check_int("guard8.fail",
            (long)(flags & RESISTANCE_LEARN_F_GUARD_FAIL),
            (long)RESISTANCE_LEARN_F_GUARD_FAIL);
  check_int("guard8.learned_invalid", (long)st.learned_valid, 0L);

  /* 9. effective_r：learned 可用且在护栏内取 learned；否则取 fallback。 */
  resistance_learning_reset(&st);
  st.learned_valid = 1U;
  st.learned_r_ohm = 2.5f; /* 2.3*2.0=4.6 内 */
  check_close("eff9.learned",
              resistance_learning_effective_r(&cfg, &st, 9.9f), 2.5f, 1e-6f);
  st.learned_r_ohm = 9.9f; /* 越护栏 */
  check_close("eff9.guard_fallback",
              resistance_learning_effective_r(&cfg, &st, 2.3f), 2.3f, 1e-6f);
  st.learned_valid = 0U;
  check_close("eff9.invalid_fallback",
              resistance_learning_effective_r(&cfg, &st, 2.2f), 2.2f, 1e-6f);

  /* 10. store -> load 往返。 */
  {
    resistance_learning_state_t back;
    uint8_t hcount = 0U, lvalid = 0U;
    float hist[3];
    float lr = 0.0f;
    uint32_t lrev = 0UL;

    resistance_learning_reset(&st);
    (void)resistance_learning_update(&cfg, &st, 2.00f, 1U, 0.0f, 0U);
    (void)resistance_learning_update(&cfg, &st, 2.04f, 1U, 0.0f, 0U);
    (void)resistance_learning_update(&cfg, &st, 2.05f, 1U, 2.06f, 1U);
    resistance_learning_store(&st, &hcount, hist, &lvalid, &lr, &lrev);
    resistance_learning_reset(&back);
    resistance_learning_load(&back, hcount, hist, lvalid, lr, lrev);
    check_int("rt10.count", (long)back.history_count, (long)st.history_count);
    check_close("rt10.hist0", back.history_r_ohm[0], st.history_r_ohm[0], 1e-6f);
    check_close("rt10.hist2", back.history_r_ohm[2], st.history_r_ohm[2], 1e-6f);
    check_int("rt10.learned_valid", (long)back.learned_valid, (long)st.learned_valid);
    check_close("rt10.learned_r", back.learned_r_ohm, st.learned_r_ohm, 1e-6f);
    check_int("rt10.revision", (long)back.learned_revision, (long)st.learned_revision);
  }

  /* 11. reset 清空一切。 */
  resistance_learning_reset(&st);
  check_int("reset11.count", (long)st.history_count, 0L);
  check_int("reset11.learned_valid", (long)st.learned_valid, 0L);
  check_int("reset11.revision", (long)st.learned_revision, 0L);
  check_true("reset11.learned_r_zero", st.learned_r_ohm == 0.0f);

  if (g_failures == 0)
  {
    printf("\nALL TESTS PASSED\n");
    return 0;
  }
  printf("\n%d TEST(S) FAILED\n", g_failures);
  return 1;
}
