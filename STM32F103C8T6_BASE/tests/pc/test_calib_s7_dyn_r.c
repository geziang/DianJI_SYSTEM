/*
 * S7 低速动态 R 块统计 PC 端单元测试（SPEC-RUN FR-1.2）。不依赖任何 HAL/STM32 头。
 *
 * 覆盖行为：建立段跳过、|Iq| 纳入门、分块 ratio-of-means、空块丢弃、
 * 块缓冲满封顶、块数>=5 截尾 1 / 不足全均值、无块 finalize 失败。
 *
 * 编译运行（在 STM32F103C8T6_BASE 目录下）：
 *   gcc tests/pc/test_calib_s7_dyn_r.c Core/Src/calib_stats.c -I Core/Inc -lm -o tests/pc/t7
 *   ./tests/pc/t7            (Linux/macOS)
 *   tests\pc\t7.exe          (Windows)
 * 全部用例通过返回 0，否则返回非 0。
 */
#include <stdio.h>
#include <math.h>

#include "calib_stats.h"

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

static void feed_block(calib_s7_dyn_r_accum_t *acc,
                       const calib_s7_dyn_r_cfg_t *cfg,
                       float vq_v, float iq_a, uint16_t calls)
{
  uint16_t i;
  for (i = 0U; i < calls; ++i)
  {
    calib_s7_dyn_r_feed(acc, cfg, vq_v, iq_a);
  }
}

int main(void)
{
  calib_s7_dyn_r_cfg_t cfg;
  calib_s7_dyn_r_accum_t acc;
  float r = 0.0f;

  cfg.skip_ms = 5U;      /* 前 5 拍为建立段 */
  cfg.block_ms = 4U;     /* 每 4 拍结算一块 */
  cfg.max_blocks = 24U;
  cfg.min_iq_a = 0.09f;  /* S7 工作点 0.18A 的一半 */

  /* 1. 建立段跳过：前 5 拍喂大信号不应计入任何样本。 */
  calib_s7_dyn_r_init(&acc, &cfg);
  feed_block(&acc, &cfg, 9.9f, 9.9f, 5U);
  check_int("settle.skip_samples", (long)acc.sample_count, 0L);
  check_int("settle.skip_blocks", (long)acc.block_count, 0L);

  /* 2. |Iq| 门：建立段后喂低于门限的样本不计入，但块计时照走。 */
  feed_block(&acc, &cfg, 0.4f, 0.05f, 8U); /* 2 个块周期，全部低于门 */
  check_int("gate.low_iq_samples", (long)acc.sample_count, 0L);
  check_int("gate.low_iq_blocks", (long)acc.block_count, 0L); /* 空块丢弃 */

  /* 3. ratio-of-means：Vq=0.4V、Iq=0.2A -> 每块 R=2.0。 */
  calib_s7_dyn_r_init(&acc, &cfg);
  feed_block(&acc, &cfg, 0.4f, 0.2f, 4U);   /* 块 1 */
  feed_block(&acc, &cfg, 0.4f, 0.2f, 4U);   /* 块 2 */
  check_int("ratio.block_count", (long)acc.block_count, 2L);
  check_int("ratio.sample_count", (long)acc.sample_count, 8L);
  check_close("ratio.block0", acc.block_mean_r[0], 2.0f, 1e-5f);
  check_close("ratio.block1", acc.block_mean_r[1], 2.0f, 1e-5f);
  check_int("ratio.finalize_ok", (long)calib_s7_dyn_r_finalize(&acc, &r), 1L);
  check_close("ratio.finalize_r", r, 2.0f, 1e-5f); /* 2 块 <5，全均值无截尾 */

  /* 4. 均值之比（非逐样本比）：块内 Vq 混合但均值比不变。
   * 块内两拍 (0.3V,0.15A)+(0.5V,0.25A) -> sum(V)/sum(I)=0.8/0.4=2.0。 */
  calib_s7_dyn_r_init(&acc, &cfg);
  {
    uint16_t i;
    for (i = 0U; i < 2U; ++i)
    {
      calib_s7_dyn_r_feed(&acc, &cfg, 0.3f, 0.15f);
      calib_s7_dyn_r_feed(&acc, &cfg, 0.5f, 0.25f);
    }
  }
  check_close("ratio.mixed_block", acc.block_mean_r[0], 2.0f, 1e-5f);

  /* 5. 截尾均值：6 块 [2,2,2,2,2,6] -> 丢两端各 1 -> 均值 2.0（不截尾则为 2.667）。 */
  calib_s7_dyn_r_init(&acc, &cfg);
  feed_block(&acc, &cfg, 0.40f, 0.20f, 4U);
  feed_block(&acc, &cfg, 0.40f, 0.20f, 4U);
  feed_block(&acc, &cfg, 0.40f, 0.20f, 4U);
  feed_block(&acc, &cfg, 0.40f, 0.20f, 4U);
  feed_block(&acc, &cfg, 0.40f, 0.20f, 4U);
  feed_block(&acc, &cfg, 1.20f, 0.20f, 4U); /* 离群块 R=6.0 */
  check_int("trim.block_count", (long)acc.block_count, 6L);
  check_int("trim.finalize_ok", (long)calib_s7_dyn_r_finalize(&acc, &r), 1L);
  check_close("trim.finalize_r", r, 2.0f, 1e-4f);

  /* 6. 缓冲满封顶：max_blocks=3，喂 5 块只留前 3 块，样本计数继续涨。 */
  calib_s7_dyn_r_init(&acc, &cfg);
  cfg.max_blocks = 3U;
  feed_block(&acc, &cfg, 0.4f, 0.2f, 4U);
  feed_block(&acc, &cfg, 0.4f, 0.2f, 4U);
  feed_block(&acc, &cfg, 0.4f, 0.2f, 4U);
  feed_block(&acc, &cfg, 0.4f, 0.2f, 4U);
  feed_block(&acc, &cfg, 0.4f, 0.2f, 4U);
  check_int("cap.block_count", (long)acc.block_count, 3L);
  check_int("cap.sample_count", (long)acc.sample_count, 20L);
  cfg.max_blocks = 24U; /* 恢复，避免影响后续用例 */

  /* 7. 负 Iq 同样过门：S7 反向驱动时 ratio 仍为正（Vq 也为负）。 */
  calib_s7_dyn_r_init(&acc, &cfg);
  feed_block(&acc, &cfg, -0.4f, -0.2f, 4U);
  check_close("gate.negative_iq_r", acc.block_mean_r[0], 2.0f, 1e-5f);

  /* 8. 无块 finalize 返回失败。 */
  calib_s7_dyn_r_init(&acc, &cfg);
  check_int("empty.finalize_fail", (long)calib_s7_dyn_r_finalize(&acc, &r), 0L);

  if (g_failures == 0)
  {
    printf("\nALL TESTS PASSED\n");
    return 0;
  }
  printf("\n%d TEST(S) FAILED\n", g_failures);
  return 1;
}
