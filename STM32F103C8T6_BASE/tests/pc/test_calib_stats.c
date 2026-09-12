/*
 * calib_stats PC 端单元测试。不依赖任何 HAL/STM32 头，仅用标准库。
 *
 * 编译运行（在 STM32F103C8T6_BASE 目录下）：
 *   gcc tests/pc/test_calib_stats.c Core/Src/calib_stats.c -I Core/Inc -lm -o tests/pc/t
 *   ./tests/pc/t            (Linux/macOS)
 *   tests\pc\t.exe          (Windows)
 * 全部用例通过返回 0，否则返回非 0。
 */
#include <stdio.h>
#include <math.h>

#include "calib_stats.h"

static int g_failures = 0;

static float deg2rad(float d)
{
  return d * 3.14159265358979f / 180.0f;
}

static void check_close(const char *name, float got, float expect, float tol)
{
  float err = fabsf(got - expect);
  if (err <= tol)
  {
    printf("[PASS] %-22s got=%.6f expect=%.6f\n", name, got, expect);
  }
  else
  {
    printf("[FAIL] %-22s got=%.6f expect=%.6f err=%.6f\n", name, got, expect, err);
    ++g_failures;
  }
}

int main(void)
{
  /* 1. 最小二乘：y = 2x + 0.1，应精确还原 */
  {
    float x[4] = {0.0f, 1.0f, 2.0f, 3.0f};
    float y[4] = {0.1f, 2.1f, 4.1f, 6.1f};
    calib_line_t line = calib_least_squares(x, y, 4);
    check_close("ls.slope", line.slope, 2.0f, 1e-4f);
    check_close("ls.intercept", line.intercept, 0.1f, 1e-4f);
    check_close("ls.r2", line.r2, 1.0f, 1e-4f);
  }

  /* 1b. 样本不足应返回 r2=-1 表无效 */
  {
    float x1[1] = {1.0f}, y1[1] = {2.0f};
    calib_line_t bad = calib_least_squares(x1, y1, 1);
    check_close("ls.invalid_r2", bad.r2, -1.0f, 1e-6f);
  }

  /* 2. 去极值均值：{1,2,3,4,100} 两端各丢 1 -> {2,3,4} = 3 */
  {
    float v[5] = {3.0f, 100.0f, 1.0f, 4.0f, 2.0f};
    check_close("trimmed_mean", calib_trimmed_mean(v, 5, 1), 3.0f, 1e-5f);
  }

  /* 3. 中位数/IQR：{1,2,3,4,5} -> median3, Q1=2, Q3=4, iqr=2 */
  {
    float v[5] = {5.0f, 1.0f, 4.0f, 2.0f, 3.0f};
    float median = 0.0f, iqr = 0.0f;
    calib_median_iqr(v, 5, &median, &iqr);
    check_close("median", median, 3.0f, 1e-5f);
    check_close("iqr", iqr, 2.0f, 1e-5f);
  }

  /* 4. 圆周均值：355deg 与 5deg 应平均到 0 附近（跨 0 点） */
  {
    float a[2] = {deg2rad(355.0f), deg2rad(5.0f)};
    check_close("circ_mean", calib_circular_mean(a, 2), 0.0f, 1e-4f);
  }

  /* 5. 圆周标准差：三个近乎同向的角应接近 0 */
  {
    float a[3] = {0.09f, 0.10f, 0.11f};
    float s = calib_circular_std(a, 3);
    check_close("circ_std", s, 0.0f, 0.01f);
  }

  /* 6. unwrap：179deg -> -179deg 应连续到 181deg */
  {
    float u = calib_angle_unwrap(deg2rad(179.0f), deg2rad(-179.0f));
    check_close("unwrap", u, deg2rad(181.0f), 1e-4f);
  }

  if (g_failures == 0)
  {
    printf("\nALL TESTS PASSED\n");
    return 0;
  }
  printf("\n%d TEST(S) FAILED\n", g_failures);
  return 1;
}
