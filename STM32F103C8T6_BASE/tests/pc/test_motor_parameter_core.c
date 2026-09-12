/*
 * 电机参数包纯逻辑核心 PC 端单元测试（SPEC-RUN FR-2.1/2.3/附录 A）。
 * 不依赖任何 HAL 头。
 *
 * 覆盖行为：CRC32 覆盖范围、五道校验（版本/ID/CRC/范围/状态）、
 * v1->v2 迁移、双页择优四用例（双新取新/一坏取好/相等取 A/双坏取 none）。
 *
 * 编译运行（在 STM32F103C8T6_BASE 目录下）：
 *   gcc tests/pc/test_motor_parameter_core.c Core/Src/motor_parameter_core.c -I Core/Inc -lm -o tests/pc/t8
 *   ./tests/pc/t8            (Linux/macOS)
 *   tests\pc\t8.exe          (Windows)
 * 全部用例通过返回 0，否则返回非 0。
 */
#include <stdio.h>
#include <string.h>

#include "motor_parameter_core.h"

static int g_failures = 0;

static void check_int(const char *name, long got, long expect)
{
  if (got == expect)
  {
    printf("[PASS] %-28s got=%ld expect=%ld\n", name, got, expect);
  }
  else
  {
    printf("[FAIL] %-28s got=%ld expect=%ld\n", name, got, expect);
    ++g_failures;
  }
}

static const uint32_t HW_ID = 0x1234ABCDUL;
static const uint32_t MOTOR_ID = 0x5678EF01UL;
static const uint32_t WIRING_REV = 7UL;
static const float MAX_BUS_V = 10.0f;

/* 构造一份合法 v2 包（调用者随后可破坏单字段再重算 CRC）。 */
static void fill_valid(motor_parameter_package_t *p, uint32_t revision)
{
  memset(p, 0, sizeof(*p));
  p->format_version = MOTOR_PARAMETER_FORMAT_VERSION_2;
  p->hardware_id = HW_ID;
  p->motor_id = MOTOR_ID;
  p->wiring_revision = WIRING_REV;
  p->parameter_revision = revision;
  p->current_gain_u_a_per_raw = 0.001f;
  p->current_gain_v_a_per_raw = 0.001f;
  p->current_sign_u = 1;
  p->current_sign_v = -1;
  p->phase_resistance_ohm = 2.3f;
  p->phase_inductance_h = 500e-6f;
  p->pole_pairs = 7U;
  p->encoder_direction = 1;
  p->electrical_offset_rad = 0.5f;
  p->id_kp = 2.0f;
  p->id_ki = 0.23f;
  p->iq_kp = 2.0f;
  p->iq_ki = 0.23f;
  p->current_limit_a = 1.0f;
  p->voltage_limit_v = 6.0f;
  p->state = MOTOR_PARAMETER_STATE_COMMITTED;
  p->phase_map_a = 0U;
  p->phase_map_b = 1U;
  p->phase_map_c = 2U;
  p->phase_seq_verified = 1U;
  p->learned_r_valid = 1U;
  p->learned_r_count = 3U;
  p->learned_phase_resistance_ohm = 2.31f;
  p->learned_r_history_ohm[0] = 2.30f;
  p->learned_r_history_ohm[1] = 2.31f;
  p->learned_r_history_ohm[2] = 2.32f;
  p->learned_r_revision = 4UL;
  p->crc = motor_parameter_core_calculate_crc(p);
}

int main(void)
{
  motor_parameter_package_t p;
  motor_parameter_package_t a;
  motor_parameter_package_t b;
  motor_parameter_package_t out;

  /* 1. 合法 v2 包通过校验；CRC 随内容变化（敏感性）。 */
  fill_valid(&p, 1UL);
  check_int("valid.ok",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_OK);
  {
    motor_parameter_package_t p2 = p;
    p2.phase_resistance_ohm = 2.4f;
    p2.crc = motor_parameter_core_calculate_crc(&p2);
    if (p2.crc == p.crc)
    {
      printf("[FAIL] %-28s\n", "crc.sensitivity");
      ++g_failures;
    }
    else
    {
      printf("[PASS] %-28s\n", "crc.sensitivity");
    }
  }

  /* 2. CRC 错误：篡改字节不重算 CRC。 */
  p.phase_resistance_ohm = 9.9f; /* 内容变，crc 未更新 */
  check_int("valid.crc_error",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_CRC_ERROR);
  fill_valid(&p, 1UL);

  /* 3. 版本错误。 */
  p.format_version = 99UL;
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.format_error",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_FORMAT_ERROR);
  fill_valid(&p, 1UL);

  /* 4. ID 失配（硬件/电机/接线任一）。 */
  p.hardware_id = 0xDEADUL;
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.id_mismatch",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_ID_MISMATCH);
  fill_valid(&p, 1UL);

  /* 5. 范围错误：R<=0 / 换相重复 / 换相越界 / 电流超 5A / 电压超母线 / 极对为 0。 */
  p.phase_resistance_ohm = 0.0f;
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.r_zero",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_RANGE_ERROR);
  fill_valid(&p, 1UL);

  p.phase_map_b = 0U; /* 与 a 重复 */
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.phase_dup",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_RANGE_ERROR);
  fill_valid(&p, 1UL);

  p.phase_map_c = 5U; /* 越界 */
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.phase_range",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_RANGE_ERROR);
  fill_valid(&p, 1UL);

  p.current_limit_a = 6.0f;
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.current_over",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_RANGE_ERROR);
  fill_valid(&p, 1UL);

  p.voltage_limit_v = 12.0f; /* 超 max_bus 10V（V3P 硬约束） */
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.voltage_over",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_RANGE_ERROR);
  fill_valid(&p, 1UL);

  p.pole_pairs = 0U;
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.pole_zero",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_RANGE_ERROR);
  fill_valid(&p, 1UL);

  /* 6. 状态错误：DYNAMIC_CANDIDATE 不允许作为加载包。 */
  p.state = MOTOR_PARAMETER_STATE_DYNAMIC_CANDIDATE;
  p.crc = motor_parameter_core_calculate_crc(&p);
  check_int("valid.state_error",
            (long)motor_parameter_core_validate(&p, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
            (long)MOTOR_PARAMETER_CORE_STATE_ERROR);
  fill_valid(&p, 1UL);

  /* 7. v1->v2 迁移：恒等换相、未验证相序、学习值清零，迁移后仍过校验。 */
  {
    motor_parameter_package_t v1;
    fill_valid(&v1, 1UL);
    v1.format_version = MOTOR_PARAMETER_FORMAT_VERSION_1;
    v1.crc = motor_parameter_core_calculate_crc(&v1);
    motor_parameter_core_migrate_v1(&v1);
    check_int("migr.version", (long)v1.format_version,
              (long)MOTOR_PARAMETER_FORMAT_VERSION_2);
    check_int("migr.identity_a", (long)v1.phase_map_a, 0L);
    check_int("migr.identity_b", (long)v1.phase_map_b, 1L);
    check_int("migr.identity_c", (long)v1.phase_map_c, 2L);
    check_int("migr.seq_unverified", (long)v1.phase_seq_verified, 0L);
    check_int("migr.learned_cleared", (long)v1.learned_r_valid, 0L);
    check_int("migr.learned_count", (long)v1.learned_r_count, 0L);
    v1.crc = motor_parameter_core_calculate_crc(&v1);
    check_int("migr.still_valid",
              (long)motor_parameter_core_validate(&v1, HW_ID, MOTOR_ID, WIRING_REV, MAX_BUS_V),
              (long)MOTOR_PARAMETER_CORE_OK);
  }

  /* 8. 双页择优（FR-2.3）四用例。 */
  fill_valid(&a, 5UL);
  fill_valid(&b, 9UL);

  /* 8a. 双有效取 revision 新者 -> B。 */
  check_int("select.both_newer_b",
            (long)motor_parameter_core_select_page(&a, 1U, &b, 1U, &out),
            (long)MOTOR_PARAMETER_PAGE_SELECT_B);
  check_int("select.both_newer_b_rev", (long)out.parameter_revision, 9L);

  /* 8b. A 更新 -> A。 */
  fill_valid(&a, 11UL);
  check_int("select.both_newer_a",
            (long)motor_parameter_core_select_page(&a, 1U, &b, 1U, &out),
            (long)MOTOR_PARAMETER_PAGE_SELECT_A);
  check_int("select.both_newer_a_rev", (long)out.parameter_revision, 11L);

  /* 8c. 相等取 A（确定性回退）。 */
  fill_valid(&a, 9UL);
  check_int("select.equal_takes_a",
            (long)motor_parameter_core_select_page(&a, 1U, &b, 1U, &out),
            (long)MOTOR_PARAMETER_PAGE_SELECT_A);

  /* 8d. 一坏取好：A 坏 -> B；B 坏 -> A；双坏 -> none。 */
  check_int("select.a_bad",
            (long)motor_parameter_core_select_page(&a, 0U, &b, 1U, &out),
            (long)MOTOR_PARAMETER_PAGE_SELECT_B);
  check_int("select.b_bad",
            (long)motor_parameter_core_select_page(&a, 1U, &b, 0U, &out),
            (long)MOTOR_PARAMETER_PAGE_SELECT_A);
  check_int("select.both_bad",
            (long)motor_parameter_core_select_page(&a, 0U, &b, 0U, &out),
            (long)MOTOR_PARAMETER_PAGE_SELECT_NONE);
  /* out 为空指针时仅返回选择结果，不崩溃。 */
  check_int("select.null_out",
            (long)motor_parameter_core_select_page(&a, 1U, &b, 1U, (motor_parameter_package_t *)0),
            (long)MOTOR_PARAMETER_PAGE_SELECT_A);

  if (g_failures == 0)
  {
    printf("\nALL TESTS PASSED\n");
    return 0;
  }
  printf("\n%d TEST(S) FAILED\n", g_failures);
  return 1;
}
