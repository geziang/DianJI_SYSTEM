/*
 * 电机参数存储：RAM 活动包/候选包管理 + Flash 双页 A/B 后端编排（SPEC-RUN R2）。
 *
 * 纯逻辑（CRC/校验/择优）在 motor_parameter_core，页擦写在 motor_flash_store，
 * 本模块只做编排与 RAM 状态：
 *  - init：上电影子加载——直读两页 magic+包，validate 判每页有效性，择优拷入
 *    RAM active（FR-2.3/FR-2.4）；
 *  - commit：擦"非活跃页"->写新包(含读回比对)->RAM active 切新值。旧页全程
 *    不动，任意时刻掉电至少一页完整可用；新页 revision 恒为两页最大值+1。
 */
#include "motor_parameter_store.h"

#include "board_config.h"
#include "motor_flash_store.h"
#include <stddef.h>

/* 页内镜像：magic + 参数包。半字对齐，总尺寸受 FR-2.1 静态断言约束。 */
#define MOTOR_PARAMETER_PAGE_MAGIC (0x50415231UL) /* "PAR1" */

typedef struct
{
  uint32_t magic;
  motor_parameter_package_t package;
} motor_parameter_page_image_t;

typedef char motor_parameter_page_size_check[
    (sizeof(motor_parameter_page_image_t) <= BOARD_CONFIG_FLASH_PAGE_SIZE) ? 1 : -1];

static motor_parameter_package_t motor_parameter_active;
static motor_parameter_package_t motor_parameter_candidate;
static uint8_t motor_parameter_active_valid;
static uint8_t motor_parameter_candidate_valid;
static motor_parameter_page_select_t motor_parameter_source_page =
    MOTOR_PARAMETER_PAGE_SELECT_NONE;
/* 两页各自最近一次读到的 revision（init 时刷新，commit 后同步更新）。 */
static uint32_t motor_parameter_page_revision[2];

static motor_parameter_store_status_t motor_parameter_map_core_status(
    motor_parameter_core_status_t status)
{
  switch (status)
  {
    case MOTOR_PARAMETER_CORE_OK: return MOTOR_PARAMETER_STORE_OK;
    case MOTOR_PARAMETER_CORE_INVALID_ARG: return MOTOR_PARAMETER_STORE_INVALID_ARG;
    case MOTOR_PARAMETER_CORE_FORMAT_ERROR: return MOTOR_PARAMETER_STORE_FORMAT_ERROR;
    case MOTOR_PARAMETER_CORE_CRC_ERROR: return MOTOR_PARAMETER_STORE_CRC_ERROR;
    case MOTOR_PARAMETER_CORE_ID_MISMATCH: return MOTOR_PARAMETER_STORE_ID_MISMATCH;
    case MOTOR_PARAMETER_CORE_RANGE_ERROR: return MOTOR_PARAMETER_STORE_RANGE_ERROR;
    case MOTOR_PARAMETER_CORE_STATE_ERROR: return MOTOR_PARAMETER_STORE_STATE_ERROR;
    default: return MOTOR_PARAMETER_STORE_FORMAT_ERROR;
  }
}

/* 读取一页：magic 匹配且包过五道校验才算有效（v1 包就地迁移为 v2 内存副本）。 */
static uint8_t motor_parameter_read_page(uint32_t address,
                                         motor_parameter_package_t *out,
                                         uint32_t *revision_out)
{
  const motor_parameter_page_image_t *image;

  if ((out == NULL) || (revision_out == NULL))
  {
    return 0U;
  }
  image = (const motor_parameter_page_image_t *)(uintptr_t)address;
  if (image->magic != MOTOR_PARAMETER_PAGE_MAGIC)
  {
    return 0U;
  }
  if (motor_parameter_core_validate(&image->package,
                                    BOARD_CONFIG_HARDWARE_ID,
                                    BOARD_CONFIG_MOTOR_ID,
                                    BOARD_CONFIG_WIRING_REVISION,
                                    BOARD_CONFIG_MAX_BUS_VOLTAGE_V) != MOTOR_PARAMETER_CORE_OK)
  {
    return 0U;
  }
  *out = image->package;
  if (out->format_version == MOTOR_PARAMETER_FORMAT_VERSION_1)
  {
    motor_parameter_core_migrate_v1(out);
    out->crc = motor_parameter_core_calculate_crc(out);
  }
  *revision_out = out->parameter_revision;
  return 1U;
}

void motor_parameter_store_init(void)
{
  motor_parameter_package_t page_a;
  motor_parameter_package_t page_b;
  uint8_t a_valid;
  uint8_t b_valid;
  motor_parameter_package_t chosen;

  motor_parameter_active_valid = 0U;
  motor_parameter_candidate_valid = 0U;
  motor_parameter_source_page = MOTOR_PARAMETER_PAGE_SELECT_NONE;
  motor_parameter_page_revision[0] = 0UL;
  motor_parameter_page_revision[1] = 0UL;

  a_valid = motor_parameter_read_page(BOARD_CONFIG_FLASH_PAGE_A_ADDR, &page_a,
                                      &motor_parameter_page_revision[0]);
  b_valid = motor_parameter_read_page(BOARD_CONFIG_FLASH_PAGE_B_ADDR, &page_b,
                                      &motor_parameter_page_revision[1]);
  motor_parameter_source_page = motor_parameter_core_select_page(
      a_valid != 0U ? &page_a : NULL, a_valid,
      b_valid != 0U ? &page_b : NULL, b_valid, &chosen);
  if (motor_parameter_source_page != MOTOR_PARAMETER_PAGE_SELECT_NONE)
  {
    motor_parameter_active = chosen;
    motor_parameter_active_valid = 1U;
  }
}

motor_parameter_store_status_t motor_parameter_store_validate(const motor_parameter_package_t *package)
{
  return motor_parameter_map_core_status(
      motor_parameter_core_validate(package,
                                    BOARD_CONFIG_HARDWARE_ID,
                                    BOARD_CONFIG_MOTOR_ID,
                                    BOARD_CONFIG_WIRING_REVISION,
                                    BOARD_CONFIG_MAX_BUS_VOLTAGE_V));
}

motor_parameter_store_status_t motor_parameter_store_load(motor_parameter_package_t *package)
{
  if (package == NULL)
  {
    return MOTOR_PARAMETER_STORE_INVALID_ARG;
  }
  if (motor_parameter_active_valid == 0U)
  {
    return MOTOR_PARAMETER_STORE_NO_PACKAGE;
  }
  if (motor_parameter_store_validate(&motor_parameter_active) != MOTOR_PARAMETER_STORE_OK)
  {
    motor_parameter_active_valid = 0U;
    return MOTOR_PARAMETER_STORE_NO_PACKAGE;
  }
  *package = motor_parameter_active;
  return MOTOR_PARAMETER_STORE_OK;
}

motor_parameter_store_status_t motor_parameter_store_stage_candidate(const motor_parameter_package_t *candidate)
{
  if (candidate == NULL)
  {
    return MOTOR_PARAMETER_STORE_INVALID_ARG;
  }
  if (candidate->state != MOTOR_PARAMETER_STATE_DYNAMIC_CANDIDATE)
  {
    return MOTOR_PARAMETER_STORE_STATE_ERROR;
  }
  motor_parameter_candidate = *candidate;
  motor_parameter_candidate_valid = 1U;
  return MOTOR_PARAMETER_STORE_OK;
}

motor_parameter_store_status_t motor_parameter_store_commit_candidate(uint8_t safe_idle)
{
  motor_parameter_package_t committed;
  motor_parameter_store_status_t status;
  uint32_t newest_revision;
  uint32_t target_address;
  uint8_t target_is_b;
  motor_parameter_page_image_t image;
  motor_flash_store_status_t flash_status;

  if (safe_idle == 0U)
  {
    return MOTOR_PARAMETER_STORE_NOT_SAFE_IDLE;
  }
  if (motor_parameter_candidate_valid == 0U)
  {
    return MOTOR_PARAMETER_STORE_NO_PACKAGE;
  }

  committed = motor_parameter_candidate;
  committed.state = MOTOR_PARAMETER_STATE_COMMITTED;
  /* 新 revision = 两页现存最大值 + 1（单调递增，掉电重启后择优方向不变）。 */
  newest_revision = (motor_parameter_page_revision[1] > motor_parameter_page_revision[0])
                        ? motor_parameter_page_revision[1]
                        : motor_parameter_page_revision[0];
  committed.parameter_revision = newest_revision + 1UL;
  committed.format_version = MOTOR_PARAMETER_FORMAT_VERSION_2;
  committed.crc = motor_parameter_core_calculate_crc(&committed);
  status = motor_parameter_store_validate(&committed);
  if (status != MOTOR_PARAMETER_STORE_OK)
  {
    return status;
  }

  /* 目标页 = 非活跃页：当前择优为 B（或仅 B 有效）写 A，否则写 B；
   * 首次写入（两页皆空）固定从 A 开始交替。 */
  target_is_b = (motor_parameter_source_page != MOTOR_PARAMETER_PAGE_SELECT_B) ? 1U : 0U;
  target_address = (target_is_b != 0U) ? BOARD_CONFIG_FLASH_PAGE_B_ADDR
                                       : BOARD_CONFIG_FLASH_PAGE_A_ADDR;

  image.magic = MOTOR_PARAMETER_PAGE_MAGIC;
  image.package = committed;
  flash_status = motor_flash_store_erase_page(target_address);
  if (flash_status != MOTOR_FLASH_STORE_OK)
  {
    return (flash_status == MOTOR_FLASH_STORE_INVALID_ARG)
               ? MOTOR_PARAMETER_STORE_INVALID_ARG
               : MOTOR_PARAMETER_STORE_FLASH_ERASE_ERROR;
  }
  flash_status = motor_flash_store_write_page(target_address, (const uint8_t *)&image,
                                              (uint32_t)sizeof(image));
  if (flash_status == MOTOR_FLASH_STORE_VERIFY_ERROR)
  {
    return MOTOR_PARAMETER_STORE_FLASH_VERIFY_ERROR;
  }
  if (flash_status != MOTOR_FLASH_STORE_OK)
  {
    return MOTOR_PARAMETER_STORE_FLASH_PROGRAM_ERROR;
  }

  /* 读回校验已通过：新页才被记为有效（=标记活跃），旧页保留为回滚副本。 */
  motor_parameter_page_revision[target_is_b != 0U ? 1U : 0U] = committed.parameter_revision;
  motor_parameter_active = committed;
  motor_parameter_active_valid = 1U;
  motor_parameter_candidate_valid = 0U;
  motor_parameter_source_page = (target_is_b != 0U) ? MOTOR_PARAMETER_PAGE_SELECT_B
                                                    : MOTOR_PARAMETER_PAGE_SELECT_A;
  return MOTOR_PARAMETER_STORE_OK;
}

motor_parameter_store_status_t motor_parameter_store_activate_ram_candidate(void)
{
  motor_parameter_package_t activated;

  if (motor_parameter_candidate_valid == 0U)
  {
    return MOTOR_PARAMETER_STORE_NO_PACKAGE;
  }
  activated = motor_parameter_candidate;
  activated.state = MOTOR_PARAMETER_STATE_COMMITTED;
  activated.crc = motor_parameter_core_calculate_crc(&activated);
  if (motor_parameter_store_validate(&activated) != MOTOR_PARAMETER_STORE_OK)
  {
    return MOTOR_PARAMETER_STORE_STATE_ERROR;
  }
  motor_parameter_active = activated;
  motor_parameter_active_valid = 1U;
  /* RAM 激活不改变 Flash 两页内容与择优来源。 */
  return MOTOR_PARAMETER_STORE_OK;
}

motor_parameter_store_status_t motor_parameter_store_clear_storage(uint8_t safe_idle)
{
  motor_flash_store_status_t flash_status;

  if (safe_idle == 0U)
  {
    return MOTOR_PARAMETER_STORE_NOT_SAFE_IDLE;
  }
  flash_status = motor_flash_store_erase_page(BOARD_CONFIG_FLASH_PAGE_A_ADDR);
  if (flash_status != MOTOR_FLASH_STORE_OK)
  {
    return (flash_status == MOTOR_FLASH_STORE_INVALID_ARG)
               ? MOTOR_PARAMETER_STORE_INVALID_ARG
               : MOTOR_PARAMETER_STORE_FLASH_ERASE_ERROR;
  }
  flash_status = motor_flash_store_erase_page(BOARD_CONFIG_FLASH_PAGE_B_ADDR);
  if (flash_status != MOTOR_FLASH_STORE_OK)
  {
    return (flash_status == MOTOR_FLASH_STORE_INVALID_ARG)
               ? MOTOR_PARAMETER_STORE_INVALID_ARG
               : MOTOR_PARAMETER_STORE_FLASH_ERASE_ERROR;
  }
  motor_parameter_page_revision[0] = 0UL;
  motor_parameter_page_revision[1] = 0UL;
  motor_parameter_store_invalidate();
  return MOTOR_PARAMETER_STORE_OK;
}

void motor_parameter_store_invalidate(void)
{
  motor_parameter_active_valid = 0U;
  motor_parameter_candidate_valid = 0U;
  motor_parameter_source_page = MOTOR_PARAMETER_PAGE_SELECT_NONE;
}

uint8_t motor_parameter_store_has_active(void)
{
  return motor_parameter_active_valid;
}

const motor_parameter_package_t *motor_parameter_store_get_active(void)
{
  return (motor_parameter_active_valid != 0U) ? &motor_parameter_active : NULL;
}

const motor_parameter_package_t *motor_parameter_store_get_candidate(void)
{
  return (motor_parameter_candidate_valid != 0U) ? &motor_parameter_candidate : NULL;
}

uint32_t motor_parameter_store_calculate_crc(const motor_parameter_package_t *package)
{
  return motor_parameter_core_calculate_crc(package);
}

const char *motor_parameter_store_get_source_page_text(void)
{
  switch (motor_parameter_source_page)
  {
    case MOTOR_PARAMETER_PAGE_SELECT_A: return "A";
    case MOTOR_PARAMETER_PAGE_SELECT_B: return "B";
    default: return "none";
  }
}

uint32_t motor_parameter_store_get_active_page_revision(uint8_t page_b)
{
  return motor_parameter_page_revision[(page_b != 0U) ? 1U : 0U];
}

const char *motor_parameter_store_status_text(motor_parameter_store_status_t status)
{
  switch (status)
  {
    case MOTOR_PARAMETER_STORE_OK: return "OK";
    case MOTOR_PARAMETER_STORE_NO_PACKAGE: return "NO_PACKAGE";
    case MOTOR_PARAMETER_STORE_INVALID_ARG: return "INVALID_ARG";
    case MOTOR_PARAMETER_STORE_FORMAT_ERROR: return "FORMAT_ERROR";
    case MOTOR_PARAMETER_STORE_CRC_ERROR: return "CRC_ERROR";
    case MOTOR_PARAMETER_STORE_ID_MISMATCH: return "ID_MISMATCH";
    case MOTOR_PARAMETER_STORE_RANGE_ERROR: return "RANGE_ERROR";
    case MOTOR_PARAMETER_STORE_STATE_ERROR: return "STATE_ERROR";
    case MOTOR_PARAMETER_STORE_NOT_SAFE_IDLE: return "NOT_SAFE_IDLE";
    case MOTOR_PARAMETER_STORE_FLASH_ERASE_ERROR: return "FLASH_ERASE_ERROR";
    case MOTOR_PARAMETER_STORE_FLASH_PROGRAM_ERROR: return "FLASH_PROGRAM_ERROR";
    case MOTOR_PARAMETER_STORE_FLASH_VERIFY_ERROR: return "FLASH_VERIFY_ERROR";
    default: return "UNKNOWN";
  }
}
