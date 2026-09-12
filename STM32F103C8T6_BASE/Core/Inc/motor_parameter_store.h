#ifndef MOTOR_PARAMETER_STORE_H
#define MOTOR_PARAMETER_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 参数包类型/状态与纯逻辑校验全部下沉到 motor_parameter_core（零 HAL，PC 可测）；
 * 本头文件原样重导出，既有调用方（control_loop/safety_manager）不用改。 */
#include "motor_parameter_core.h"

typedef enum
{
  MOTOR_PARAMETER_STORE_OK = 0,
  MOTOR_PARAMETER_STORE_NO_PACKAGE,
  MOTOR_PARAMETER_STORE_INVALID_ARG,
  MOTOR_PARAMETER_STORE_FORMAT_ERROR,
  MOTOR_PARAMETER_STORE_CRC_ERROR,
  MOTOR_PARAMETER_STORE_ID_MISMATCH,
  MOTOR_PARAMETER_STORE_RANGE_ERROR,
  MOTOR_PARAMETER_STORE_STATE_ERROR,
  MOTOR_PARAMETER_STORE_NOT_SAFE_IDLE,
  /* Flash 后端专用状态码（FR-2.2：任一步失败返回明确状态码） */
  MOTOR_PARAMETER_STORE_FLASH_ERASE_ERROR,
  MOTOR_PARAMETER_STORE_FLASH_PROGRAM_ERROR,
  MOTOR_PARAMETER_STORE_FLASH_VERIFY_ERROR
} motor_parameter_store_status_t;

void motor_parameter_store_init(void);
motor_parameter_store_status_t motor_parameter_store_load(motor_parameter_package_t *package);
motor_parameter_store_status_t motor_parameter_store_validate(const motor_parameter_package_t *package);
motor_parameter_store_status_t motor_parameter_store_stage_candidate(const motor_parameter_package_t *candidate);
/* stage->commit 两阶段。safe_idle 必须为 1（SAFE_IDLE 且功率已撤），内部完成：
 * 擦非活跃页 -> 写新包 -> 读回校验 -> 择优切换；任意时刻掉电至少一页完好。 */
motor_parameter_store_status_t motor_parameter_store_commit_candidate(uint8_t safe_idle);
/* 仅把候选激活到 RAM（本次启动生效，不写 Flash）。相序未验证的候选走这里
 * （FR-1.3：不得进入永久固化），Flash 写失败时的降级路径也走这里。 */
motor_parameter_store_status_t motor_parameter_store_activate_ram_candidate(void);
/* c=clear：失效 RAM 学习值/活动包并擦除两个参数页（要求 SAFE_IDLE），
 * 清除后等同未标定（FR-2.5）。 */
motor_parameter_store_status_t motor_parameter_store_clear_storage(uint8_t safe_idle);
void motor_parameter_store_invalidate(void);
uint8_t motor_parameter_store_has_active(void);
const motor_parameter_package_t *motor_parameter_store_get_active(void);
const motor_parameter_package_t *motor_parameter_store_get_candidate(void);
uint32_t motor_parameter_store_calculate_crc(const motor_parameter_package_t *package);
/* 上电择优结果："A" / "B" / "none"（FR-2.4 日志来源）。 */
const char *motor_parameter_store_get_source_page_text(void);
uint32_t motor_parameter_store_get_active_page_revision(uint8_t page_b);
const char *motor_parameter_store_status_text(motor_parameter_store_status_t status);
const char *motor_parameter_state_text(motor_parameter_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAMETER_STORE_H */
