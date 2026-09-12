/*
 * 电机参数 Flash 后端（HAL，SPEC-RUN FR-2.2）。
 *
 * 只做"页级擦除 + 半字编程 + 读回逐字比对"三件事，不认识参数包语义；
 * 双页择优/掉电安全时序由 motor_parameter_store 编排：
 *   擦"非活跃页" -> 写新包 -> 读回校验通过后新页才被视为可择优（=标记活跃），
 *   任意时刻掉电至少一页完整可用（FR-2.3）。
 *
 * 约束：F1 擦写会 stall 内核，仅允许在 SAFE_IDLE、motor_drv_disable 且
 * PWM 强制零之后调用；写 Flash 期间控制环路不得运行。
 */
#ifndef MOTOR_FLASH_STORE_H
#define MOTOR_FLASH_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  MOTOR_FLASH_STORE_OK = 0,
  MOTOR_FLASH_STORE_INVALID_ARG,
  MOTOR_FLASH_STORE_ERASE_ERROR,
  MOTOR_FLASH_STORE_PROGRAM_ERROR,
  MOTOR_FLASH_STORE_VERIFY_ERROR
} motor_flash_store_status_t;

/* 擦除一个参数页（1 KB）。address 必须是 board_config 定义的页 A/B 基址。 */
motor_flash_store_status_t motor_flash_store_erase_page(uint32_t address);

/* 把 src 起始的 byte_count 字节以半字编程写入页（从 address 开始），
 * 写完后逐半字读回比对，任一不等返回 VERIFY_ERROR（旧副本在他页，不受影响）。 */
motor_flash_store_status_t motor_flash_store_write_page(uint32_t address,
                                                        const uint8_t *src,
                                                        uint32_t byte_count);

/* 读回比对：页内 byte_count 字节与 src 逐半字比对（用于外部复验）。 */
motor_flash_store_status_t motor_flash_store_verify_page(uint32_t address,
                                                         const uint8_t *src,
                                                         uint32_t byte_count);

const char *motor_flash_store_status_text(motor_flash_store_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_FLASH_STORE_H */
