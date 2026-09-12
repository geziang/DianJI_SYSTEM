/*
 * 电机参数 Flash 后端实现。直接使用 HAL FLASH 模块（stm32f1xx_hal_conf.h
 * 已使能 HAL_FLASH_MODULE_ENABLED），不另造寄存器序列（FR-2.2）。
 *
 * F103 页擦除典型 20~40 ms、半字编程数十周期，期间内核 stall：调用方必须
 * 保证控制环路已停、MOTOR_EN 已拉低、PWM 已强制零。
 */
#include "motor_flash_store.h"

#include "board_config.h"
#include "main.h"

/* 编程粒度：F1 支持按半字（16 bit）编程。 */
#define MOTOR_FLASH_STORE_HALFWORD_BYTES (2U)

static motor_flash_store_status_t motor_flash_store_unlock(void)
{
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return MOTOR_FLASH_STORE_ERASE_ERROR;
  }
  return MOTOR_FLASH_STORE_OK;
}

static void motor_flash_store_lock(void)
{
  (void)HAL_FLASH_Lock();
}

motor_flash_store_status_t motor_flash_store_erase_page(uint32_t address)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error = 0U;
  HAL_StatusTypeDef hal_status;

  if ((address != BOARD_CONFIG_FLASH_PAGE_A_ADDR) &&
      (address != BOARD_CONFIG_FLASH_PAGE_B_ADDR))
  {
    return MOTOR_FLASH_STORE_INVALID_ARG;
  }
  if (motor_flash_store_unlock() != MOTOR_FLASH_STORE_OK)
  {
    return MOTOR_FLASH_STORE_ERASE_ERROR;
  }
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = address;
  erase.NbPages = 1U;
  erase.Banks = FLASH_BANK_1;
  hal_status = HAL_FLASHEx_Erase(&erase, &page_error);
  motor_flash_store_lock();
  if (hal_status != HAL_OK)
  {
    return MOTOR_FLASH_STORE_ERASE_ERROR;
  }
  return MOTOR_FLASH_STORE_OK;
}

motor_flash_store_status_t motor_flash_store_write_page(uint32_t address,
                                                        const uint8_t *src,
                                                        uint32_t byte_count)
{
  uint32_t offset;
  HAL_StatusTypeDef hal_status;
  motor_flash_store_status_t verify;

  if ((src == NULL) || ((byte_count % MOTOR_FLASH_STORE_HALFWORD_BYTES) != 0U) ||
      (byte_count > BOARD_CONFIG_FLASH_PAGE_SIZE))
  {
    return MOTOR_FLASH_STORE_INVALID_ARG;
  }
  if (motor_flash_store_unlock() != MOTOR_FLASH_STORE_OK)
  {
    return MOTOR_FLASH_STORE_PROGRAM_ERROR;
  }
  for (offset = 0U; offset < byte_count; offset += MOTOR_FLASH_STORE_HALFWORD_BYTES)
  {
    uint16_t halfword;
    halfword = (uint16_t)((uint16_t)src[offset] |
                          ((uint16_t)src[offset + 1U] << 8));
    hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                   address + offset, (uint64_t)halfword);
    if (hal_status != HAL_OK)
    {
      motor_flash_store_lock();
      return MOTOR_FLASH_STORE_PROGRAM_ERROR;
    }
  }
  motor_flash_store_lock();
  verify = motor_flash_store_verify_page(address, src, byte_count);
  if (verify != MOTOR_FLASH_STORE_OK)
  {
    return MOTOR_FLASH_STORE_VERIFY_ERROR;
  }
  return MOTOR_FLASH_STORE_OK;
}

motor_flash_store_status_t motor_flash_store_verify_page(uint32_t address,
                                                         const uint8_t *src,
                                                         uint32_t byte_count)
{
  uint32_t offset;
  const uint16_t *written;

  if ((src == NULL) || ((byte_count % MOTOR_FLASH_STORE_HALFWORD_BYTES) != 0U) ||
      (byte_count > BOARD_CONFIG_FLASH_PAGE_SIZE))
  {
    return MOTOR_FLASH_STORE_INVALID_ARG;
  }
  written = (const uint16_t *)(uintptr_t)address;
  for (offset = 0U; offset < byte_count; offset += MOTOR_FLASH_STORE_HALFWORD_BYTES)
  {
    uint16_t expect;
    expect = (uint16_t)((uint16_t)src[offset] |
                        ((uint16_t)src[offset + 1U] << 8));
    if (written[offset / MOTOR_FLASH_STORE_HALFWORD_BYTES] != expect)
    {
      return MOTOR_FLASH_STORE_VERIFY_ERROR;
    }
  }
  return MOTOR_FLASH_STORE_OK;
}

const char *motor_flash_store_status_text(motor_flash_store_status_t status)
{
  switch (status)
  {
    case MOTOR_FLASH_STORE_OK: return "OK";
    case MOTOR_FLASH_STORE_INVALID_ARG: return "INVALID_ARG";
    case MOTOR_FLASH_STORE_ERASE_ERROR: return "ERASE_ERROR";
    case MOTOR_FLASH_STORE_PROGRAM_ERROR: return "PROGRAM_ERROR";
    case MOTOR_FLASH_STORE_VERIFY_ERROR: return "VERIFY_ERROR";
    default: return "UNKNOWN";
  }
}
