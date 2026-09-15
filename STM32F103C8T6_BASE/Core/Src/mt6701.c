/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mt6701.c
  * @brief   MT6701 I2C encoder driver based on STM32 HAL.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mt6701.h"

#include "board_config.h"
#include <stddef.h>

#define MT6701_I2C_ADDRESS_8BIT              (MT6701_DEFAULT_ADDRESS_7BIT << 1)
#define MT6701_I2C_TIMEOUT_MS                (2U)

#define MT6701_REG_ANGLE_HIGH                (0x03U)
#define MT6701_REG_ANGLE_LOW                 (0x04U)
#define MT6701_REG_DIR                       (0x29U)
#define MT6701_REG_ZERO_HIGH                 (0x32U)
#define MT6701_REG_ZERO_LOW                  (0x33U)

#define MT6701_REG_ANGLE_LOW_POS             (2U)
#define MT6701_REG_DIR_POS                   (1U)
#define MT6701_REG_ZERO_HIGH_POS             (0U)

#define MT6701_REG_DIR_MASK                  (0x01U << MT6701_REG_DIR_POS)
#define MT6701_REG_ZERO_HIGH_MASK            (0x0FU << MT6701_REG_ZERO_HIGH_POS)

/* 最近一次成功读角捕获的磁场状态位（ANGLE_LOW bit[1:0]，见头文件探针说明）。 */
static uint8_t mt6701_last_field_status = 0U;

static mt6701_status_t mt6701_select_register(uint8_t reg)
{
  uint8_t data;
  HAL_StatusTypeDef hal_status;

  data = reg;
  hal_status = HAL_I2C_Master_Transmit(board_config_get_encoder_i2c(),
                                       MT6701_I2C_ADDRESS_8BIT,
                                       &data,
                                       1U,
                                       MT6701_I2C_TIMEOUT_MS);

  return (hal_status == HAL_OK) ? MT6701_STATUS_OK : MT6701_STATUS_I2C_TX_ERROR;
}

static mt6701_status_t mt6701_read_byte(uint8_t reg, uint8_t *value)
{
  HAL_StatusTypeDef hal_status;
  mt6701_status_t status;

  if (value == NULL)
  {
    return MT6701_STATUS_INVALID_ARG;
  }

  status = mt6701_select_register(reg);
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  hal_status = HAL_I2C_Master_Receive(board_config_get_encoder_i2c(),
                                      MT6701_I2C_ADDRESS_8BIT,
                                      value,
                                      1U,
                                      MT6701_I2C_TIMEOUT_MS);

  return (hal_status == HAL_OK) ? MT6701_STATUS_OK : MT6701_STATUS_I2C_RX_ERROR;
}

static mt6701_status_t mt6701_write_byte(uint8_t reg, uint8_t value)
{
  uint8_t data[2];
  HAL_StatusTypeDef hal_status;

  data[0] = reg;
  data[1] = value;

  hal_status = HAL_I2C_Master_Transmit(board_config_get_encoder_i2c(),
                                       MT6701_I2C_ADDRESS_8BIT,
                                       data,
                                       2U,
                                       MT6701_I2C_TIMEOUT_MS);

  return (hal_status == HAL_OK) ? MT6701_STATUS_OK : MT6701_STATUS_I2C_TX_ERROR;
}

mt6701_status_t mt6701_is_connected(void)
{
  HAL_StatusTypeDef hal_status;

  hal_status = HAL_I2C_IsDeviceReady(board_config_get_encoder_i2c(),
                                     MT6701_I2C_ADDRESS_8BIT,
                                     2U,
                                     MT6701_I2C_TIMEOUT_MS);

  return (hal_status == HAL_OK) ? MT6701_STATUS_OK : MT6701_STATUS_I2C_MEM_ERROR;
}

mt6701_status_t mt6701_read_raw_angle(uint16_t *angle_raw)
{
  uint8_t angle_high;
  uint8_t angle_high2;
  uint8_t angle_low;
  uint8_t diff;
  mt6701_status_t status;

  if (angle_raw == NULL)
  {
    return MT6701_STATUS_INVALID_ARG;
  }

  /* 高-低-高三读一致性：一个样本拆两个 I2C 事务存在撕裂/位错窗口
   * （HAL OK ≠ 数据对，MT6701 I2C 无校验和；2026-09-15 上板实锤：
   * 静止电机被读出持续 -2000rpm = 成段垃圾数据流经速度环引发 overspeed
   * 误跳并驱动 PI 输出虚假转矩指令）。两个高字节不一致（±1 LSB 容差
   * 防真实转动的边界抖动）则整拍作废。 */
  status = mt6701_read_byte(MT6701_REG_ANGLE_HIGH, &angle_high);
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  status = mt6701_read_byte(MT6701_REG_ANGLE_LOW, &angle_low);
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  status = mt6701_read_byte(MT6701_REG_ANGLE_HIGH, &angle_high2);
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  diff = (uint8_t)(angle_high - angle_high2);
  if ((diff > 1U) && (diff < 255U))
  {
    return MT6701_STATUS_I2C_RX_ERROR; /* 撕裂/位错样本：按坏读处理 */
  }
  /* 磁场状态探针：ANGLE_LOW bit[1:0]（01=正常）——此前被 >>2 丢弃。 */
  mt6701_last_field_status = (uint8_t)(angle_low & MT6701_FIELD_STATUS_MASK);
  /* 一致时取第二次高字节（更接近低字节时刻）。 */
  *angle_raw = (uint16_t)((((uint16_t)angle_high2) << 6) |
                          (((uint16_t)angle_low) >> MT6701_REG_ANGLE_LOW_POS));
  *angle_raw &= MT6701_RAW_MASK_14BIT;

  return MT6701_STATUS_OK;
}

uint8_t mt6701_get_last_field_status(void)
{
  return mt6701_last_field_status;
}

mt6701_status_t mt6701_read_angle_degrees_x100(uint16_t *angle_degrees_x100)
{
  uint16_t angle_raw;
  uint32_t scaled;
  mt6701_status_t status;

  if (angle_degrees_x100 == NULL)
  {
    return MT6701_STATUS_INVALID_ARG;
  }

  status = mt6701_read_raw_angle(&angle_raw);
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  scaled = ((uint32_t)angle_raw * 36000UL) / MT6701_RAW_FULL_SCALE;
  *angle_degrees_x100 = (uint16_t)scaled;

  return MT6701_STATUS_OK;
}

mt6701_status_t mt6701_read_snapshot(mt6701_snapshot_t *snapshot)
{
  mt6701_status_t status;

  if (snapshot == NULL)
  {
    return MT6701_STATUS_INVALID_ARG;
  }

  status = mt6701_read_raw_angle(&snapshot->raw_angle);
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  snapshot->angle_degrees_x100 =
      (uint16_t)(((uint32_t)snapshot->raw_angle * 36000UL) / MT6701_RAW_FULL_SCALE);
  snapshot->field_status = MT6701_FIELD_STATUS_UNKNOWN;

  return MT6701_STATUS_OK;
}

mt6701_status_t mt6701_set_zero_position(uint16_t value)
{
  uint8_t data;
  mt6701_status_t status;

  if ((value & (uint16_t)(~0x0FFFU)) != 0U)
  {
    return MT6701_STATUS_VALUE_RANGE;
  }

  status = mt6701_write_byte(MT6701_REG_ZERO_LOW, (uint8_t)(value & 0x00FFU));
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  status = mt6701_read_byte(MT6701_REG_ZERO_HIGH, &data);
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  data &= (uint8_t)(~MT6701_REG_ZERO_HIGH_MASK);
  data |= (uint8_t)((value >> 8) << MT6701_REG_ZERO_HIGH_POS);

  return mt6701_write_byte(MT6701_REG_ZERO_HIGH, data);
}

mt6701_status_t mt6701_set_direction(mt6701_direction_t direction)
{
  uint8_t data;
  mt6701_status_t status;

  if ((direction != MT6701_DIRECTION_CW) && (direction != MT6701_DIRECTION_CCW))
  {
    return MT6701_STATUS_INVALID_ARG;
  }

  status = mt6701_read_byte(MT6701_REG_DIR, &data);
  if (status != MT6701_STATUS_OK)
  {
    return status;
  }

  if (direction == MT6701_DIRECTION_CW)
  {
    data &= (uint8_t)(~MT6701_REG_DIR_MASK);
  }
  else
  {
    data |= MT6701_REG_DIR_MASK;
  }

  return mt6701_write_byte(MT6701_REG_DIR, data);
}

const char *mt6701_status_text(mt6701_status_t status)
{
  switch (status)
  {
    case MT6701_STATUS_OK:
      return "OK";
    case MT6701_STATUS_INVALID_ARG:
      return "INVALID_ARG";
    case MT6701_STATUS_I2C_TX_ERROR:
      return "I2C_TX_ERROR";
    case MT6701_STATUS_I2C_RX_ERROR:
      return "I2C_RX_ERROR";
    case MT6701_STATUS_I2C_MEM_ERROR:
      return "I2C_MEM_ERROR";
    case MT6701_STATUS_VALUE_RANGE:
      return "VALUE_RANGE";
    default:
      return "UNKNOWN";
  }
}

const char *mt6701_field_status_text(mt6701_field_status_t status)
{
  switch (status)
  {
    case MT6701_FIELD_STATUS_NORMAL:
      return "normal";
    case MT6701_FIELD_STATUS_STRONG:
      return "strong";
    case MT6701_FIELD_STATUS_WEAK:
      return "weak";
    case MT6701_FIELD_STATUS_ERROR:
      return "error";
    case MT6701_FIELD_STATUS_UNKNOWN:
    default:
      return "unknown";
  }
}
