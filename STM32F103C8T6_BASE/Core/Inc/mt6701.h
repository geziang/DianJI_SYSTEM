/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mt6701.h
  * @brief   MT6701 I2C encoder driver based on STM32 HAL.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MT6701_H
#define MT6701_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MT6701_LIB_PORT_VERSION              "f103-hal-port-0.1"

/* MT6701 default 7-bit I2C address. HAL uses the shifted 8-bit address in .c. */
#define MT6701_DEFAULT_ADDRESS_7BIT          (0x06U)

/* MT6701 I2C angle is 14 bit, 0..16383 maps to 0..360 degrees. */
#define MT6701_RAW_TO_DEGREES_NUMERATOR      (360U)
#define MT6701_RAW_FULL_SCALE                (16384U)
#define MT6701_RAW_MASK_14BIT                (0x3FFFU)

typedef enum
{
  MT6701_STATUS_OK = 0,
  MT6701_STATUS_INVALID_ARG,
  MT6701_STATUS_I2C_TX_ERROR,
  MT6701_STATUS_I2C_RX_ERROR,
  MT6701_STATUS_I2C_MEM_ERROR,
  MT6701_STATUS_VALUE_RANGE
} mt6701_status_t;

typedef enum
{
  MT6701_FIELD_STATUS_UNKNOWN = 0,
  MT6701_FIELD_STATUS_NORMAL,
  MT6701_FIELD_STATUS_STRONG,
  MT6701_FIELD_STATUS_WEAK,
  MT6701_FIELD_STATUS_ERROR
} mt6701_field_status_t;

typedef enum
{
  MT6701_DIRECTION_CW = 0,
  MT6701_DIRECTION_CCW = 1
} mt6701_direction_t;

typedef struct
{
  uint16_t raw_angle;
  uint16_t angle_degrees_x100;
  mt6701_field_status_t field_status;
} mt6701_snapshot_t;

mt6701_status_t mt6701_is_connected(void);
mt6701_status_t mt6701_read_raw_angle(uint16_t *angle_raw);
mt6701_status_t mt6701_read_angle_degrees_x100(uint16_t *angle_degrees_x100);
mt6701_status_t mt6701_read_snapshot(mt6701_snapshot_t *snapshot);

mt6701_status_t mt6701_set_zero_position(uint16_t value);
mt6701_status_t mt6701_set_direction(mt6701_direction_t direction);

const char *mt6701_status_text(mt6701_status_t status);
const char *mt6701_field_status_text(mt6701_field_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* MT6701_H */
