/**
  ******************************************************************************
  * @file    lsm6dsr_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lsm6dsr_reg.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LSM6DSR_REGS_H
#define LSM6DSR_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup LSM6DSR
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
  *              You can create a sensor configuration by your own or using
  *              Unico / Unicleo tools available on STMicroelectronics
  *              web site.
  *
  * @{
  *
  */

typedef struct
{
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup LSM6DSR_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define LSM6DSR_I2C_ADD_L     0xD5U
#define LSM6DSR_I2C_ADD_H     0xD7U

/** Device Identification (Who am I) **/
#define LSM6DSR_ID            0x6AU //TODO Isn't the one showed

/**
  * @}
  *
  */

#define LSM6DSR_FUNC_CFG_ACCESS              0x01U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 7;
  uint8_t func_cfg_en              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t func_cfg_en              : 1;
  uint8_t not_used_01              : 7;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_func_cfg_access_t;

#define LSM6DSR_SENSOR_SYNC_TIME_FRAME       0x04U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tph                      : 4;
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t tph                      : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensor_sync_time_frame_t;

#define LSM6DSR_SENSOR_SYNC_RES_RATIO        0x05U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t rr                       : 2;
  uint8_t not_used_01              : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t rr                       : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensor_sync_res_ratio_t;

#define LSM6DSR_FIFO_CTRL1                   0x06U
typedef struct
{
  uint8_t fth                      : 8;  /* + FIFO_CTRL2(fth) */
} lsm6dsr_fifo_ctrl1_t;

#define LSM6DSR_FIFO_CTRL2                   0x07U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fth                      : 3;  /* + FIFO_CTRL1(fth) */
  uint8_t fifo_temp_en             : 1;
  uint8_t not_used_01              : 4;
  uint8_t fifo_timer_en            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_timer_en            : 1;
  uint8_t not_used_01              : 4;
  uint8_t fifo_temp_en             : 1;
  uint8_t fth                      : 3;  /* + FIFO_CTRL1(fth) */
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_fifo_ctrl2_t;

#define LSM6DSR_FIFO_CTRL3                   0x08U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t dec_fifo_xl              : 3;
  uint8_t dec_fifo_gyro            : 3;
  uint8_t not_used_01              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t dec_fifo_gyro            : 3;
  uint8_t dec_fifo_xl              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_fifo_ctrl3_t;

#define LSM6DSR_FIFO_CTRL4                   0x09U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t dec_ds3_fifo             : 3;
  uint8_t dec_ds4_fifo             : 3;
  uint8_t only_high_data           : 1;
  uint8_t stop_on_fth              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t stop_on_fth              : 1;
  uint8_t only_high_data           : 1;
  uint8_t dec_ds4_fifo             : 3;
  uint8_t dec_ds3_fifo             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_fifo_ctrl4_t;

#define LSM6DSR_FIFO_CTRL5                   0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_mode                : 3;
  uint8_t odr_fifo                 : 4;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t odr_fifo                 : 4;
  uint8_t fifo_mode                : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_fifo_ctrl5_t;

#define LSM6DSR_DRDY_PULSE_CFG               0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 7;
  uint8_t drdy_pulsed              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t drdy_pulsed              : 1;
  uint8_t not_used_01              : 7;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_drdy_pulse_cfg_t;

#define LSM6DSR_INT1_CTRL                    0x0DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_full_flag           : 1;
  uint8_t not_used_01              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t int1_full_flag           : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_int1_ctrl_t;

#define LSM6DSR_INT2_CTRL                    0x0EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_full_flag           : 1;
  uint8_t not_used_01              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t int2_full_flag           : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_int2_ctrl_t;

#define LSM6DSR_WHO_AM_I                     0x0FU
#define LSM6DSR_CTRL1_XL                     0x10U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bw0_xl                   : 1;
  uint8_t lpf1_bw_sel              : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_xl                   : 4;
  uint8_t fs_xl                    : 2;
  uint8_t lpf1_bw_sel              : 1;
  uint8_t bw0_xl                   : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl1_xl_t;

#define LSM6DSR_CTRL2_G                      0x11U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 3;  /* fs_g + fs_125 */
  uint8_t odr_g                    : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_g                    : 4;
  uint8_t fs_g                     : 3;  /* fs_g + fs_125 */
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl2_g_t;

#define LSM6DSR_CTRL3_C                      0x12U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset                 : 1;
  uint8_t ble                      : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                     : 1;
  uint8_t bdu                      : 1;
  uint8_t h_lactive                : 1;
  uint8_t pp_od                    : 1;
  uint8_t sim                      : 1;
  uint8_t if_inc                   : 1;
  uint8_t ble                      : 1;
  uint8_t sw_reset                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl3_c_t;

#define LSM6DSR_CTRL4_C                      0x13U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t den_drdy_int1            : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t sleep                    : 1;
  uint8_t den_xl_en                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_xl_en                : 1;
  uint8_t sleep                    : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t den_drdy_int1            : 1;
  uint8_t drdy_mask                : 1;
  uint8_t i2c_disable              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl4_c_t;

#define LSM6DSR_CTRL5_C                      0x14U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl                    : 2;
  uint8_t st_g                     : 2;
  uint8_t den_lh                   : 1;
  uint8_t rounding                 : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t rounding                 : 3;
  uint8_t den_lh                   : 1;
  uint8_t st_g                     : 2;
  uint8_t st_xl                    : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl5_c_t;

#define LSM6DSR_CTRL6_C                      0x15U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ftype                    : 2;
  uint8_t not_used_01              : 1;
  uint8_t usr_off_w                : 1;
  uint8_t xl_hm_mode               : 1;
uint8_t den_mode                 :
  3;  /* trig_en + lvl_en + lvl2_en */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
uint8_t den_mode                 :
  3;  /* trig_en + lvl_en + lvl2_en */
  uint8_t xl_hm_mode               : 1;
  uint8_t usr_off_w                : 1;
  uint8_t not_used_01              : 1;
  uint8_t ftype                    : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl6_c_t;

#define LSM6DSR_CTRL7_G                      0x16U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t rounding_status          : 1;
  uint8_t not_used_02              : 1;
  uint8_t hpm_g                    : 2;
  uint8_t hp_en_g                  : 1;
  uint8_t g_hm_mode                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t g_hm_mode                : 1;
  uint8_t hp_en_g                  : 1;
  uint8_t hpm_g                    : 2;
  uint8_t not_used_02              : 1;
  uint8_t rounding_status          : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl7_g_t;

#define LSM6DSR_CTRL8_XL                     0x17U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t low_pass_on_6d           : 1;
  uint8_t not_used_01              : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t input_composite          : 1;
  uint8_t hp_ref_mode              : 1;
  uint8_t hpcf_xl                  : 2;
  uint8_t lpf2_xl_en               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t lpf2_xl_en               : 1;
  uint8_t hpcf_xl                  : 2;
  uint8_t hp_ref_mode              : 1;
  uint8_t input_composite          : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t not_used_01              : 1;
  uint8_t low_pass_on_6d           : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl8_xl_t;

#define LSM6DSR_CTRL9_XL                     0x18U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t soft_en                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t den_xl_g                 : 1;
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_x                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_z                    : 1;
  uint8_t den_xl_g                 : 1;
  uint8_t not_used_02              : 1;
  uint8_t soft_en                  : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl9_xl_t;

#define LSM6DSR_CTRL10_C                     0x19U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t func_en                  : 1;
  uint8_t tilt_en                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t timer_en                 : 1;
  uint8_t not_used_03              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_03              : 2;
  uint8_t timer_en                 : 1;
  uint8_t not_used_02              : 1;
  uint8_t tilt_en                  : 1;
  uint8_t func_en                  : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl10_c_t;

#define LSM6DSR_MASTER_CONFIG                0x1AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t master_on                : 1;
  uint8_t iron_en                  : 1;
  uint8_t pass_through_mode        : 1;
  uint8_t pull_up_en               : 1;
  uint8_t start_config             : 1;
  uint8_t not_used_01              : 1;
  uint8_t  data_valid_sel_fifo     : 1;
  uint8_t drdy_on_int1             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t drdy_on_int1             : 1;
  uint8_t  data_valid_sel_fifo     : 1;
  uint8_t not_used_01              : 1;
  uint8_t start_config             : 1;
  uint8_t pull_up_en               : 1;
  uint8_t pass_through_mode        : 1;
  uint8_t iron_en                  : 1;
  uint8_t master_on                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_master_config_t;

#define LSM6DSR_WAKE_UP_SRC                  0x1BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_state_ia           : 1;
  uint8_t ff_ia                    : 1;
  uint8_t not_used_01              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t ff_ia                    : 1;
  uint8_t sleep_state_ia           : 1;
  uint8_t wu_ia                    : 1;
  uint8_t x_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t z_wu                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_wake_up_src_t;

#define LSM6DSR_TAP_SRC                      0x1CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t tap_ia                   : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t tap_ia                   : 1;
  uint8_t single_tap               : 1;
  uint8_t double_tap               : 1;
  uint8_t tap_sign                 : 1;
  uint8_t x_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t z_tap                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_tap_src_t;

#define LSM6DSR_D6D_SRC                      0x1DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                       : 1;
  uint8_t xh                       : 1;
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t den_drdy                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_drdy                 : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t zh                       : 1;
  uint8_t zl                       : 1;
  uint8_t yh                       : 1;
  uint8_t yl                       : 1;
  uint8_t xh                       : 1;
  uint8_t xl                       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_d6d_src_t;

#define LSM6DSR_STATUS_REG                   0x1EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t tda                      : 1;
  uint8_t gda                      : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_status_reg_t;

#define LSM6DSR_STATUS_SPIAUX                0x1EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t gyro_settling            : 1;
  uint8_t not_used_01              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t gyro_settling            : 1;
  uint8_t gda                      : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_status_spiaux_t;

#define LSM6DSR_OUT_TEMP_L                   0x20U
#define LSM6DSR_OUT_TEMP_H                   0x21U
#define LSM6DSR_OUTX_L_G                     0x22U
#define LSM6DSR_OUTX_H_G                     0x23U
#define LSM6DSR_OUTY_L_G                     0x24U
#define LSM6DSR_OUTY_H_G                     0x25U
#define LSM6DSR_OUTZ_L_G                     0x26U
#define LSM6DSR_OUTZ_H_G                     0x27U
#define LSM6DSR_OUTX_L_XL                    0x28U
#define LSM6DSR_OUTX_H_XL                    0x29U
#define LSM6DSR_OUTY_L_XL                    0x2AU
#define LSM6DSR_OUTY_H_XL                    0x2BU
#define LSM6DSR_OUTZ_L_XL                    0x2CU
#define LSM6DSR_OUTZ_H_XL                    0x2DU
#define LSM6DSR_SENSORHUB1_REG               0x2EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub1_reg_t;

#define LSM6DSR_SENSORHUB2_REG               0x2FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub2_reg_t;

#define LSM6DSR_SENSORHUB3_REG               0x30U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub3_reg_t;

#define LSM6DSR_SENSORHUB4_REG               0x31U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub4_reg_t;

#define LSM6DSR_SENSORHUB5_REG               0x32U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub5_reg_t;

#define LSM6DSR_SENSORHUB6_REG               0x33U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub6_reg_t;

#define LSM6DSR_SENSORHUB7_REG               0x34U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub7_reg_t;

#define LSM6DSR_SENSORHUB8_REG               0x35U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub8_reg_t;

#define LSM6DSR_SENSORHUB9_REG               0x36U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub9_reg_t;

#define LSM6DSR_SENSORHUB10_REG              0x37U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub10_reg_t;

#define LSM6DSR_SENSORHUB11_REG              0x38U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub11_reg_t;

#define LSM6DSR_SENSORHUB12_REG              0x39U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub12_reg_t;

#define LSM6DSR_FIFO_STATUS1                 0x3AU
typedef struct
{
  uint8_t diff_fifo                : 8;  /* + FIFO_STATUS2(diff_fifo) */
} lsm6dsr_fifo_status1_t;

#define LSM6DSR_FIFO_STATUS2                 0x3BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff_fifo                : 3;  /* + FIFO_STATUS1(diff_fifo) */
  uint8_t not_used_01              : 1;
  uint8_t fifo_empty               : 1;
  uint8_t fifo_full_smart          : 1;
  uint8_t over_run                 : 1;
  uint8_t waterm                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t waterm                   : 1;
  uint8_t over_run                 : 1;
  uint8_t fifo_full_smart          : 1;
  uint8_t fifo_empty               : 1;
  uint8_t not_used_01              : 1;
  uint8_t diff_fifo                : 3;  /* + FIFO_STATUS1(diff_fifo) */
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_fifo_status2_t;

#define LSM6DSR_FIFO_STATUS3                 0x3CU
typedef struct
{
uint8_t fifo_pattern             :
  8;  /* + FIFO_STATUS4(fifo_pattern) */
} lsm6dsr_fifo_status3_t;

#define LSM6DSR_FIFO_STATUS4                 0x3DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
uint8_t fifo_pattern             :
  2;  /* + FIFO_STATUS3(fifo_pattern) */
  uint8_t not_used_01              : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 6;
uint8_t fifo_pattern             :
  2;  /* + FIFO_STATUS3(fifo_pattern) */
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_fifo_status4_t;

#define LSM6DSR_FIFO_DATA_OUT_L              0x3E
#define LSM6DSR_FIFO_DATA_OUT_H              0x3F
#define LSM6DSR_TIMESTAMP0_REG               0x40
#define LSM6DSR_TIMESTAMP1_REG               0x41
#define LSM6DSR_TIMESTAMP2_REG               0x42

#define LSM6DSR_SENSORHUB13_REG              0x4DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub13_reg_t;

#define LSM6DSR_SENSORHUB14_REG              0x4EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub14_reg_t;

#define LSM6DSR_SENSORHUB15_REG              0x4FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub15_reg_t;

#define LSM6DSR_SENSORHUB16_REG              0x50U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub16_reg_t;

#define LSM6DSR_SENSORHUB17_REG              0x51U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub17_reg_t;

#define LSM6DSR_SENSORHUB18_REG              0x52U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_sensorhub18_reg_t;

#define LSM6DSR_FUNC_SRC1                    0x53U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sensorhub_end_op         : 1;
  uint8_t si_end_op                : 1;
  uint8_t hi_fail                  : 1;
  uint8_t not_used_01              : 2;
  uint8_t tilt_ia                  : 1;
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t tilt_ia                  : 1;
  uint8_t not_used_01              : 2;
  uint8_t hi_fail                  : 1;
  uint8_t si_end_op                : 1;
  uint8_t sensorhub_end_op         : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_func_src1_t;

#define LSM6DSR_FUNC_SRC2                    0x54U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t slave0_nack              : 1;
  uint8_t slave1_nack              : 1;
  uint8_t slave2_nack              : 1;
  uint8_t slave3_nack              : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t slave3_nack              : 1;
  uint8_t slave2_nack              : 1;
  uint8_t slave1_nack              : 1;
  uint8_t slave0_nack              : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_func_src2_t;

#define LSM6DSR_TAP_CFG                      0x58U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lir                      : 1;
  uint8_t tap_z_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t slope_fds                : 1;
  uint8_t inact_en                 : 2;
  uint8_t interrupts_enable        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t interrupts_enable        : 1;
  uint8_t inact_en                 : 2;
  uint8_t slope_fds                : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_z_en                 : 1;
  uint8_t lir                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_tap_cfg_t;

#define LSM6DSR_TAP_THS_6D                   0x59U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths                  : 5;
  uint8_t sixd_ths                 : 2;
  uint8_t d4d_en                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t d4d_en                   : 1;
  uint8_t sixd_ths                 : 2;
  uint8_t tap_ths                  : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_tap_ths_6d_t;

#define LSM6DSR_INT_DUR2                     0x5AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t shock                    : 2;
  uint8_t quiet                    : 2;
  uint8_t dur                      : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dur                      : 4;
  uint8_t quiet                    : 2;
  uint8_t shock                    : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_int_dur2_t;

#define LSM6DSR_WAKE_UP_THS                  0x5BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wk_ths                   : 6;
  uint8_t not_used_01              : 1;
  uint8_t single_double_tap        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t single_double_tap        : 1;
  uint8_t not_used_01              : 1;
  uint8_t wk_ths                   : 6;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_wake_up_ths_t;

#define LSM6DSR_WAKE_UP_DUR                  0x5CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur                : 4;
  uint8_t timer_hr                 : 1;
  uint8_t wake_dur                 : 2;
  uint8_t ff_dur                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                   : 1;
  uint8_t wake_dur                 : 2;
  uint8_t timer_hr                 : 1;
  uint8_t sleep_dur                : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_wake_up_dur_t;

#define LSM6DSR_FREE_FALL                    0x5DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths                   : 3;
  uint8_t ff_dur                   : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                   : 5;
  uint8_t ff_ths                   : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_free_fall_t;

#define LSM6DSR_MD1_CFG                      0x5EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_timer               : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_inact_state         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_inact_state         : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_timer               : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_md1_cfg_t;

#define LSM6DSR_MD2_CFG                      0x5FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_iron                : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_inact_state         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_inact_state         : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_iron                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_md2_cfg_t;

#define LSM6DSR_MASTER_CMD_CODE              0x60U
typedef struct
{
  uint8_t master_cmd_code          : 8;
} lsm6dsr_master_cmd_code_t;

#define LSM6DSR_SENS_SYNC_SPI_ERROR_CODE     0x61U
typedef struct
{
  uint8_t error_code               : 8;
} lsm6dsr_sens_sync_spi_error_code_t;

#define LSM6DSR_OUT_MAG_RAW_X_L              0x66U
#define LSM6DSR_OUT_MAG_RAW_X_H              0x67U
#define LSM6DSR_OUT_MAG_RAW_Y_L              0x68U
#define LSM6DSR_OUT_MAG_RAW_Y_H              0x69U
#define LSM6DSR_OUT_MAG_RAW_Z_L              0x6AU
#define LSM6DSR_OUT_MAG_RAW_Z_H              0x6BU
#define LSM6DSR_INT_OIS                      0x6FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t lvl2_ois                 : 1;
  uint8_t int2_drdy_ois            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_drdy_ois            : 1;
  uint8_t lvl2_ois                 : 1;
  uint8_t not_used_01              : 6;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_int_ois_t;

#define LSM6DSR_CTRL1_OIS                    0x70U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ois_en_spi2              : 1;
  uint8_t fs_g_ois                 : 3;  /* fs_g_ois + fs_125_ois */
  uint8_t mode4_en                 : 1;
  uint8_t sim_ois                  : 1;
  uint8_t lvl1_ois                 : 1;
  uint8_t ble_ois                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ble_ois                  : 1;
  uint8_t lvl1_ois                 : 1;
  uint8_t sim_ois                  : 1;
  uint8_t mode4_en                 : 1;
  uint8_t fs_g_ois                 : 3;  /* fs_g_ois + fs_125_ois */
  uint8_t ois_en_spi2              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl1_ois_t;

#define LSM6DSR_CTRL2_OIS                    0x71U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t hp_en_ois                : 1;
  uint8_t ftype_ois                : 2;
  uint8_t not_used_01              : 1;
  uint8_t hpm_ois                  : 2;
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t hpm_ois                  : 2;
  uint8_t not_used_01              : 1;
  uint8_t ftype_ois                : 2;
  uint8_t hp_en_ois                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl2_ois_t;

#define LSM6DSR_CTRL3_OIS                    0x72U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_ois_clampdis          : 1;
  uint8_t st_ois                   : 2;
  uint8_t filter_xl_conf_ois       : 2;
  uint8_t fs_xl_ois                : 2;
  uint8_t den_lh_ois               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_lh_ois               : 1;
  uint8_t fs_xl_ois                : 2;
  uint8_t filter_xl_conf_ois       : 2;
  uint8_t st_ois                   : 2;
  uint8_t st_ois_clampdis          : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_ctrl3_ois_t;

#define LSM6DSR_X_OFS_USR                    0x73U
#define LSM6DSR_Y_OFS_USR                    0x74U
#define LSM6DSR_Z_OFS_USR                    0x75U
#define LSM6DSR_SLV0_ADD                     0x02U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t rw_0                     : 1;
  uint8_t slave0_add               : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave0_add               : 7;
  uint8_t rw_0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_slv0_add_t;

#define LSM6DSR_SLV0_SUBADD                  0x03U
typedef struct
{
  uint8_t slave0_reg               : 8;
} lsm6dsr_slv0_subadd_t;

#define LSM6DSR_SLAVE0_CONFIG                0x04U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave0_numop             : 3;
  uint8_t src_mode                 : 1;
  uint8_t aux_sens_on              : 2;
  uint8_t slave0_rate              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave0_rate              : 2;
  uint8_t aux_sens_on              : 2;
  uint8_t src_mode                 : 1;
  uint8_t slave0_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_slave0_config_t;

#define LSM6DSR_SLV1_ADD                     0x05U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_1                      : 1;
  uint8_t slave1_add               : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave1_add               : 7;
  uint8_t r_1                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_slv1_add_t;

#define LSM6DSR_SLV1_SUBADD                  0x06U
typedef struct
{
  uint8_t slave1_reg               : 8;
} lsm6dsr_slv1_subadd_t;

#define LSM6DSR_SLAVE1_CONFIG                0x07U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave1_numop             : 3;
  uint8_t not_used_01              : 2;
  uint8_t write_once               : 1;
  uint8_t slave1_rate              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave1_rate              : 2;
  uint8_t write_once               : 1;
  uint8_t not_used_01              : 2;
  uint8_t slave1_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_slave1_config_t;

#define LSM6DSR_SLV2_ADD                     0x08U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_2                      : 1;
  uint8_t slave2_add               : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave2_add               : 7;
  uint8_t r_2                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_slv2_add_t;

#define LSM6DSR_SLV2_SUBADD                  0x09U
typedef struct
{
  uint8_t slave2_reg               : 8;
} lsm6dsr_slv2_subadd_t;

#define LSM6DSR_SLAVE2_CONFIG                0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave2_numop             : 3;
  uint8_t not_used_01              : 3;
  uint8_t slave2_rate              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave2_rate              : 2;
  uint8_t not_used_01              : 3;
  uint8_t slave2_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_slave2_config_t;

#define LSM6DSR_SLV3_ADD                     0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_3                      : 1;
  uint8_t slave3_add               : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave3_add               : 7;
  uint8_t r_3                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_slv3_add_t;

#define LSM6DSR_SLV3_SUBADD                  0x0CU
typedef struct
{
  uint8_t slave3_reg               : 8;
} lsm6dsr_slv3_subadd_t;

#define LSM6DSR_SLAVE3_CONFIG                0x0DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave3_numop             : 3;
  uint8_t not_used_01              : 3;
  uint8_t slave3_rate              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave3_rate              : 2;
  uint8_t not_used_01              : 3;
  uint8_t slave3_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsr_slave3_config_t;

#define LSM6DSR_DATAWRITE_SRC_MODE_SUB_SLV0  0x0EU
typedef struct
{
  uint8_t slave_dataw              : 8;
} lsm6dsr_datawrite_src_mode_sub_slv0_t;

#define LSM6DSR_MAG_SI_XX                    0x24U
#define LSM6DSR_MAG_SI_XY                    0x25U
#define LSM6DSR_MAG_SI_XZ                    0x26U
#define LSM6DSR_MAG_SI_YX                    0x27U
#define LSM6DSR_MAG_SI_YY                    0x28U
#define LSM6DSR_MAG_SI_YZ                    0x29U
#define LSM6DSR_MAG_SI_ZX                    0x2AU
#define LSM6DSR_MAG_SI_ZY                    0x2BU
#define LSM6DSR_MAG_SI_ZZ                    0x2CU
#define LSM6DSR_MAG_OFFX_L                   0x2DU
#define LSM6DSR_MAG_OFFX_H                   0x2EU
#define LSM6DSR_MAG_OFFY_L                   0x2FU
#define LSM6DSR_MAG_OFFY_H                   0x30U
#define LSM6DSR_MAG_OFFZ_L                   0x31U
#define LSM6DSR_MAG_OFFZ_H                   0x32U

/**
  * @defgroup LSM6DSR_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union
{
  lsm6dsr_func_cfg_access_t                  func_cfg_access;
  lsm6dsr_sensor_sync_time_frame_t           sensor_sync_time_frame;
  lsm6dsr_sensor_sync_res_ratio_t            sensor_sync_res_ratio;
  lsm6dsr_fifo_ctrl1_t                       fifo_ctrl1;
  lsm6dsr_fifo_ctrl2_t                       fifo_ctrl2;
  lsm6dsr_fifo_ctrl3_t                       fifo_ctrl3;
  lsm6dsr_fifo_ctrl4_t                       fifo_ctrl4;
  lsm6dsr_fifo_ctrl5_t                       fifo_ctrl5;
  lsm6dsr_drdy_pulse_cfg_t                   drdy_pulse_cfg;
  lsm6dsr_int1_ctrl_t                        int1_ctrl;
  lsm6dsr_int2_ctrl_t                        int2_ctrl;
  lsm6dsr_ctrl1_xl_t                         ctrl1_xl;
  lsm6dsr_ctrl2_g_t                          ctrl2_g;
  lsm6dsr_ctrl3_c_t                          ctrl3_c;
  lsm6dsr_ctrl4_c_t                          ctrl4_c;
  lsm6dsr_ctrl5_c_t                          ctrl5_c;
  lsm6dsr_ctrl6_c_t                          ctrl6_c;
  lsm6dsr_ctrl7_g_t                          ctrl7_g;
  lsm6dsr_ctrl8_xl_t                         ctrl8_xl;
  lsm6dsr_ctrl9_xl_t                         ctrl9_xl;
  lsm6dsr_ctrl10_c_t                         ctrl10_c;
  lsm6dsr_master_config_t                    master_config;
  lsm6dsr_wake_up_src_t                      wake_up_src;
  lsm6dsr_tap_src_t                          tap_src;
  lsm6dsr_d6d_src_t                          d6d_src;
  lsm6dsr_status_reg_t                       status_reg;
  lsm6dsr_status_spiaux_t                    status_spiaux;
  lsm6dsr_sensorhub1_reg_t                   sensorhub1_reg;
  lsm6dsr_sensorhub2_reg_t                   sensorhub2_reg;
  lsm6dsr_sensorhub3_reg_t                   sensorhub3_reg;
  lsm6dsr_sensorhub4_reg_t                   sensorhub4_reg;
  lsm6dsr_sensorhub5_reg_t                   sensorhub5_reg;
  lsm6dsr_sensorhub6_reg_t                   sensorhub6_reg;
  lsm6dsr_sensorhub7_reg_t                   sensorhub7_reg;
  lsm6dsr_sensorhub8_reg_t                   sensorhub8_reg;
  lsm6dsr_sensorhub9_reg_t                   sensorhub9_reg;
  lsm6dsr_sensorhub10_reg_t                  sensorhub10_reg;
  lsm6dsr_sensorhub11_reg_t                  sensorhub11_reg;
  lsm6dsr_sensorhub12_reg_t                  sensorhub12_reg;
  lsm6dsr_fifo_status1_t                     fifo_status1;
  lsm6dsr_fifo_status2_t                     fifo_status2;
  lsm6dsr_fifo_status3_t                     fifo_status3;
  lsm6dsr_fifo_status4_t                     fifo_status4;
  lsm6dsr_sensorhub13_reg_t                  sensorhub13_reg;
  lsm6dsr_sensorhub14_reg_t                  sensorhub14_reg;
  lsm6dsr_sensorhub15_reg_t                  sensorhub15_reg;
  lsm6dsr_sensorhub16_reg_t                  sensorhub16_reg;
  lsm6dsr_sensorhub17_reg_t                  sensorhub17_reg;
  lsm6dsr_sensorhub18_reg_t                  sensorhub18_reg;
  lsm6dsr_func_src1_t                        func_src1;
  lsm6dsr_func_src2_t                        func_src2;
  lsm6dsr_tap_cfg_t                          tap_cfg;
  lsm6dsr_tap_ths_6d_t                       tap_ths_6d;
  lsm6dsr_int_dur2_t                         int_dur2;
  lsm6dsr_wake_up_ths_t                      wake_up_ths;
  lsm6dsr_wake_up_dur_t                      wake_up_dur;
  lsm6dsr_free_fall_t                        free_fall;
  lsm6dsr_md1_cfg_t                          md1_cfg;
  lsm6dsr_md2_cfg_t                          md2_cfg;
  lsm6dsr_master_cmd_code_t                  master_cmd_code;
  lsm6dsr_sens_sync_spi_error_code_t         sens_sync_spi_error_code;
  lsm6dsr_int_ois_t                          int_ois;
  lsm6dsr_ctrl1_ois_t                        ctrl1_ois;
  lsm6dsr_ctrl2_ois_t                        ctrl2_ois;
  lsm6dsr_ctrl3_ois_t                        ctrl3_ois;
  lsm6dsr_slv0_add_t                         slv0_add;
  lsm6dsr_slv0_subadd_t                      slv0_subadd;
  lsm6dsr_slave0_config_t                    slave0_config;
  lsm6dsr_slv1_add_t                         slv1_add;
  lsm6dsr_slv1_subadd_t                      slv1_subadd;
  lsm6dsr_slave1_config_t                    slave1_config;
  lsm6dsr_slv2_add_t                         slv2_add;
  lsm6dsr_slv2_subadd_t                      slv2_subadd;
  lsm6dsr_slave2_config_t                    slave2_config;
  lsm6dsr_slv3_add_t                         slv3_add;
  lsm6dsr_slv3_subadd_t                      slv3_subadd;
  lsm6dsr_slave3_config_t                    slave3_config;
  lsm6dsr_datawrite_src_mode_sub_slv0_t
  datawrite_src_mode_sub_slv0;
  bitwise_t                                    bitwise;
  uint8_t                                      byte;
} lsm6dsr_reg_t;

/**
  * @}
  *
  */

int32_t lsm6dsr_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);
int32_t lsm6dsr_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                            uint8_t *data,
                            uint16_t len);

float_t lsm6dsr_from_fs2g_to_mg(int16_t lsb);
float_t lsm6dsr_from_fs4g_to_mg(int16_t lsb);
float_t lsm6dsr_from_fs8g_to_mg(int16_t lsb);
float_t lsm6dsr_from_fs16g_to_mg(int16_t lsb);

float_t lsm6dsr_from_fs125dps_to_mdps(int16_t lsb);
float_t lsm6dsr_from_fs250dps_to_mdps(int16_t lsb);
float_t lsm6dsr_from_fs500dps_to_mdps(int16_t lsb);
float_t lsm6dsr_from_fs1000dps_to_mdps(int16_t lsb);
float_t lsm6dsr_from_fs2000dps_to_mdps(int16_t lsb);

float_t lsm6dsr_from_lsb_to_celsius(int16_t lsb);

typedef enum
{
  LSM6DSR_2g       = 0,
  LSM6DSR_16g      = 1,
  LSM6DSR_4g       = 2,
  LSM6DSR_8g       = 3,
  LSM6DSR_XL_FS_ND = 4,  /* ERROR CODE */
} lsm6dsr_fs_xl_t;
int32_t lsm6dsr_xl_full_scale_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_fs_xl_t val);
int32_t lsm6dsr_xl_full_scale_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_fs_xl_t *val);

typedef enum
{
  LSM6DSR_XL_ODR_OFF      =  0,
  LSM6DSR_XL_ODR_12Hz5    =  1,
  LSM6DSR_XL_ODR_26Hz     =  2,
  LSM6DSR_XL_ODR_52Hz     =  3,
  LSM6DSR_XL_ODR_104Hz    =  4,
  LSM6DSR_XL_ODR_208Hz    =  5,
  LSM6DSR_XL_ODR_416Hz    =  6,
  LSM6DSR_XL_ODR_833Hz    =  7,
  LSM6DSR_XL_ODR_1k66Hz   =  8,
  LSM6DSR_XL_ODR_3k33Hz   =  9,
  LSM6DSR_XL_ODR_6k66Hz   = 10,
  LSM6DSR_XL_ODR_1Hz6     = 11,
} lsm6dsr_odr_xl_t;
int32_t lsm6dsr_xl_data_rate_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_odr_xl_t val);
int32_t lsm6dsr_xl_data_rate_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_odr_xl_t *val);

typedef enum
{
  LSM6DSR_250dps   = 0,
  LSM6DSR_125dps   = 1,
  LSM6DSR_500dps   = 2,
  LSM6DSR_1000dps  = 4,
  LSM6DSR_2000dps  = 6,
} lsm6dsr_fs_g_t;
int32_t lsm6dsr_gy_full_scale_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_fs_g_t val);
int32_t lsm6dsr_gy_full_scale_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_fs_g_t *val);

typedef enum
{
  LSM6DSR_GY_ODR_OFF    =  0,
  LSM6DSR_GY_ODR_12Hz5  =  1,
  LSM6DSR_GY_ODR_26Hz   =  2,
  LSM6DSR_GY_ODR_52Hz   =  3,
  LSM6DSR_GY_ODR_104Hz  =  4,
  LSM6DSR_GY_ODR_208Hz  =  5,
  LSM6DSR_GY_ODR_416Hz  =  6,
  LSM6DSR_GY_ODR_833Hz  =  7,
  LSM6DSR_GY_ODR_1k66Hz =  8,
  LSM6DSR_GY_ODR_3k33Hz =  9,
  LSM6DSR_GY_ODR_6k66Hz = 10,
} lsm6dsr_odr_g_t;
int32_t lsm6dsr_gy_data_rate_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_odr_g_t val);
int32_t lsm6dsr_gy_data_rate_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_odr_g_t *val);

int32_t lsm6dsr_block_data_update_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lsm6dsr_block_data_update_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

typedef enum
{
  LSM6DSR_LSb_1mg  = 0,
  LSM6DSR_LSb_16mg = 1,
} lsm6dsr_usr_off_w_t;
int32_t lsm6dsr_xl_offset_weight_set(stmdev_ctx_t *ctx,
                                       lsm6dsr_usr_off_w_t val);
int32_t lsm6dsr_xl_offset_weight_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_usr_off_w_t *val);

typedef enum
{
  LSM6DSR_XL_HIGH_PERFORMANCE  = 0,
  LSM6DSR_XL_NORMAL            = 1,
} lsm6dsr_xl_hm_mode_t;
int32_t lsm6dsr_xl_power_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_xl_hm_mode_t val);
int32_t lsm6dsr_xl_power_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_xl_hm_mode_t *val);

typedef enum
{
  LSM6DSR_STAT_RND_DISABLE  = 0,
  LSM6DSR_STAT_RND_ENABLE   = 1,
} lsm6dsr_rounding_status_t;
int32_t lsm6dsr_rounding_on_status_set(stmdev_ctx_t *ctx,
                                         lsm6dsr_rounding_status_t val);
int32_t lsm6dsr_rounding_on_status_get(stmdev_ctx_t *ctx,
                                         lsm6dsr_rounding_status_t *val);

typedef enum
{
  LSM6DSR_GY_HIGH_PERFORMANCE  = 0,
  LSM6DSR_GY_NORMAL            = 1,
} lsm6dsr_g_hm_mode_t;
int32_t lsm6dsr_gy_power_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_g_hm_mode_t val);
int32_t lsm6dsr_gy_power_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_g_hm_mode_t *val);

typedef struct
{
  lsm6dsr_wake_up_src_t        wake_up_src;
  lsm6dsr_tap_src_t            tap_src;
  lsm6dsr_d6d_src_t            d6d_src;
  lsm6dsr_status_reg_t         status_reg;
  lsm6dsr_func_src1_t          func_src1;
  lsm6dsr_func_src2_t          func_src2;
} lsm6dsr_all_sources_t;
int32_t lsm6dsr_all_sources_get(stmdev_ctx_t *ctx,
                                  lsm6dsr_all_sources_t *val);

int32_t lsm6dsr_status_reg_get(stmdev_ctx_t *ctx,
                                 lsm6dsr_status_reg_t *val);

int32_t lsm6dsr_xl_flag_data_ready_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t lsm6dsr_gy_flag_data_ready_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t lsm6dsr_temp_flag_data_ready_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

int32_t lsm6dsr_xl_usr_offset_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsr_xl_usr_offset_get(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsr_timestamp_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_timestamp_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSR_LSB_6ms4 = 0,
  LSM6DSR_LSB_25us = 1,
} lsm6dsr_timer_hr_t;
int32_t lsm6dsr_timestamp_res_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_timer_hr_t val);
int32_t lsm6dsr_timestamp_res_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_timer_hr_t *val);

typedef enum
{
  LSM6DSR_ROUND_DISABLE            = 0,
  LSM6DSR_ROUND_XL                 = 1,
  LSM6DSR_ROUND_GY                 = 2,
  LSM6DSR_ROUND_GY_XL              = 3,
  LSM6DSR_ROUND_SH1_TO_SH6         = 4,
  LSM6DSR_ROUND_XL_SH1_TO_SH6      = 5,
  LSM6DSR_ROUND_GY_XL_SH1_TO_SH12  = 6,
  LSM6DSR_ROUND_GY_XL_SH1_TO_SH6   = 7,
} lsm6dsr_rounding_t;
int32_t lsm6dsr_rounding_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_rounding_t val);
int32_t lsm6dsr_rounding_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_rounding_t *val);

int32_t lsm6dsr_temperature_raw_get(stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t lsm6dsr_angular_rate_raw_get(stmdev_ctx_t *ctx,
                                       int16_t *val);

int32_t lsm6dsr_acceleration_raw_get(stmdev_ctx_t *ctx,
                                       int16_t *val);

int32_t lsm6dsr_mag_calibrated_raw_get(stmdev_ctx_t *ctx,
                                         int16_t *val);

int32_t lsm6dsr_fifo_raw_data_get(stmdev_ctx_t *ctx,
                                    uint8_t *buffer,
                                    uint8_t len);

typedef enum
{
  LSM6DSR_USER_BANK   = 0,
  LSM6DSR_BANK_A      = 1,
} lsm6dsr_func_cfg_en_t;
int32_t lsm6dsr_mem_bank_set(stmdev_ctx_t *ctx,
                               lsm6dsr_func_cfg_en_t val);
int32_t lsm6dsr_mem_bank_get(stmdev_ctx_t *ctx,
                               lsm6dsr_func_cfg_en_t *val);

typedef enum
{
  LSM6DSR_DRDY_LATCHED    = 0,
  LSM6DSR_DRDY_PULSED     = 1,
} lsm6dsr_drdy_pulsed_t;
int32_t lsm6dsr_data_ready_mode_set(stmdev_ctx_t *ctx,
                                      lsm6dsr_drdy_pulsed_t val);
int32_t lsm6dsr_data_ready_mode_get(stmdev_ctx_t *ctx,
                                      lsm6dsr_drdy_pulsed_t *val);

int32_t lsm6dsr_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsr_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSR_LSB_AT_LOW_ADD  = 0,
  LSM6DSR_MSB_AT_LOW_ADD  = 1,
} lsm6dsr_ble_t;
int32_t lsm6dsr_data_format_set(stmdev_ctx_t *ctx,
                                  lsm6dsr_ble_t val);
int32_t lsm6dsr_data_format_get(stmdev_ctx_t *ctx,
                                  lsm6dsr_ble_t *val);

int32_t lsm6dsr_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSR_XL_ST_DISABLE    = 0,
  LSM6DSR_XL_ST_POSITIVE   = 1,
  LSM6DSR_XL_ST_NEGATIVE   = 2,
} lsm6dsr_st_xl_t;
int32_t lsm6dsr_xl_self_test_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_st_xl_t val);
int32_t lsm6dsr_xl_self_test_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_st_xl_t *val);

typedef enum
{
  LSM6DSR_GY_ST_DISABLE    = 0,
  LSM6DSR_GY_ST_POSITIVE   = 1,
  LSM6DSR_GY_ST_NEGATIVE   = 3,
} lsm6dsr_st_g_t;
int32_t lsm6dsr_gy_self_test_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_st_g_t val);
int32_t lsm6dsr_gy_self_test_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_st_g_t *val);

int32_t lsm6dsr_filter_settling_mask_set(stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm6dsr_filter_settling_mask_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

typedef enum
{
  LSM6DSR_USE_SLOPE = 0,
  LSM6DSR_USE_HPF   = 1,
} lsm6dsr_slope_fds_t;
int32_t lsm6dsr_xl_hp_path_internal_set(stmdev_ctx_t *ctx,
                                          lsm6dsr_slope_fds_t val);
int32_t lsm6dsr_xl_hp_path_internal_get(stmdev_ctx_t *ctx,
                                          lsm6dsr_slope_fds_t *val);

typedef enum
{
  LSM6DSR_XL_ANA_BW_1k5Hz = 0,
  LSM6DSR_XL_ANA_BW_400Hz = 1,
} lsm6dsr_bw0_xl_t;
int32_t lsm6dsr_xl_filter_analog_set(stmdev_ctx_t *ctx,
                                       lsm6dsr_bw0_xl_t val);
int32_t lsm6dsr_xl_filter_analog_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_bw0_xl_t *val);

typedef enum
{
  LSM6DSR_XL_LP1_ODR_DIV_2 = 0,
  LSM6DSR_XL_LP1_ODR_DIV_4 = 1,
  LSM6DSR_XL_LP1_NA        = 2,
} lsm6dsr_lpf1_bw_sel_t;
int32_t lsm6dsr_xl_lp1_bandwidth_set(stmdev_ctx_t *ctx,
                                       lsm6dsr_lpf1_bw_sel_t val);
int32_t lsm6dsr_xl_lp1_bandwidth_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_lpf1_bw_sel_t *val);

typedef enum
{
  LSM6DSR_XL_LOW_LAT_LP_ODR_DIV_50     = 0x00,
  LSM6DSR_XL_LOW_LAT_LP_ODR_DIV_100    = 0x01,
  LSM6DSR_XL_LOW_LAT_LP_ODR_DIV_9      = 0x02,
  LSM6DSR_XL_LOW_LAT_LP_ODR_DIV_400    = 0x03,
  LSM6DSR_XL_LOW_NOISE_LP_ODR_DIV_50   = 0x10,
  LSM6DSR_XL_LOW_NOISE_LP_ODR_DIV_100  = 0x11,
  LSM6DSR_XL_LOW_NOISE_LP_ODR_DIV_9    = 0x12,
  LSM6DSR_XL_LOW_NOISE_LP_ODR_DIV_400  = 0x13,
  LSM6DSR_XL_LP_NA                     = 0x14
} lsm6dsr_input_composite_t;
int32_t lsm6dsr_xl_lp2_bandwidth_set(stmdev_ctx_t *ctx,
                                       lsm6dsr_input_composite_t val);
int32_t lsm6dsr_xl_lp2_bandwidth_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_input_composite_t *val);

int32_t lsm6dsr_xl_reference_mode_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lsm6dsr_xl_reference_mode_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

typedef enum
{
  LSM6DSR_XL_HP_ODR_DIV_4      = 0x00, /* Slope filter */
  LSM6DSR_XL_HP_ODR_DIV_100    = 0x01,
  LSM6DSR_XL_HP_ODR_DIV_9      = 0x02,
  LSM6DSR_XL_HP_ODR_DIV_400    = 0x03,
  LSM6DSR_XL_HP_NA             = 0x04,
} lsm6dsr_hpcf_xl_t;
int32_t lsm6dsr_xl_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                      lsm6dsr_hpcf_xl_t val);
int32_t lsm6dsr_xl_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                      lsm6dsr_hpcf_xl_t *val);

typedef enum
{
  LSM6DSR_XL_UI_LP1_ODR_DIV_2 = 0,
  LSM6DSR_XL_UI_LP1_ODR_DIV_4 = 1,
  LSM6DSR_XL_UI_LP1_NA        = 2,  /* ERROR CODE */
} lsm6dsr_ui_lpf1_bw_sel_t;
int32_t lsm6dsr_xl_ui_lp1_bandwidth_set(stmdev_ctx_t *ctx,
                                          lsm6dsr_ui_lpf1_bw_sel_t val);
int32_t lsm6dsr_xl_ui_lp1_bandwidth_get(stmdev_ctx_t *ctx,
                                          lsm6dsr_ui_lpf1_bw_sel_t *val);

int32_t lsm6dsr_xl_ui_slope_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_xl_ui_slope_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSR_AUX_LP_LIGHT          = 2,
  LSM6DSR_AUX_LP_NORMAL         = 3,
  LSM6DSR_AUX_LP_STRONG         = 0,
  LSM6DSR_AUX_LP_AGGRESSIVE     = 1,
} lsm6dsr_filter_xl_conf_ois_t;
int32_t lsm6dsr_xl_aux_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                          lsm6dsr_filter_xl_conf_ois_t val);
int32_t lsm6dsr_xl_aux_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                          lsm6dsr_filter_xl_conf_ois_t *val);

typedef enum
{
  LSM6DSR_LP2_ONLY                    = 0x00,

  LSM6DSR_HP_16mHz_LP2                = 0x80,
  LSM6DSR_HP_65mHz_LP2                = 0x90,
  LSM6DSR_HP_260mHz_LP2               = 0xA0,
  LSM6DSR_HP_1Hz04_LP2                = 0xB0,

  LSM6DSR_HP_DISABLE_LP1_LIGHT        = 0x0A,
  LSM6DSR_HP_DISABLE_LP1_NORMAL       = 0x09,
  LSM6DSR_HP_DISABLE_LP_STRONG        = 0x08,
  LSM6DSR_HP_DISABLE_LP1_AGGRESSIVE   = 0x0B,

  LSM6DSR_HP_16mHz_LP1_LIGHT          = 0x8A,
  LSM6DSR_HP_65mHz_LP1_NORMAL         = 0x99,
  LSM6DSR_HP_260mHz_LP1_STRONG        = 0xA8,
  LSM6DSR_HP_1Hz04_LP1_AGGRESSIVE     = 0xBB,
} lsm6dsr_lpf1_sel_g_t;
int32_t lsm6dsr_gy_band_pass_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_lpf1_sel_g_t val);
int32_t lsm6dsr_gy_band_pass_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_lpf1_sel_g_t *val);

int32_t lsm6dsr_gy_ui_high_pass_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_gy_ui_high_pass_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LSM6DSR_HP_DISABLE_LP_173Hz        = 0x02,
  LSM6DSR_HP_DISABLE_LP_237Hz        = 0x01,
  LSM6DSR_HP_DISABLE_LP_351Hz        = 0x00,
  LSM6DSR_HP_DISABLE_LP_937Hz        = 0x03,

  LSM6DSR_HP_16mHz_LP_173Hz          = 0x82,
  LSM6DSR_HP_65mHz_LP_237Hz          = 0x91,
  LSM6DSR_HP_260mHz_LP_351Hz         = 0xA0,
  LSM6DSR_HP_1Hz04_LP_937Hz          = 0xB3,
} lsm6dsr_hp_en_ois_t;
int32_t lsm6dsr_gy_aux_bandwidth_set(stmdev_ctx_t *ctx,
                                       lsm6dsr_hp_en_ois_t val);
int32_t lsm6dsr_gy_aux_bandwidth_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_hp_en_ois_t *val);

int32_t lsm6dsr_aux_status_reg_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_status_spiaux_t *val);

int32_t lsm6dsr_aux_xl_flag_data_ready_get(stmdev_ctx_t *ctx,
                                             uint8_t *val);

int32_t lsm6dsr_aux_gy_flag_data_ready_get(stmdev_ctx_t *ctx,
                                             uint8_t *val);

int32_t lsm6dsr_aux_gy_flag_settling_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

typedef enum
{
  LSM6DSR_AUX_DEN_DISABLE         = 0,
  LSM6DSR_AUX_DEN_LEVEL_LATCH     = 3,
  LSM6DSR_AUX_DEN_LEVEL_TRIG      = 2,
} lsm6dsr_lvl_ois_t;
int32_t lsm6dsr_aux_den_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_lvl_ois_t val);
int32_t lsm6dsr_aux_den_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_lvl_ois_t *val);

int32_t lsm6dsr_aux_drdy_on_int2_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t lsm6dsr_aux_drdy_on_int2_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

typedef enum
{
  LSM6DSR_AUX_DISABLE   = 0,
  LSM6DSR_MODE_3_GY     = 1,
  LSM6DSR_MODE_4_GY_XL  = 3,
} lsm6dsr_ois_en_spi2_t;
int32_t lsm6dsr_aux_mode_set(stmdev_ctx_t *ctx,
                               lsm6dsr_ois_en_spi2_t val);
int32_t lsm6dsr_aux_mode_get(stmdev_ctx_t *ctx,
                               lsm6dsr_ois_en_spi2_t *val);

typedef enum
{
  LSM6DSR_250dps_AUX   = 0,
  LSM6DSR_125dps_AUX   = 1,
  LSM6DSR_500dps_AUX   = 2,
  LSM6DSR_1000dps_AUX  = 4,
  LSM6DSR_2000dps_AUX  = 6,
} lsm6dsr_fs_g_ois_t;
int32_t lsm6dsr_aux_gy_full_scale_set(stmdev_ctx_t *ctx,
                                        lsm6dsr_fs_g_ois_t val);
int32_t lsm6dsr_aux_gy_full_scale_get(stmdev_ctx_t *ctx,
                                        lsm6dsr_fs_g_ois_t *val);

typedef enum
{
  LSM6DSR_AUX_SPI_4_WIRE = 0,
  LSM6DSR_AUX_SPI_3_WIRE = 1,
} lsm6dsr_sim_ois_t;
int32_t lsm6dsr_aux_spi_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_sim_ois_t val);
int32_t lsm6dsr_aux_spi_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_sim_ois_t *val);

typedef enum
{
  LSM6DSR_AUX_LSB_AT_LOW_ADD = 0,
  LSM6DSR_AUX_MSB_AT_LOW_ADD = 1,
} lsm6dsr_ble_ois_t;
int32_t lsm6dsr_aux_data_format_set(stmdev_ctx_t *ctx,
                                      lsm6dsr_ble_ois_t val);
int32_t lsm6dsr_aux_data_format_get(stmdev_ctx_t *ctx,
                                      lsm6dsr_ble_ois_t *val);

typedef enum
{
  LSM6DSR_ENABLE_CLAMP    = 0,
  LSM6DSR_DISABLE_CLAMP   = 1,
} lsm6dsr_st_ois_clampdis_t;
int32_t lsm6dsr_aux_gy_clamp_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_st_ois_clampdis_t val);
int32_t lsm6dsr_aux_gy_clamp_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_st_ois_clampdis_t *val);

typedef enum
{
  LSM6DSR_AUX_GY_DISABLE  = 0,
  LSM6DSR_AUX_GY_POS      = 1,
  LSM6DSR_AUX_GY_NEG      = 3,
} lsm6dsr_st_ois_t;
int32_t lsm6dsr_aux_gy_self_test_set(stmdev_ctx_t *ctx,
                                       lsm6dsr_st_ois_t val);
int32_t lsm6dsr_aux_gy_self_test_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_st_ois_t *val);

typedef enum
{
  LSM6DSR_AUX_2g   = 0,
  LSM6DSR_AUX_16g  = 1,
  LSM6DSR_AUX_4g   = 2,
  LSM6DSR_AUX_8g   = 3,
} lsm6dsr_fs_xl_ois_t;
int32_t lsm6dsr_aux_xl_full_scale_set(stmdev_ctx_t *ctx,
                                        lsm6dsr_fs_xl_ois_t val);
int32_t lsm6dsr_aux_xl_full_scale_get(stmdev_ctx_t *ctx,
                                        lsm6dsr_fs_xl_ois_t *val);

typedef enum
{
  LSM6DSR_AUX_DEN_ACTIVE_LOW   = 0,
  LSM6DSR_AUX_DEN_ACTIVE_HIGH  = 1,
} lsm6dsr_den_lh_ois_t;
int32_t lsm6dsr_aux_den_polarity_set(stmdev_ctx_t *ctx,
                                       lsm6dsr_den_lh_ois_t val);
int32_t lsm6dsr_aux_den_polarity_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_den_lh_ois_t *val);

typedef enum
{
  LSM6DSR_SPI_4_WIRE  = 0,
  LSM6DSR_SPI_3_WIRE  = 1,
} lsm6dsr_sim_t;
int32_t lsm6dsr_spi_mode_set(stmdev_ctx_t *ctx,
                               lsm6dsr_sim_t val);
int32_t lsm6dsr_spi_mode_get(stmdev_ctx_t *ctx,
                               lsm6dsr_sim_t *val);

typedef enum
{
  LSM6DSR_I2C_ENABLE   = 0,
  LSM6DSR_I2C_DISABLE  = 1,
} lsm6dsr_i2c_disable_t;
int32_t lsm6dsr_i2c_interface_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_i2c_disable_t val);
int32_t lsm6dsr_i2c_interface_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_i2c_disable_t *val);

typedef struct
{
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_full_flag           : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_inact_state         : 1;
  uint8_t den_drdy_int1            : 1;
  uint8_t drdy_on_int1             : 1;
} lsm6dsr_int1_route_t;
int32_t lsm6dsr_pin_int1_route_set(stmdev_ctx_t *ctx,
                                     lsm6dsr_int1_route_t val);
int32_t lsm6dsr_pin_int1_route_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_int1_route_t *val);

typedef struct
{
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_full_flag           : 1;
  uint8_t int2_iron                : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_inact_state         : 1;
} lsm6dsr_int2_route_t;
int32_t lsm6dsr_pin_int2_route_set(stmdev_ctx_t *ctx,
                                     lsm6dsr_int2_route_t val);
int32_t lsm6dsr_pin_int2_route_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_int2_route_t *val);

typedef enum
{
  LSM6DSR_PUSH_PULL   = 0,
  LSM6DSR_OPEN_DRAIN  = 1,
} lsm6dsr_pp_od_t;
int32_t lsm6dsr_pin_mode_set(stmdev_ctx_t *ctx,
                               lsm6dsr_pp_od_t val);
int32_t lsm6dsr_pin_mode_get(stmdev_ctx_t *ctx,
                               lsm6dsr_pp_od_t *val);

typedef enum
{
  LSM6DSR_ACTIVE_HIGH   = 0,
  LSM6DSR_ACTIVE_LOW    = 1,
} lsm6dsr_h_lactive_t;
int32_t lsm6dsr_pin_polarity_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_h_lactive_t val);
int32_t lsm6dsr_pin_polarity_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_h_lactive_t *val);

int32_t lsm6dsr_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSR_INT_PULSED   = 0,
  LSM6DSR_INT_LATCHED  = 1,
} lsm6dsr_lir_t;
int32_t lsm6dsr_int_notification_set(stmdev_ctx_t *ctx,
                                       lsm6dsr_lir_t val);
int32_t lsm6dsr_int_notification_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_lir_t *val);

int32_t lsm6dsr_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_gy_sleep_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_gy_sleep_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSR_PROPERTY_DISABLE          = 0,
  LSM6DSR_XL_12Hz5_GY_NOT_AFFECTED  = 1,
  LSM6DSR_XL_12Hz5_GY_SLEEP         = 2,
  LSM6DSR_XL_12Hz5_GY_PD            = 3,
} lsm6dsr_inact_en_t;
int32_t lsm6dsr_act_mode_set(stmdev_ctx_t *ctx,
                               lsm6dsr_inact_en_t val);
int32_t lsm6dsr_act_mode_get(stmdev_ctx_t *ctx,
                               lsm6dsr_inact_en_t *val);

int32_t lsm6dsr_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_tap_src_get(stmdev_ctx_t *ctx,
                              lsm6dsr_tap_src_t *val);

int32_t lsm6dsr_tap_detection_on_z_set(stmdev_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm6dsr_tap_detection_on_z_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t lsm6dsr_tap_detection_on_y_set(stmdev_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm6dsr_tap_detection_on_y_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t lsm6dsr_tap_detection_on_x_set(stmdev_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm6dsr_tap_detection_on_x_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t lsm6dsr_tap_threshold_x_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_tap_threshold_x_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lsm6dsr_tap_shock_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_tap_shock_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_tap_quiet_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_tap_quiet_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_tap_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_tap_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSR_ONLY_SINGLE          = 0,
  LSM6DSR_BOTH_SINGLE_DOUBLE   = 1,
} lsm6dsr_single_double_tap_t;
int32_t lsm6dsr_tap_mode_set(stmdev_ctx_t *ctx,
                               lsm6dsr_single_double_tap_t val);
int32_t lsm6dsr_tap_mode_get(stmdev_ctx_t *ctx,
                               lsm6dsr_single_double_tap_t *val);

typedef enum
{
  LSM6DSR_ODR_DIV_2_FEED      = 0,
  LSM6DSR_LPF2_FEED           = 1,
} lsm6dsr_low_pass_on_6d_t;
int32_t lsm6dsr_6d_feed_data_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_low_pass_on_6d_t val);
int32_t lsm6dsr_6d_feed_data_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_low_pass_on_6d_t *val);

typedef enum
{
  LSM6DSR_DEG_80      = 0,
  LSM6DSR_DEG_70      = 1,
  LSM6DSR_DEG_60      = 2,
  LSM6DSR_DEG_50      = 3,
} lsm6dsr_sixd_ths_t;
int32_t lsm6dsr_6d_threshold_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_sixd_ths_t val);
int32_t lsm6dsr_6d_threshold_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_sixd_ths_t *val);

int32_t lsm6dsr_4d_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_4d_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_ff_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_ff_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSR_FF_TSH_156mg = 0,
  LSM6DSR_FF_TSH_219mg = 1,
  LSM6DSR_FF_TSH_250mg = 2,
  LSM6DSR_FF_TSH_312mg = 3,
  LSM6DSR_FF_TSH_344mg = 4,
  LSM6DSR_FF_TSH_406mg = 5,
  LSM6DSR_FF_TSH_469mg = 6,
  LSM6DSR_FF_TSH_500mg = 7,
} lsm6dsr_ff_ths_t;
int32_t lsm6dsr_ff_threshold_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_ff_ths_t val);
int32_t lsm6dsr_ff_threshold_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_ff_ths_t *val);

int32_t lsm6dsr_fifo_watermark_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm6dsr_fifo_watermark_get(stmdev_ctx_t *ctx,
                                     uint16_t *val);

int32_t lsm6dsr_fifo_data_level_get(stmdev_ctx_t *ctx,
                                      uint16_t *val);

int32_t lsm6dsr_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_fifo_pattern_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsr_fifo_temp_batch_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_fifo_temp_batch_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LSM6DSR_TRG_XL_GY_DRDY     = 0,
  LSM6DSR_TRG_SH_DRDY        = 1,
} lsm6dsr_trigger_fifo_t;
int32_t lsm6dsr_fifo_write_trigger_set(stmdev_ctx_t *ctx,
                                         lsm6dsr_trigger_fifo_t val);
int32_t lsm6dsr_fifo_write_trigger_get(stmdev_ctx_t *ctx,
                                         lsm6dsr_trigger_fifo_t *val);

typedef enum
{
  LSM6DSR_FIFO_XL_DISABLE  = 0,
  LSM6DSR_FIFO_XL_NO_DEC   = 1,
  LSM6DSR_FIFO_XL_DEC_2    = 2,
  LSM6DSR_FIFO_XL_DEC_3    = 3,
  LSM6DSR_FIFO_XL_DEC_4    = 4,
  LSM6DSR_FIFO_XL_DEC_8    = 5,
  LSM6DSR_FIFO_XL_DEC_16   = 6,
  LSM6DSR_FIFO_XL_DEC_32   = 7,
} lsm6dsr_dec_fifo_xl_t;
int32_t lsm6dsr_fifo_xl_batch_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_dec_fifo_xl_t val);
int32_t lsm6dsr_fifo_xl_batch_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_dec_fifo_xl_t *val);

typedef enum
{
  LSM6DSR_FIFO_GY_DISABLE = 0,
  LSM6DSR_FIFO_GY_NO_DEC  = 1,
  LSM6DSR_FIFO_GY_DEC_2   = 2,
  LSM6DSR_FIFO_GY_DEC_3   = 3,
  LSM6DSR_FIFO_GY_DEC_4   = 4,
  LSM6DSR_FIFO_GY_DEC_8   = 5,
  LSM6DSR_FIFO_GY_DEC_16  = 6,
  LSM6DSR_FIFO_GY_DEC_32  = 7,
} lsm6dsr_dec_fifo_gyro_t;
int32_t lsm6dsr_fifo_gy_batch_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_dec_fifo_gyro_t val);
int32_t lsm6dsr_fifo_gy_batch_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_dec_fifo_gyro_t *val);

typedef enum
{
  LSM6DSR_FIFO_DS3_DISABLE   = 0,
  LSM6DSR_FIFO_DS3_NO_DEC    = 1,
  LSM6DSR_FIFO_DS3_DEC_2     = 2,
  LSM6DSR_FIFO_DS3_DEC_3     = 3,
  LSM6DSR_FIFO_DS3_DEC_4     = 4,
  LSM6DSR_FIFO_DS3_DEC_8     = 5,
  LSM6DSR_FIFO_DS3_DEC_16    = 6,
  LSM6DSR_FIFO_DS3_DEC_32    = 7,
} lsm6dsr_dec_ds3_fifo_t;
int32_t lsm6dsr_fifo_dataset_3_batch_set(stmdev_ctx_t *ctx,
                                           lsm6dsr_dec_ds3_fifo_t val);
int32_t lsm6dsr_fifo_dataset_3_batch_get(stmdev_ctx_t *ctx,
                                           lsm6dsr_dec_ds3_fifo_t *val);

typedef enum
{
  LSM6DSR_FIFO_DS4_DISABLE  = 0,
  LSM6DSR_FIFO_DS4_NO_DEC   = 1,
  LSM6DSR_FIFO_DS4_DEC_2    = 2,
  LSM6DSR_FIFO_DS4_DEC_3    = 3,
  LSM6DSR_FIFO_DS4_DEC_4    = 4,
  LSM6DSR_FIFO_DS4_DEC_8    = 5,
  LSM6DSR_FIFO_DS4_DEC_16   = 6,
  LSM6DSR_FIFO_DS4_DEC_32   = 7,
} lsm6dsr_dec_ds4_fifo_t;
int32_t lsm6dsr_fifo_dataset_4_batch_set(stmdev_ctx_t *ctx,
                                           lsm6dsr_dec_ds4_fifo_t val);
int32_t lsm6dsr_fifo_dataset_4_batch_get(stmdev_ctx_t *ctx,
                                           lsm6dsr_dec_ds4_fifo_t *val);

int32_t lsm6dsr_fifo_xl_gy_8bit_format_set(stmdev_ctx_t *ctx,
                                             uint8_t val);
int32_t lsm6dsr_fifo_xl_gy_8bit_format_get(stmdev_ctx_t *ctx,
                                             uint8_t *val);

int32_t lsm6dsr_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t lsm6dsr_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

typedef enum
{
  LSM6DSR_BYPASS_MODE           = 0,
  LSM6DSR_FIFO_MODE             = 1,
  LSM6DSR_STREAM_TO_FIFO_MODE   = 3,
  LSM6DSR_BYPASS_TO_STREAM_MODE = 4,
  LSM6DSR_STREAM_MODE           = 6,
} lsm6dsr_fifo_mode_t;
int32_t lsm6dsr_fifo_mode_set(stmdev_ctx_t *ctx,
                                lsm6dsr_fifo_mode_t val);
int32_t lsm6dsr_fifo_mode_get(stmdev_ctx_t *ctx,
                                lsm6dsr_fifo_mode_t *val);

typedef enum
{
  LSM6DSR_FIFO_DISABLE   =  0,
  LSM6DSR_FIFO_12Hz5     =  1,
  LSM6DSR_FIFO_26Hz      =  2,
  LSM6DSR_FIFO_52Hz      =  3,
  LSM6DSR_FIFO_104Hz     =  4,
  LSM6DSR_FIFO_208Hz     =  5,
  LSM6DSR_FIFO_416Hz     =  6,
  LSM6DSR_FIFO_833Hz     =  7,
  LSM6DSR_FIFO_1k66Hz    =  8,
  LSM6DSR_FIFO_3k33Hz    =  9,
  LSM6DSR_FIFO_6k66Hz    = 10,
} lsm6dsr_odr_fifo_t;
int32_t lsm6dsr_fifo_data_rate_set(stmdev_ctx_t *ctx,
                                     lsm6dsr_odr_fifo_t val);
int32_t lsm6dsr_fifo_data_rate_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_odr_fifo_t *val);

typedef enum
{
  LSM6DSR_DEN_ACT_LOW    = 0,
  LSM6DSR_DEN_ACT_HIGH   = 1,
} lsm6dsr_den_lh_t;
int32_t lsm6dsr_den_polarity_set(stmdev_ctx_t *ctx,
                                   lsm6dsr_den_lh_t val);
int32_t lsm6dsr_den_polarity_get(stmdev_ctx_t *ctx,
                                   lsm6dsr_den_lh_t *val);

typedef enum
{
  LSM6DSR_DEN_DISABLE    = 0,
  LSM6DSR_LEVEL_FIFO     = 6,
  LSM6DSR_LEVEL_LETCHED  = 3,
  LSM6DSR_LEVEL_TRIGGER  = 2,
  LSM6DSR_EDGE_TRIGGER   = 4,
} lsm6dsr_den_mode_t;
int32_t lsm6dsr_den_mode_set(stmdev_ctx_t *ctx,
                               lsm6dsr_den_mode_t val);
int32_t lsm6dsr_den_mode_get(stmdev_ctx_t *ctx,
                               lsm6dsr_den_mode_t *val);

typedef enum
{
  LSM6DSR_STAMP_IN_GY_DATA     = 0,
  LSM6DSR_STAMP_IN_XL_DATA     = 1,
  LSM6DSR_STAMP_IN_GY_XL_DATA  = 2,
} lsm6dsr_den_xl_en_t;
int32_t lsm6dsr_den_enable_set(stmdev_ctx_t *ctx,
                                 lsm6dsr_den_xl_en_t val);
int32_t lsm6dsr_den_enable_get(stmdev_ctx_t *ctx,
                                 lsm6dsr_den_xl_en_t *val);

int32_t lsm6dsr_den_mark_axis_z_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_den_mark_axis_z_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lsm6dsr_den_mark_axis_y_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_den_mark_axis_y_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lsm6dsr_den_mark_axis_x_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_den_mark_axis_x_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lsm6dsr_mag_soft_iron_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_mag_soft_iron_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_mag_hard_iron_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_mag_hard_iron_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_mag_soft_iron_mat_set(stmdev_ctx_t *ctx,
                                        uint8_t *buff);
int32_t lsm6dsr_mag_soft_iron_mat_get(stmdev_ctx_t *ctx,
                                        uint8_t *buff);

int32_t lsm6dsr_mag_offset_set(stmdev_ctx_t *ctx, int16_t *val);
int32_t lsm6dsr_mag_offset_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm6dsr_sh_sync_sens_frame_set(stmdev_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm6dsr_sh_sync_sens_frame_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

typedef enum
{
  LSM6DSR_RES_RATIO_2_11  = 0,
  LSM6DSR_RES_RATIO_2_12  = 1,
  LSM6DSR_RES_RATIO_2_13  = 2,
  LSM6DSR_RES_RATIO_2_14  = 3,
} lsm6dsr_rr_t;
int32_t lsm6dsr_sh_sync_sens_ratio_set(stmdev_ctx_t *ctx,
                                         lsm6dsr_rr_t val);
int32_t lsm6dsr_sh_sync_sens_ratio_get(stmdev_ctx_t *ctx,
                                         lsm6dsr_rr_t *val);

int32_t lsm6dsr_sh_master_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_sh_master_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsr_sh_pass_through_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_sh_pass_through_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LSM6DSR_EXT_PULL_UP       = 0,
  LSM6DSR_INTERNAL_PULL_UP  = 1,
} lsm6dsr_pull_up_en_t;
int32_t lsm6dsr_sh_pin_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsr_pull_up_en_t val);
int32_t lsm6dsr_sh_pin_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsr_pull_up_en_t *val);

typedef enum
{
  LSM6DSR_XL_GY_DRDY        = 1,
  LSM6DSR_EXT_ON_INT2_PIN   = 0,
} lsm6dsr_start_config_t;
int32_t lsm6dsr_sh_syncro_mode_set(stmdev_ctx_t *ctx,
                                     lsm6dsr_start_config_t val);
int32_t lsm6dsr_sh_syncro_mode_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_start_config_t *val);

int32_t lsm6dsr_sh_drdy_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsr_sh_drdy_on_int1_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef struct
{
  lsm6dsr_sensorhub1_reg_t   sh_byte_1;
  lsm6dsr_sensorhub2_reg_t   sh_byte_2;
  lsm6dsr_sensorhub3_reg_t   sh_byte_3;
  lsm6dsr_sensorhub4_reg_t   sh_byte_4;
  lsm6dsr_sensorhub5_reg_t   sh_byte_5;
  lsm6dsr_sensorhub6_reg_t   sh_byte_6;
  lsm6dsr_sensorhub7_reg_t   sh_byte_7;
  lsm6dsr_sensorhub8_reg_t   sh_byte_8;
  lsm6dsr_sensorhub9_reg_t   sh_byte_9;
  lsm6dsr_sensorhub10_reg_t  sh_byte_10;
  lsm6dsr_sensorhub11_reg_t  sh_byte_11;
  lsm6dsr_sensorhub12_reg_t  sh_byte_12;
  lsm6dsr_sensorhub13_reg_t  sh_byte_13;
  lsm6dsr_sensorhub14_reg_t  sh_byte_14;
  lsm6dsr_sensorhub15_reg_t  sh_byte_15;
  lsm6dsr_sensorhub16_reg_t  sh_byte_16;
  lsm6dsr_sensorhub17_reg_t  sh_byte_17;
  lsm6dsr_sensorhub18_reg_t  sh_byte_18;
} lsm6dsr_emb_sh_read_t;
int32_t lsm6dsr_sh_read_data_raw_get(stmdev_ctx_t *ctx,
                                       lsm6dsr_emb_sh_read_t *val);

int32_t lsm6dsr_sh_cmd_sens_sync_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t lsm6dsr_sh_cmd_sens_sync_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t lsm6dsr_sh_spi_sync_error_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lsm6dsr_sh_spi_sync_error_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

typedef enum
{
  LSM6DSR_NORMAL_MODE_READ  = 0,
  LSM6DSR_SRC_MODE_READ     = 1,
} lsm6dsr_src_mode_t;
int32_t lsm6dsr_sh_cfg_slave_0_rd_mode_set(stmdev_ctx_t *ctx,
                                             lsm6dsr_src_mode_t val);
int32_t lsm6dsr_sh_cfg_slave_0_rd_mode_get(stmdev_ctx_t *ctx,
                                             lsm6dsr_src_mode_t *val);

typedef enum
{
  LSM6DSR_SLV_0        = 0,
  LSM6DSR_SLV_0_1      = 1,
  LSM6DSR_SLV_0_1_2    = 2,
  LSM6DSR_SLV_0_1_2_3  = 3,
} lsm6dsr_aux_sens_on_t;
int32_t lsm6dsr_sh_num_of_dev_connected_set(stmdev_ctx_t *ctx,
                                              lsm6dsr_aux_sens_on_t val);
int32_t lsm6dsr_sh_num_of_dev_connected_get(stmdev_ctx_t *ctx,
                                              lsm6dsr_aux_sens_on_t *val);

typedef struct
{
  uint8_t   slv0_add;
  uint8_t   slv0_subadd;
  uint8_t   slv0_data;
} lsm6dsr_sh_cfg_write_t;
int32_t lsm6dsr_sh_cfg_write(stmdev_ctx_t *ctx,
                               lsm6dsr_sh_cfg_write_t *val);

typedef struct
{
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_len;
} lsm6dsr_sh_cfg_read_t;
int32_t lsm6dsr_sh_slv0_cfg_read(stmdev_ctx_t *ctx,
                                   lsm6dsr_sh_cfg_read_t *val);
int32_t lsm6dsr_sh_slv1_cfg_read(stmdev_ctx_t *ctx,
                                   lsm6dsr_sh_cfg_read_t *val);
int32_t lsm6dsr_sh_slv2_cfg_read(stmdev_ctx_t *ctx,
                                   lsm6dsr_sh_cfg_read_t *val);
int32_t lsm6dsr_sh_slv3_cfg_read(stmdev_ctx_t *ctx,
                                   lsm6dsr_sh_cfg_read_t *val);

typedef enum
{
  LSM6DSR_SL0_NO_DEC   = 0,
  LSM6DSR_SL0_DEC_2    = 1,
  LSM6DSR_SL0_DEC_4    = 2,
  LSM6DSR_SL0_DEC_8    = 3,
} lsm6dsr_slave0_rate_t;
int32_t lsm6dsr_sh_slave_0_dec_set(stmdev_ctx_t *ctx,
                                     lsm6dsr_slave0_rate_t val);
int32_t lsm6dsr_sh_slave_0_dec_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_slave0_rate_t *val);

typedef enum
{
  LSM6DSR_EACH_SH_CYCLE     = 0,
  LSM6DSR_ONLY_FIRST_CYCLE  = 1,
} lsm6dsr_write_once_t;
int32_t lsm6dsr_sh_write_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsr_write_once_t val);
int32_t lsm6dsr_sh_write_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsr_write_once_t *val);

typedef enum
{
  LSM6DSR_SL1_NO_DEC   = 0,
  LSM6DSR_SL1_DEC_2    = 1,
  LSM6DSR_SL1_DEC_4    = 2,
  LSM6DSR_SL1_DEC_8    = 3,
} lsm6dsr_slave1_rate_t;
int32_t lsm6dsr_sh_slave_1_dec_set(stmdev_ctx_t *ctx,
                                     lsm6dsr_slave1_rate_t val);
int32_t lsm6dsr_sh_slave_1_dec_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_slave1_rate_t *val);

typedef enum
{
  LSM6DSR_SL2_NO_DEC  = 0,
  LSM6DSR_SL2_DEC_2   = 1,
  LSM6DSR_SL2_DEC_4   = 2,
  LSM6DSR_SL2_DEC_8   = 3,
} lsm6dsr_slave2_rate_t;
int32_t lsm6dsr_sh_slave_2_dec_set(stmdev_ctx_t *ctx,
                                     lsm6dsr_slave2_rate_t val);
int32_t lsm6dsr_sh_slave_2_dec_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_slave2_rate_t *val);

typedef enum
{
  LSM6DSR_SL3_NO_DEC  = 0,
  LSM6DSR_SL3_DEC_2   = 1,
  LSM6DSR_SL3_DEC_4   = 2,
  LSM6DSR_SL3_DEC_8   = 3,
} lsm6dsr_slave3_rate_t;
int32_t lsm6dsr_sh_slave_3_dec_set(stmdev_ctx_t *ctx,
                                     lsm6dsr_slave3_rate_t val);
int32_t lsm6dsr_sh_slave_3_dec_get(stmdev_ctx_t *ctx,
                                     lsm6dsr_slave3_rate_t *val);

/**
  * @}
  *
  */



/*
 * MRT code TODO
 */


//Includes
#include <stm32f4xx_hal.h>


//Variables and defines (functions defined at the end)

#define    BOOT_TIME            10 //ms
#define TX_BUF_DIM          50



//FOR THE ISM
static uint8_t whoamI, rst;
stmdev_ctx_t dev_ctx;

//Necessary for acceleration
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];

//Necessary for angular rate
static int16_t data_raw_angular_rate[3];
static float angular_rate_mdps[3];

//Necessary for temp
static int16_t data_raw_temperature[1];
static float temperature_degC[1];

//For communication
UART_HandleTypeDef* Guart;



//Helper functions
static int32_t write(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len);
static int32_t read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);

//User functions
void MRT_LSM6DSR_Setup(stmdev_ctx_t* dev_ctx, I2C_HandleTypeDef* SENSOR_BUS, UART_HandleTypeDef* uart);
void MRT_LSM6DSR_getAcceleration(int16_t data_raw_acceleration[3],float acceleration_mg[3]);
void MRT_LSM6DSR_getAngularRate(int16_t data_raw_angular_rate[3],float angular_rate_mdps[3]);
void MRT_LSM6DSR_getTemperature(int16_t data_raw_temperature[1],float temperature_degC[1]);


#ifdef __cplusplus
}
#endif

#endif /* LSM6DSR_DRIVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
