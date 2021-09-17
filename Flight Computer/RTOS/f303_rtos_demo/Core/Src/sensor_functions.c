/*
 * sensor_functions.c
 * modified from ST libraries for the Flight Computer project
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "lps22hh_reg.h"
#include "stm32f3xx_hal.h"


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/


// For LPS22HL
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static uint8_t whoamI_lps22hh, rst_lps22hh;


/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t lps22hh_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t lps22hh_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);

/* LPS22HH Functions ---------------------------------------------------------*/
stmdev_ctx_t lps22hh_init(void){
	stmdev_ctx_t dev_ctx_lps22hh;

	/* Initialize mems driver interface */
	dev_ctx_lps22hh.write_reg = lps22hh_write;
	dev_ctx_lps22hh.read_reg = lps22hh_read;
	dev_ctx_lps22hh.handle = &hi2c3;


	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	whoamI_lps22hh = 0;
	lps22hh_device_id_get(&dev_ctx_lps22hh, &whoamI_lps22hh);

	if ( whoamI_lps22hh != LPS22HH_ID ){
		while (1); /*manage here device not found */
	}


	/* Restore default configuration */
	lps22hh_reset_set(&dev_ctx_lps22hh, PROPERTY_ENABLE);

	do {
		lps22hh_reset_get(&dev_ctx_lps22hh, &rst_lps22hh);
	} while (rst_lps22hh);

	/* Enable Block Data Update */
	lps22hh_block_data_update_set(&dev_ctx_lps22hh, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lps22hh_data_rate_set(&dev_ctx_lps22hh, LPS22HH_10_Hz_LOW_NOISE);

	return dev_ctx_lps22hh;
}

void get_pressure(stmdev_ctx_t dev_ctx_lps22hh, float *pressure){
	/* Read output only if new value is available */
	lps22hh_reg_t reg;
	lps22hh_read_reg(&dev_ctx_lps22hh, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	if (reg.status.p_da) {
	  memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
	  lps22hh_pressure_raw_get(&dev_ctx_lps22hh, &data_raw_pressure);
	  *pressure = lps22hh_from_lsb_to_hpa( data_raw_pressure);
	}
}

void get_temperature(stmdev_ctx_t dev_ctx_lps22hh, float *temperature){
	/* Read output only if new value is available */
	lps22hh_reg_t reg;
	lps22hh_read_reg(&dev_ctx_lps22hh, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	if (reg.status.t_da) {
	  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	  lps22hh_temperature_raw_get(&dev_ctx_lps22hh, &data_raw_temperature);
	  *temperature = lps22hh_from_lsb_to_celsius(data_raw_temperature);
	}
}


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lps22hh_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{

  HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lps22hh_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}


/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}
