/*
 * i2c_sensors_functions.c
 *
 * contains the functions needed to communicate with the lps22hh (barometric pressure sensor)
 * and ism330dlc/lsm6dsl (6 DOF acc/gyro).
 *
 *  Created on: Dec 22, 2021
 *      Author: jasper but copied jennie
 */


#include "i2c_sensor_functions.h"

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/

// For LSM6DSL
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static uint8_t whoamI_lsm6dsl, rst_lsm6dsl;

// For LPS22HH
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
static int32_t lsm6dsl_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t lsm6dsl_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static int32_t lps22hh_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t lps22hh_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);

/* LSM6DSL Functions ---------------------------------------------------------*/

stmdev_ctx_t lsm6dsl_init(void){

	stmdev_ctx_t dev_ctx_lsm6dsl;

	/* Initialize mems driver interface */
	dev_ctx_lsm6dsl.write_reg = lsm6dsl_write;
	dev_ctx_lsm6dsl.read_reg = lsm6dsl_read;
	dev_ctx_lsm6dsl.handle = &SENSOR_BUS;

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	lsm6dsl_device_id_get(&dev_ctx_lsm6dsl, &whoamI_lsm6dsl);

	if (whoamI_lsm6dsl != LSM6DSL_ID){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		__BKPT();
//		Error_Handler();
	}

	/* Restore default configuration */
	lsm6dsl_reset_set(&dev_ctx_lsm6dsl, PROPERTY_ENABLE);

	do {
	lsm6dsl_reset_get(&dev_ctx_lsm6dsl, &rst_lsm6dsl);
	} while (rst_lsm6dsl);

	/* Enable Block Data Update */
	lsm6dsl_block_data_update_set(&dev_ctx_lsm6dsl, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lsm6dsl_xl_data_rate_set(&dev_ctx_lsm6dsl, LSM6DSL_XL_ODR_104Hz);
	lsm6dsl_gy_data_rate_set(&dev_ctx_lsm6dsl, LSM6DSL_GY_ODR_104Hz);

	/* Set full scale */
	lsm6dsl_xl_full_scale_set(&dev_ctx_lsm6dsl, LSM6DSL_8g);
	lsm6dsl_gy_full_scale_set(&dev_ctx_lsm6dsl, LSM6DSL_2000dps);

	/* Configure filtering chain(No aux interface)
	* Accelerometer - LPF1 + LPF2 path
	*/
	lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx_lsm6dsl, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
	/* Accelerometer - High Pass / Slope path */
	//lsm6dsl_xl_reference_mode_set(&dev_ctx_lsm, PROPERTY_DISABLE);
	//lsm6dsl_xl_hp_bandwidth_set(&dev_ctx_lsm, LSM6DSL_XL_HP_ODR_DIV_100);
	/* Gyroscope - filtering chain */
	lsm6dsl_gy_band_pass_set(&dev_ctx_lsm6dsl, LSM6DSL_HP_260mHz_LP1_STRONG);

	return dev_ctx_lsm6dsl;
}

void get_acceleration(stmdev_ctx_t dev_ctx_lsm6dsl, float *acceleration_mg){

	uint8_t reg;

	/* Read output only if new xl value is available */
	lsm6dsl_xl_flag_data_ready_get(&dev_ctx_lsm6dsl, &reg);

	if (reg) {
	  /* Read acceleration field data */
	  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  lsm6dsl_acceleration_raw_get(&dev_ctx_lsm6dsl, data_raw_acceleration);
	  acceleration_mg[0] =
		lsm6dsl_from_fs8g_to_mg(data_raw_acceleration[0]);
	  acceleration_mg[1] =
		lsm6dsl_from_fs8g_to_mg(data_raw_acceleration[1]);
	  acceleration_mg[2] =
		lsm6dsl_from_fs8g_to_mg(data_raw_acceleration[2]);
	}

}

void get_angvelocity(stmdev_ctx_t dev_ctx_lsm6dsl, float *angular_rate_mdps){
	uint8_t reg;

	/* Read output only if new gyro value is available*/
	lsm6dsl_gy_flag_data_ready_get(&dev_ctx_lsm6dsl, &reg);

	if (reg) {
	  /* Read angular rate field data */
	  memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	  lsm6dsl_angular_rate_raw_get(&dev_ctx_lsm6dsl, data_raw_angular_rate);
	  angular_rate_mdps[0] =
		lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
	  angular_rate_mdps[1] =
		lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
	  angular_rate_mdps[2] =
		lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
	}
}

/* LPS22HH Functions ---------------------------------------------------------*/
stmdev_ctx_t lps22hh_init(void){
	stmdev_ctx_t dev_ctx_lps22hh;

	/* Initialize mems driver interface */
	dev_ctx_lps22hh.write_reg = lps22hh_write;
	dev_ctx_lps22hh.read_reg = lps22hh_read;
	dev_ctx_lps22hh.handle = &SENSOR_BUS;


	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	whoamI_lps22hh = 0;
	lps22hh_device_id_get(&dev_ctx_lps22hh, &whoamI_lps22hh);

	if ( whoamI_lps22hh != LPS22HH_ID ){
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		__BKPT();
		Error_Handler();
	}


	/* Restore default configuration */
	lps22hh_reset_set(&dev_ctx_lps22hh, PROPERTY_ENABLE);

	do {
		lps22hh_reset_get(&dev_ctx_lps22hh, &rst_lps22hh);
	} while (rst_lps22hh);

	/* Enable Block Data Update */
	lps22hh_block_data_update_set(&dev_ctx_lps22hh, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lps22hh_data_rate_set(&dev_ctx_lps22hh, LPS22HH_200_Hz);

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


/* Platform Dependent Sensor Read/Write Functions ----------------------------*/

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
static int32_t lsm6dsl_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{

  HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_L, reg,
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
static int32_t lsm6dsl_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
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

  HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_L, reg,
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
  HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_L, reg,
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
