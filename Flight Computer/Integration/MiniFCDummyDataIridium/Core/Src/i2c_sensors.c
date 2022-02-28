/*
 * i2c_sensors.c
 *
 *  Created on: Jan 30, 2022
 *      Author: Jacoby
 */



/*
 * MRT code TODO
 */

#include <i2c_sensors.h>


//Variables and defines (functions defined at the end)

#define    BOOT_TIME            100 //ms



// For LSM6DSL
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t lsm_data_raw_temperature;
static uint8_t lsm_whoamI, lsm_rst;

// For LPS22HH
static uint32_t data_raw_pressure;
static int16_t lps_data_raw_temperature;
static uint8_t lps_whoamI, lps_rst;

//For communication
UART_HandleTypeDef* Guart;





//Functions prototypes
static int32_t lsm_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t lsm_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t lps_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t lps_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);


/*
 * LSM6DSR
 */

static int32_t lsm_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSR_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t lsm_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSR_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}





stmdev_ctx_t  MRT_LSM6DSR_Setup(I2C_HandleTypeDef* SENSOR_BUS, UART_HandleTypeDef* uart)
	{
	  Guart = uart;
	  HAL_UART_Transmit(Guart,"LSM6DSR Setup Starts\n\r", 22, HAL_MAX_DELAY);


	  stmdev_ctx_t lsm_ctx;

	  /* Initialize mems driver interface */
	  lsm_ctx.write_reg = lsm_write;
	  lsm_ctx.read_reg = lsm_read;
	  lsm_ctx.handle = SENSOR_BUS;
	  /* Wait sensor boot time */
	  HAL_Delay(BOOT_TIME);
	  /* Check device ID */
	  lsm6dsr_device_id_get(&lsm_ctx, &lsm_whoamI);

	  HAL_UART_Transmit(Guart,"Checking Sensor ID...", 21, HAL_MAX_DELAY);


	  if (lsm_whoamI != LSM6DSR_ID){
		  HAL_UART_Transmit(Guart,"NOT OK\n\r", 8, HAL_MAX_DELAY);
		  HAL_UART_Transmit(Guart,"This Device is: " , 16, HAL_MAX_DELAY);
		  char buffer[10];
		  sprintf(buffer, "%X\r\n", lsm_whoamI);

			__BKPT();

		  HAL_UART_Transmit(Guart,buffer, strlen(buffer), HAL_MAX_DELAY);
		  HAL_UART_Transmit(Guart,"\n\rProgram Terminated\n\r", 22, HAL_MAX_DELAY);
		  while(1);
	  }
	  HAL_UART_Transmit(Guart,"OK\n\r", 6, HAL_MAX_DELAY);

	  /* Restore default configuration */
	  lsm6dsr_reset_set(&lsm_ctx, PROPERTY_ENABLE);


	  HAL_Delay(1000);

	  do {
	    lsm6dsr_reset_get(&lsm_ctx, &lsm_rst);
	  } while (lsm_rst);


	  /* Disable I3C interface */
	  //TODO JASPER lsm6dsr_i3c_disable_set(&lsm_ctx, LSM6DSR_I3C_DISABLE);

	  /* Enable Block Data Update */
	  lsm6dsr_block_data_update_set(&lsm_ctx, PROPERTY_ENABLE);
	  /* Set Output Data Rate */
	  lsm6dsr_xl_data_rate_set(&lsm_ctx, LSM6DSR_XL_ODR_12Hz5);
	  lsm6dsr_gy_data_rate_set(&lsm_ctx, LSM6DSR_GY_ODR_12Hz5);
	  /* Set full scale */
	  lsm6dsr_xl_full_scale_set(&lsm_ctx, LSM6DSR_2g);
	  lsm6dsr_gy_full_scale_set(&lsm_ctx, LSM6DSR_2000dps);
	  /* Configure filtering chain(No aux interface)
	   * Accelerometer - LPF1 + LPF2 path
	   */
	  //TODO JASPER lsm6dsr_xl_hp_path_on_out_set(&lsm_ctx, LSM6DSR_LP_ODR_DIV_100);
	  //TODO JASPER lsm6dsr_xl_filter_lp2_set(&lsm_ctx, PROPERTY_ENABLE);
	  HAL_UART_Transmit(Guart,"LLSM6DSR Setup Ends\n\r", 25, HAL_MAX_DELAY);

	  return lsm_ctx;
	}




/*
 * Get acceleration values
 */
void MRT_LSM6DSR_getAcceleration(stmdev_ctx_t lsm_ctx,float acceleration_mg[3]){
		//lsm6dsr_reg_t reg;
		//lsm6dsr_status_reg_get(&dev_ctx, &reg.status_reg);

    	uint8_t reg;
	    lsm6dsr_xl_flag_data_ready_get(&lsm_ctx, &reg);

		//if (reg.status_reg.gda) {
	    if(reg){
		/* Read magnetic field data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lsm6dsr_acceleration_raw_get(&lsm_ctx, data_raw_acceleration);
        acceleration_mg[0] = lsm6dsr_from_fs2g_to_mg(
                               data_raw_acceleration[0]);
        acceleration_mg[1] = lsm6dsr_from_fs2g_to_mg(
                               data_raw_acceleration[1]);
        acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(
                               data_raw_acceleration[2]);
      }
}



/*
 * Get temperature value
 */
void MRT_LSM6DSR_getTemperature(stmdev_ctx_t lsm_ctx,float* temperature_degC){
	//lsm6dsr_reg_t reg;
	//lsm6dsr_status_reg_get(&dev_ctx, &reg.status_reg);

    uint8_t reg;
    lsm6dsr_temp_flag_data_ready_get(&lsm_ctx, &reg);

	//if (reg.status_reg.tda) {
    if(reg){
		//Read temperature data
		memset(lsm_data_raw_temperature, 0x00, sizeof(int16_t));
		lsm6dsr_temperature_raw_get(&lsm_ctx, &lsm_data_raw_temperature);
		*temperature_degC = lsm6dsr_from_lsb_to_celsius(lsm_data_raw_temperature);

	}
}


/*
 * Get angular rate values
 */
void MRT_LSM6DSR_getAngularRate(stmdev_ctx_t lsm_ctx,float angular_rate_mdps[3]){
		//lsm6dsr_reg_t reg;
		//lsm6dsr_status_reg_get(&dev_ctx, &reg.status_reg);

    	uint8_t reg;
	    lsm6dsr_gy_flag_data_ready_get(&lsm_ctx, &reg);

		//if (reg.status_reg.xlda) {
	    if(reg){
		/* Read magnetic field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6dsr_angular_rate_raw_get(&lsm_ctx, data_raw_angular_rate);
		angular_rate_mdps[0] =
				lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] =
				lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] =
				lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);

			/*
		fs250dps_to_mdps
		fs500dps_to_mdps
		fs1000dps_to_mdps
		fs2000dps_to_mdps
		*/

		}

}




/*
 * LPS22HH
 */


stmdev_ctx_t  MRT_LPS22HH_Setup(I2C_HandleTypeDef* SENSOR_BUS, UART_HandleTypeDef* uart)
	{

	  Guart = uart;
	  HAL_UART_Transmit(Guart,"LPS22HH Setup Starts\n\r", 22, HAL_MAX_DELAY);

	  stmdev_ctx_t lps_ctx;

	  lps22hh_reg_t reg;
	  /* Initialize mems driver interface */
	  lps_ctx.write_reg = lps_write;
	  lps_ctx.read_reg = lps_read;
	  lps_ctx.handle = SENSOR_BUS;
	  /* Wait sensor boot time */
	  HAL_Delay(BOOT_TIME);
	  /* Check device ID */
	  lps_whoamI = 0;
	  lps22hh_device_id_get(&lps_ctx, &lps_whoamI);



  	  HAL_UART_Transmit(Guart,"Checking Sensor ID...", 22, HAL_MAX_DELAY);
	  if ( lps_whoamI != LPS22HH_ID ){
		  HAL_UART_Transmit(Guart,"NOT OK\n\r", 8, HAL_MAX_DELAY);
		  HAL_UART_Transmit(Guart,"This Device is: " , 16, HAL_MAX_DELAY);
		  char buffer[10];
		  sprintf(buffer, "%X\r\n", lps_whoamI);
		  HAL_UART_Transmit(Guart,buffer, strlen(buffer), HAL_MAX_DELAY);
		  HAL_UART_Transmit(Guart,"\n\rProgram Terminated\n\r", 22, HAL_MAX_DELAY);
		  while(1);
	  }
	  HAL_UART_Transmit(Guart,"OK\n\r", 4, HAL_MAX_DELAY);

	  /* Restore default configuration */
	  lps22hh_reset_set(&lps_ctx, PROPERTY_ENABLE);

	  HAL_Delay(1000);

	  do {
	    lps22hh_reset_get(&lps_ctx, &lps_rst);
	  } while (lps_rst);


	  /* Enable Block Data Update */
	  lps22hh_block_data_update_set(&lps_ctx, PROPERTY_ENABLE);
	  /* Set Output Data Rate */
	  lps22hh_data_rate_set(&lps_ctx, LPS22HH_75_Hz_LOW_NOISE);
	  HAL_UART_Transmit(Guart,"LPS22HH Setup Ends\n\r", 24, HAL_MAX_DELAY);

	  return lps_ctx;

	}



void MRT_LPS22HH_getPressure(stmdev_ctx_t lps_ctx,float* pressure){
	/* Read output only if new value is available */
	lps22hh_reg_t reg;
	lps22hh_read_reg(&lps_ctx, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	//uint8_t reg;
	//lps22hh_press_flag_data_ready_get(&lps_ctx, &reg);

	if (reg.status.p_da) {
	//if (reg) {
	  memset(data_raw_pressure, 0x00, sizeof(uint32_t));
	  lps22hh_pressure_raw_get(&lps_ctx, &data_raw_pressure);
	  *pressure = lps22hh_from_lsb_to_hpa(data_raw_pressure);
	}
}

void MRT_LPS22HH_getTemperature(stmdev_ctx_t lps_ctx,float* temperature_degC){
	/* Read output only if new value is available */
	//lps22hh_reg_t reg;
	//lps22hh_read_reg(&lps_ctx, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	uint8_t reg;
	lps22hh_temp_flag_data_ready_get(&lps_ctx, &reg);

	//if (reg.status.t_da) {
	if (reg) {
	  memset(lps_data_raw_temperature, 0x00, sizeof(int16_t));
	  lps22hh_temperature_raw_get(&lps_ctx, &lps_data_raw_temperature);
	  *temperature_degC = lps22hh_from_lsb_to_celsius(lps_data_raw_temperature);
	}
}




static int32_t lps_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

static int32_t lps_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}
