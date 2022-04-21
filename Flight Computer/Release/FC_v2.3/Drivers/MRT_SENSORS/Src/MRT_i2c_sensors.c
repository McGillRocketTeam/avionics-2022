/*
 * MRT_i2c_sensors.c
 *
 *  Created on: Jan 30, 2022
 *      Author: Jacoby
 */


#include <MRT_i2c_sensors.h>
#include <MRT_setup.h>
#include <string.h> //memset


// For LSM6DSR
//Public
stmdev_ctx_t hlsm6dsr;
float acceleration_mg[3];
float angular_rate_mdps[3];
float lsm6dsr_temperature_degC;
//Private
int16_t data_raw_acceleration[3];
int16_t data_raw_angular_rate[3];
int16_t lsm_data_raw_temperature;
uint8_t lsm_whoamI, lsm_rst;

// For LPS22HH
//Public
stmdev_ctx_t hlps22hh;
float pressure_hPa;
float lps22hh_temperature_degC;
//Private
uint32_t data_raw_pressure;
int16_t lps_data_raw_temperature;
uint8_t lps_whoamI, lps_rst;


// For GPS
//Public
float gps_latitude;
float gps_longitude;
float gps_time;



//Private functions prototypes
static int32_t lsm_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t lsm_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t lps_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t lps_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);



//**************************************************//
//LSM6DSR

static int32_t lsm_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Write(handle, LSM6DSR_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t lsm_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Read(handle, LSM6DSR_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}


stmdev_ctx_t  MRT_LSM6DSR_Setup(I2C_HandleTypeDef* SENSOR_BUS, uint8_t LSM_ID ) {

	  println("LSM6DSR Setup Starts");

	  stmdev_ctx_t lsm_ctx;

	  /* Initialize mems driver interface */
	  lsm_ctx.write_reg = lsm_write;
	  lsm_ctx.read_reg = lsm_read;
	  lsm_ctx.handle = SENSOR_BUS;
	  /* Wait sensor boot time */
	  HAL_Delay(LSM6DSR_BOOT_TIME);
	  /* Check device ID */
	  lsm6dsr_device_id_get(&lsm_ctx, &lsm_whoamI);

	  print("Checking Sensor ID...");
	  //if (lsm_whoamI != LSM6DSR_ID){ ORIGINAL
	  if (lsm_whoamI != LSM_ID){
		  println("NOT OK");
		  print("This Device is: ");

		  char buffer[10];
		  sprintf(buffer, "%X\r\n", lsm_whoamI);
		  print(buffer);

		  println("\n\rProgram Terminated\n\r");
		  while(1);
	  }
	  println("OK");

	  /* Restore default configuration */
	  lsm6dsr_reset_set(&lsm_ctx, PROPERTY_ENABLE);
	  HAL_Delay(500);
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
	  println("LSM6DSR Setup Ends");

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
		memset(&lsm_data_raw_temperature, 0x00, sizeof(int16_t));
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




//**************************************************//
//LPS22HH

static int32_t lps_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

static int32_t lps_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}


stmdev_ctx_t  MRT_LPS22HH_Setup(I2C_HandleTypeDef* SENSOR_BUS, uint8_t LPS_ID){
	  println("LPS22HH Setup Starts");

	  stmdev_ctx_t lps_ctx;

	  lps22hh_reg_t reg;
	  /* Initialize mems driver interface */
	  lps_ctx.write_reg = lps_write;
	  lps_ctx.read_reg = lps_read;
	  lps_ctx.handle = SENSOR_BUS;
	  /* Wait sensor boot time */
	  HAL_Delay(LPS22HH_BOOT_TIME);
	  /* Check device ID */
	  lps_whoamI = 0;
	  lps22hh_device_id_get(&lps_ctx, &lps_whoamI);


	  print("Checking Sensor ID...");
	  //if ( lps_whoamI != LPS22HH_ID ){ ORIGINAL
	  if ( lps_whoamI != LPS_ID ){
		  println("NOT OK");
		  print("This Device is: ");

		  char buffer[10];
		  sprintf(buffer, "%X\r\n", lps_whoamI);
		  print(buffer);

		  println("\n\rProgram Terminated\n\r");
		  while(1);
	  }
	  println("OK");

	  /* Restore default configuration */
	  lps22hh_reset_set(&lps_ctx, PROPERTY_ENABLE);
	  HAL_Delay(500);
	  do {
	    lps22hh_reset_get(&lps_ctx, &lps_rst);
	  } while (lps_rst);


	  /* Enable Block Data Update */
	  lps22hh_block_data_update_set(&lps_ctx, PROPERTY_ENABLE);

	  /* Set Output Data Rate */
	  lps22hh_data_rate_set(&lps_ctx, LPS22HH_75_Hz_LOW_NOISE);
	  println("LPS22HH Setup Ends");

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
	  memset(&data_raw_pressure, 0x00, sizeof(uint32_t)); //TODO CAN CAUSE AN HARDFAULT
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
	  memset(&lps_data_raw_temperature, 0x00, sizeof(int16_t));
	  lps22hh_temperature_raw_get(&lps_ctx, &lps_data_raw_temperature);
	  *temperature_degC = lps22hh_from_lsb_to_celsius(lps_data_raw_temperature);
	}
}

