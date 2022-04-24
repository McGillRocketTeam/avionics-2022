/*
 * MRT_i2c_sensors_private.cpp
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */



#include <MRT_helpers.h>
#include <MRT_i2c_sensors_private.h>
#include <MRT_setup.h>


//**************************************************//
/*****LSM6DSR*****/

LSM6DSR::LSM6DSR(I2C_HandleTypeDef* i2c_bus, uint8_t address){
	println((char*) "LSM6DSR Setup Starts");

	/* Initialize mems driver interface */
	ctx.write_reg = write;
	ctx.read_reg = read;
	ctx.handle = i2c_bus;
	/* Wait sensor boot time */
	HAL_Delay(LSM6DSR_BOOT_TIME);
	/* Check device ID */
	lsm6dsr_device_id_get(&ctx, &whoamI);

	print((char*) "\tChecking Sensor ID...");
	if (whoamI != address){
	  println((char*) "NOT OK");
	  print((char*) "\tThis Device is: ");

	  char buffer[10];
	  sprintf(buffer, "%X\r\n", whoamI);
	  print(buffer);

	  println((char*) "\n\rProgram Terminated\n\r");
	  while(1);
	}
	println((char*) "OK");

	/* Restore default configuration */
	lsm6dsr_reset_set(&ctx, PROPERTY_ENABLE);
	HAL_Delay(500);
	do {
	lsm6dsr_reset_get(&ctx, &rst);
	} while (rst);

	/* Disable I3C interface */
    lsm6dsr_i3c_disable_set(&ctx, LSM6DSR_I3C_DISABLE); //TODO Jasper

	/* Enable Block Data Update */
	lsm6dsr_block_data_update_set(&ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm6dsr_xl_data_rate_set(&ctx, LSM6DSR_XL_ODR_12Hz5);
	lsm6dsr_gy_data_rate_set(&ctx, LSM6DSR_GY_ODR_12Hz5);
	/* Set full scale */
	lsm6dsr_xl_full_scale_set(&ctx, LSM6DSR_2g);
	lsm6dsr_gy_full_scale_set(&ctx, LSM6DSR_2000dps);
	/* Configure filtering chain(No aux interface)
	* Accelerometer - LPF1 + LPF2 path
	*/
	lsm6dsr_xl_hp_path_on_out_set(&ctx, LSM6DSR_LP_ODR_DIV_100); //TODO JASPER
	lsm6dsr_xl_filter_lp2_set(&ctx, PROPERTY_ENABLE); //TODO JASPER
	println((char*) "\tSetup Ends");

}


void LSM6DSR::getAcceleration(void){
	lsm6dsr_xl_flag_data_ready_get(&ctx, &reg);

	if (reg){
		// Read magnetic field data
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6dsr_acceleration_raw_get(&ctx, data_raw_acceleration);
		acceleration_mg[0] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);
	}
}


void LSM6DSR::getTemperature(void){
    lsm6dsr_temp_flag_data_ready_get(&ctx, &reg);

    if (reg){
		// Read temperature data
		memset(&data_raw_temperature, 0x00, sizeof(int16_t));
		lsm6dsr_temperature_raw_get(&ctx, &data_raw_temperature);
		temperature_degC = lsm6dsr_from_lsb_to_celsius(data_raw_temperature);
	}
}


void LSM6DSR::getAngularRate(void){
    lsm6dsr_gy_flag_data_ready_get(&ctx, &reg);

    if (reg){
		// Read magnetic field data
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6dsr_angular_rate_raw_get(&ctx, data_raw_angular_rate);
		angular_rate_mdps[0] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] = lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);

			/*
		fs250dps_to_mdps
		fs500dps_to_mdps
		fs1000dps_to_mdps
		fs2000dps_to_mdps
		*/
	}
}


int32_t LSM6DSR::write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Write((I2C_HandleTypeDef*) handle, LSM6DSR_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

int32_t LSM6DSR::read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Read((I2C_HandleTypeDef*) handle, LSM6DSR_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}



//**************************************************//
/*****LPS22HH*****/


LPS22HH::LPS22HH(I2C_HandleTypeDef* i2c_bus, uint8_t address){
	println((char*) "LPS22HH Setup Starts");

	/* Initialize mems driver interface */
	ctx.write_reg = write;
	ctx.read_reg = read;
	ctx.handle = i2c_bus;
	/* Wait sensor boot time */
	HAL_Delay(LPS22HH_BOOT_TIME);
	/* Check device ID */
	whoamI = 0;
	lps22hh_device_id_get(&ctx, &whoamI);


	print((char*) "\tChecking Sensor ID...");
	if ( whoamI != address ){
	  println((char*) "NOT OK");
	  print((char*) "\tThis Device is: ");

	  char buffer[10];
	  sprintf(buffer, "%X\r\n", whoamI);
	  print(buffer);

	  println((char*) "\n\rProgram Terminated\n\r");
	  while(1);
	}
	println((char*) "OK");

	/* Restore default configuration */
	lps22hh_reset_set(&ctx, PROPERTY_ENABLE);
	HAL_Delay(500);
	do {
	lps22hh_reset_get(&ctx, &rst);
	} while (rst);


	/* Enable Block Data Update */
	lps22hh_block_data_update_set(&ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lps22hh_data_rate_set(&ctx, LPS22HH_75_Hz_LOW_NOISE);
	println((char*) "\tSetup Ends");

}



void LPS22HH::getPressure(void){
	/* Read output only if new value is available */
	lps22hh_press_flag_data_ready_get(&ctx, &reg);

	if (reg) {
	  memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
	  lps22hh_pressure_raw_get(&ctx, &data_raw_pressure);
	  pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure);
	}
}


void LPS22HH::getTemperature(void){
	/* Read output only if new value is available */
	lps22hh_temp_flag_data_ready_get(&ctx, &reg);

	if (reg) {
	  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	  lps22hh_temperature_raw_get(&ctx, &data_raw_temperature);
	  temperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature);
	}
}



int32_t LPS22HH::write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Write((I2C_HandleTypeDef*) handle, LPS22HH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

int32_t LPS22HH::read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Read((I2C_HandleTypeDef*) handle, LPS22HH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

