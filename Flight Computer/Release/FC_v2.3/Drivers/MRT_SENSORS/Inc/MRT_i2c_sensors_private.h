/*
 * MRT_i2c_sensors.private.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: Jacoby
 */
//https://www.openstm32.org/forumthread6067

#ifndef MRT_SENSORS_INC_MRT_I2C_SENSORS_PRIVATE_H_
#define MRT_SENSORS_INC_MRT_I2C_SENSORS_PRIVATE_H_

#include <stm32f4xx_hal.h>
#include <lsm6dsr_reg.h>
#include <lps22hh_reg.h>
#include <gps.h>


/*****LSM6DSR*****/
class LSM6DSR {

	public:
		//Data
		float acceleration_mg[3];
		float angular_rate_mdps[3];
		float temperature_degC;

		//Functions
		LSM6DSR(I2C_HandleTypeDef* i2c_bus, uint8_t address);
		void getAcceleration(void);
		void getTemperature(void);
		void getAngularRate(void);

	private:
		//Data
		stmdev_ctx_t ctx;
		int16_t data_raw_acceleration[3];
		int16_t data_raw_angular_rate[3];
		int16_t data_raw_temperature;
		uint8_t whoamI, rst;

		//Functions
		static int32_t write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
		static int32_t read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
};



/*****LPS22HH*****/
class LPS22HH {

	public:
		//Data
		float pressure_hPa;
		float temperature_degC;

		//Functions
		LPS22HH(I2C_HandleTypeDef* i2c_bus, uint8_t address);
		void getPressure(void);
		void getTemperature(void);

	private:
		stmdev_ctx_t ctx;
		lps22hh_reg_t reg;
		uint32_t data_raw_pressure;
		int16_t data_raw_temperature;
		uint8_t whoamI, rst;

		//Functions
		static int32_t write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
		static int32_t read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
};


#endif /* MRT_SENSORS_INC_MRT_I2C_SENSORS_PRIVATE_H_ */
