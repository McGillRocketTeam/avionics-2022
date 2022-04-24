/*
 * MRT_i2c_sensors.h
 *
 *  Created on: Jan 30, 2022
 *      Author: Jacoby
 */

#ifndef INC_MRT_I2C_SENSORS_H_
#define INC_MRT_I2C_SENSORS_H_


#ifdef __cplusplus
	extern "C"
	{
#endif

#include <lsm6dsr_reg.h>
#include <lps22hh_reg.h>
#include <gps.h>
#include <stm32f4xx_hal.h>



//**************************************************//
/*****LSM6DSR*****/

struct HLSM6DSR{

	//Data
	float* acceleration_mg[3];
	float* angular_rate_mdps[3];
	float* temperature_degC;

	//Functions
	void (*getAcceleration)(void);
	void (*getAngularRate)(void);
	void (*getTemperature)(void);
};



//**************************************************//
/*****LPS22HH*****/

struct HLPS22HH{

	//Data
	float* pressure_hPa;
	float* temperature_degC;

	//Functions
	void (*getPressure)(void);
	void (*getTemperature)(void);
};




void MRT_i2c_sensors_Init(void);

struct HLSM6DSR MRT_LSM6DSR_Init(void);
extern struct HLSM6DSR hlsm6dsr;

struct HLPS22HH MRT_LPS22HH_Init(void);
extern struct HLPS22HH hlps22hh;



#ifdef __cplusplus
	}
#endif

#endif /* INC_MRT_I2C_SENSORS_H_ */


