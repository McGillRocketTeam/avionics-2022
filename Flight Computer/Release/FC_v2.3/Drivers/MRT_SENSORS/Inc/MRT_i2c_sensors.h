/*
 * MRT_i2c_sensors.h
 *
 *  Created on: Jan 30, 2022
 *      Author: Jacoby
 */

#ifndef INC_MRT_I2C_SENSORS_H_
#define INC_MRT_I2C_SENSORS_H_


//Includes
#include <lsm6dsr_reg.h>
#include <lps22hh_reg.h>
#include <gps.h>
#include <stm32f4xx_hal.h>


//LSM6DSR
extern stmdev_ctx_t hlsm6dsr;
extern float acceleration_mg[3];
extern float angular_rate_mdps[3];
extern float lsm6dsr_temperature_degC;

stmdev_ctx_t MRT_LSM6DSR_Setup(I2C_HandleTypeDef* SENSOR_BUS, uint8_t LSM_ID);
void MRT_LSM6DSR_getAcceleration(stmdev_ctx_t lsm_ctx,float acceleration_mg[3]);
void MRT_LSM6DSR_getAngularRate(stmdev_ctx_t lsm_ctx,float angular_rate_mdps[3]);
void MRT_LSM6DSR_getTemperature(stmdev_ctx_t lsm_ctx,float temperature_degC[1]);


//LPS22HH
extern stmdev_ctx_t hlps22hh;
extern float pressure_hPa;
extern float lps22hh_temperature_degC;

stmdev_ctx_t MRT_LPS22HH_Setup(I2C_HandleTypeDef* SENSOR_BUS, uint8_t LPS_ID);
void MRT_LPS22HH_getPressure(stmdev_ctx_t lps_ctx,float* pressure);
void MRT_LPS22HH_getTemperature(stmdev_ctx_t lps_ctx,float* lps_temperature_degC);


//GPS
extern float gps_latitude;
extern float gps_longitude;
extern float gps_time;



#endif /* INC_MRT_I2C_SENSORS_H_ */
