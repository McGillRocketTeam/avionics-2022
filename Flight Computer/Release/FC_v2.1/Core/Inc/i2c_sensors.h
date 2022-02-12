/*
 * i2c_sensors.h
 *
 *  Created on: Jan 30, 2022
 *      Author: Jacoby
 */

#ifndef INC_I2C_SENSORS_H_
#define INC_I2C_SENSORS_H_


/*
 * MRT code TODO
 */


//Includes
#include <lsm6dsr_reg.h>
#include <lps22hh_reg.h>
#include <stm32f4xx_hal.h>
//#include <string.h> /* memset */

#define TX_BUF_DIM          256


static float pressure_hPa;
static float lps_temperature_degC;

static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float lsm_temperature_degC;

stmdev_ctx_t MRT_LSM6DSR_Setup(I2C_HandleTypeDef* SENSOR_BUS, UART_HandleTypeDef* uart);
void MRT_LSM6DSR_getAcceleration(stmdev_ctx_t lsm_ctx,float acceleration_mg[3]);
void MRT_LSM6DSR_getAngularRate(stmdev_ctx_t lsm_ctx,float angular_rate_mdps[3]);
void MRT_LSM6DSR_getTemperature(stmdev_ctx_t lsm_ctx,float temperature_degC[1]);

stmdev_ctx_t MRT_LPS22HH_Setup(I2C_HandleTypeDef* SENSOR_BUS, UART_HandleTypeDef* uart);
void MRT_LPS22HH_getPressure(stmdev_ctx_t lps_ctx,float* pressure);
void MRT_LPS22HH_getTemperature(stmdev_ctx_t lps_ctx,float* lps_temperature_degC);





#endif /* INC_I2C_SENSORS_H_ */
