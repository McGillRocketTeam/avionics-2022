/*
 * i2c_sensor_functions.h
 *
 *  Created on: Dec 22, 2021
 *      Author: JO
 */

#ifndef INC_I2C_SENSOR_FUNCTIONS_H_
#define INC_I2C_SENSOR_FUNCTIONS_H_

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "lsm6dsl_reg.h"
#include "lps22hh_reg.h"
#include "stm32f4xx_hal.h"

#define SENSOR_BUS hi2c3

// lsm6dsl functions
stmdev_ctx_t lsm6dsl_init(void);
void get_acceleration(stmdev_ctx_t dev_ctx_lsm6dsl, float *acceleration_mg);
void get_angvelocity(stmdev_ctx_t dev_ctx_lsm6dsl, float *angular_rate_mdps);

// lps22hh functions
stmdev_ctx_t lps22hh_init(void);
void get_pressure(stmdev_ctx_t dev_ctx_lps22hh, float *pressure);
void get_temperature(stmdev_ctx_t dev_ctx_lps22hh, float *temperature);

#endif /* INC_I2C_SENSOR_FUNCTIONS_H_ */
