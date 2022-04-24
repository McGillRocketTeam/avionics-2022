/*
 * MRT_i2c_sensors.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: Jacoby
 */

//**************************************************//
/*****C STATIC FUNCTIONS FILE*****/

#ifdef __cplusplus
extern "C" {
#endif

#include <MRT_i2c_sensors.h>
#include <MRT_i2c_sensors_private.h>
#include <MRT_setup.h>

//Private functions prototypes
void MRT_LSM6DSR_Constructor();
void MRT_LSM6DSR_Destructor();
void MRT_LSM6DSR_getAcceleration(void);
void MRT_LSM6DSR_getAngularRate(void);
void MRT_LSM6DSR_getTemperature(void);
void MRT_LPS22HH_Constructor();
void MRT_LPS22HH_Destructor();
void MRT_LPS22HH_getPressure(void);
void MRT_LPS22HH_getTemperature(void);

//Declare c++ objects
static LSM6DSR* lsm6dsr = NULL;
static LPS22HH* lps22hh = NULL;

//Declare c structs
struct HLSM6DSR hlsm6dsr;
struct HLPS22HH hlps22hh;


//**************************************************//
/*****LSM6DSR STATIC FUNCTIONS*****/

void MRT_LSM6DSR_Constructor(){
	if (lsm6dsr==NULL){
		lsm6dsr = new LSM6DSR(&LSM6DSR_I2C, (uint8_t) MRT_LSM6DSR_ADDRESS);
	}
}

void MRT_LSM6DSR_Destructor(){
	if (lsm6dsr!=NULL){
		lsm6dsr = NULL;
	}
}

void MRT_LSM6DSR_getAcceleration(void){
	lsm6dsr->getAcceleration();
	hlsm6dsr.acceleration_mg[0] = lsm6dsr->acceleration_mg[0];
	hlsm6dsr.acceleration_mg[1] = lsm6dsr->acceleration_mg[1];
	hlsm6dsr.acceleration_mg[2] = lsm6dsr->acceleration_mg[2];
}
void MRT_LSM6DSR_getAngularRate(void){
	lsm6dsr->getAngularRate();
	hlsm6dsr.angular_rate_mdps[0] = lsm6dsr->angular_rate_mdps[0];
	hlsm6dsr.angular_rate_mdps[1] = lsm6dsr->angular_rate_mdps[1];
	hlsm6dsr.angular_rate_mdps[2] = lsm6dsr->angular_rate_mdps[2];
}
void MRT_LSM6DSR_getTemperature(void){
	lsm6dsr->getTemperature();
	hlsm6dsr.temperature_degC = lsm6dsr->temperature_degC;
}



//**************************************************//
/*****LPS22HH STATIC FUNCTIONS*****/

void MRT_LPS22HH_Constructor(){
	if (lps22hh==NULL){
		lps22hh = new LPS22HH(&LPS22HH_I2C, (uint8_t) MRT_LPS22HH_ADDRESS);
	}
}

void MRT_LPS22HH_Destructor(){
	if (lps22hh!=NULL){
		lps22hh = NULL;
	}
}

void MRT_LPS22HH_getPressure(void){
	lps22hh->getPressure();
	hlps22hh.pressure_hPa = lps22hh->pressure_hPa;
}

void MRT_LPS22HH_getTemperature(void){
	lps22hh->getTemperature();
	hlps22hh.temperature_degC = lps22hh->temperature_degC;
}





//**************************************************//
/*****PUBLIC STATIC FUNCTIONS*****/

HLSM6DSR MRT_LSM6DSR_Init(void){
	MRT_LSM6DSR_Constructor();
	HLSM6DSR lsm6dsr_handler;
	lsm6dsr_handler.getAcceleration = &MRT_LSM6DSR_getAcceleration;
	lsm6dsr_handler.getAngularRate = &MRT_LSM6DSR_getAngularRate;
	lsm6dsr_handler.getTemperature = &MRT_LSM6DSR_getTemperature;
	return lsm6dsr_handler;
}

struct HLPS22HH MRT_LPS22HH_Init(void){
	MRT_LPS22HH_Constructor();
	struct HLPS22HH lps22hh_handler;
	lps22hh_handler.getPressure = &MRT_LPS22HH_getPressure;
	lps22hh_handler.getTemperature = &MRT_LPS22HH_getTemperature;
	return lps22hh_handler;
}

void MRT_i2c_sensors_Init(void){

	//LSM6DSR
	#if LSM6DSR_
	HAL_IWDG_Refresh(&hiwdg);
	hlsm6dsr = MRT_LSM6DSR_Init();
	#endif

	//LPS22HH
	#if LPS22HH_
	HAL_IWDG_Refresh(&hiwdg);
	hlps22hh = MRT_LPS22HH_Init();
	#endif

	//GPS
	#if GPS_
	HAL_IWDG_Refresh(&hiwdg);
	//GPS_Init(&GPS_UART, &DEBUG_UART);TODO change to that?
	GPS_Init(&GPS_UART);
	#endif

}



#ifdef __cplusplus
}
#endif






