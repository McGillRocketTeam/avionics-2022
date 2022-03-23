/*
 * MRT_setup.h
 *
 *  Created on: Mar 22, 2022
 *      Author: Jacoby
 */

#ifndef INC_MRT_SETUP_H_
#define INC_MRT_SETUP_H_


//**************************************************//
//DEBUGGING
#define DEBUG 0

#if DEBUG
#define IWDG_ACTIVE 0
#else
#define IWDG_ACTIVE 1
#endif

UART_HandleTypeDef huart8;
#define DEBUG_UART huart8



//**************************************************//
//Constants
#define SEA_LEVEL_TEMPERATURE 25+273.15 //Sea level temperature in Kelvin
#define SEA_LEVEL_PRESSURE 1014 //Sea level pressure hPa
#define  BASE_HEIGHT 100 //In meters
#define M 0.0289644 //Molar mass of earth's air in kg/mol
#define go 9.80665 //Gravitational acceleration constant in m/s^2
#define R 8.31432 //Universal gas constant Nm / mol K




//**************************************************//
//RTC (24h formatting, initial RTC time is always 0 in a wake up)
//TODO Assume Launch at 3pm, sleep from 8pm to 6am
#define ALARM_A_ACTIVE 1
#define SLEEP_TIME 36000 //In seconds (10 hours)

//Launch day sleep time (launch at 3:30pm)
#define PRE_WHEN_SLEEP_TIME_SEC 0  //In seconds
#define PRE_WHEN_SLEEP_TIME_MIN 30  //In minutes
#define PRE_WHEN_SLEEP_TIME_HOURS 4  //In hours

//Post launch day sleep time (waking up at 6am)
#define POST_WHEN_SLEEP_TIME_SEC 0  //In seconds
#define POST_WHEN_SLEEP_TIME_MIN 0  //In minutes
#define POST_WHEN_SLEEP_TIME_HOURS 14  //In hours




//**************************************************//
//THREADS
#define MEMORY_THREAD 1
#define EJECTION_THREAD 0
#define TELEMETRY_THREAD 1
#define SENSORS_THREAD 1
#define WATCHDOG_THREAD 1

#define PRINTING_THREAD 0




//**************************************************//
//MEMORY THREAD
#define SD_SPI_HANDLE hspi5




//**************************************************//
//EJECTION THREAD
//TODO Ejection (just invented variables for the sake of testing)
#define MIN_APOGEE 20
#define MAX_APOGEE 30
#define DEPLOY_ALT_MIN 60
#define DEPLOY_ALT_MAX 65
#define GROUND_LEVEL 20




//**************************************************//
//TELEMETRY_THREAD
#define DATA_FREQ 10 //Times per second that you want to save data
#define SEND_FREQ 20 //Times per second that you want to transmit data

//SRadio
#define SRADIO_ 0
#define SRADIO_SPI hspi2
#define SRADIO_BUFFER_SIZE 256

//XTend
#define XTEND_ 1
#define XTEND_UART huart3
#define XTEND_BUFFER_SIZE 256

//Iridium
#define IRIDIUM_ 1
//#define IRIDIUM_I2C I2C2 //TODO defined in the IridiumSBD.h




//**************************************************//
//SENSORS THREAD
#define POLL_FREQ 20 //Times per second that you want to poll data

//I2C
#define CHECK_I2C 0

//GPS
#define GPS_UART huart6
#define GPS_DATA_BUF_DIM 100

//LSM6DSR
#define LSM_I2C hi2c3

//LPS22HH
#define LPS_I2C hi2c3




//**************************************************//
//WATCHDOG THREAD
#define WD_FREQ 1 //Times per second that you want to check threads




//**************************************************//
//PRINTING THREAD




//**************************************************//



#endif /* INC_MRT_SETUP_H_ */
