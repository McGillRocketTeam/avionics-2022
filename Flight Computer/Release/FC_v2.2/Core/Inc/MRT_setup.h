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
//#define HAL_UART_Transmit(u, b, l, d) 0 STOPS PRINTING
#endif

UART_HandleTypeDef huart8;
#define DEBUG_UART huart8

#define FORCED_APOGEE 0 //Can only take value of 0 or 1
#define FORCED_EJECTION_STAGE 0 //Can take value from 0 to 1 (boolean)
#define FORCED_STAGE 1 //Can take value from 0 to 4 (only happens if FORCED_EJECTION_STAGE is 1)


//Buzzer
#define BUZZ_SUCCESS_DURATION	75		// ms
#define BUZZ_SUCCESS_REPEATS	2
#define BUZZ_SUCCESS_FREQ		1046	// Hz

#define BUZZ_FAILURE_DURATION	1000 	// ms
#define BUZZ_FAILURE_REPEATS	3
#define	BUZZ_FAILURE_FREQ		220		// Hz


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
#define EJECTION_THREAD 1
#define TELEMETRY_THREAD 0
#define SENSORS_THREAD 1
#define WATCHDOG_THREAD 1

#define PRINTING_THREAD 0




//**************************************************//
//MEMORY THREAD
#define SD_SPI_HANDLE hspi5
#define DATA_FREQ 10 //Times per second that you want to save data



//**************************************************//
//EJECTION THREAD
//TODO Ejection (just invented variables for the sake of testing)
#define MIN_APOGEE 120
#define MAX_APOGEE 200
#define DEPLOY_ALT_MIN 60
#define DEPLOY_ALT_MAX 65
#define GROUND_LEVEL 20


#define PAD_STATE 0
#define BOOST_STATE 1
#define DROGUE_STATE 2
#define MAIN_STATE 3
#define LANDED_STATE 4




//**************************************************//
//TELEMETRY_THREAD
#define PRE_APOGEE_SEND_FREQ 50 //Times per second that you want to transmit data before apogee
#define POST_APOGEE_SEND_FREQ 100 //Times per second that you want to transmit data after apogee
#define SENSORS_SEND_FREQ_DIVIDER 10 //Number of times that you want to send sensors data relative to propulsion data
									 //(can only be equal or slower). Divides the XXXX_APOGEE_SEND_FREQ
#define IRIDIUM_SEND_FREQ_DIVIDER 5 //Number of times that you want to send iridium data relative to sensors data
									//(can only be equal or slower). Divides the XXXX_APOGEE_SEND_FREQ after it has been divided
									//by SENSORS_SEND_FREQ_DIVIDER

//SRadio
#define SRADIO_ 0
#define SRADIO_SPI hspi2
#define SRADIO_BUFFER_SIZE 256

//XTend
#define XTEND_ 0
#define XTEND_UART huart3
#define XTEND_BUFFER_SIZE 256

//Iridium
#define IRIDIUM_ 0
#define IRIDIUM_I2C 2 //I2C bus number
#define IRIDIUM_TIMEOUT 5 //Timeout in seconds

#if XTEND_
#define RADIO_BUFFER_SIZE XTEND_BUFFER_SIZE
#elif SRADIO_
#define RADIO_BUFFER_SIZE SRADIO_BUFFER_SIZE
#else
#define RADIO_BUFFER_SIZE 256
#endif


//**************************************************//
//SENSORS THREAD
#define PRE_APOGEE_POLL_FREQ 50 //Times per second that you want to poll data (pre apogee)
#define POST_APOGEE_POLL_FREQ 100 //Times per second that you want to poll data (post apogee)
#define SENSORS_POLL_FREQ_DIVIDER 10 //Number of times that you want to poll sensors data relative to propulsion data
									//(can only be equal or slower). Divides the XXXX_APOGEE_POLL_FREQ

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
#define NUMBER_OF_THREADS 4
#define WD_FREQ 1 //Times per second that you want to check threads

#if PRINTING_THREAD
#define THREAD_KEEPER 0 //If you want to check the thread states of no, can't have it if printing thread is there
#endif

#ifndef THREAD_KEEPER
#define THREAD_KEEPER 1 //If you want to check the thread states of no
#endif




//**************************************************//
//PRINTING THREAD




//**************************************************//



#endif /* INC_MRT_SETUP_H_ */
