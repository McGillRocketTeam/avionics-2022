/*
 * MRT_setup.h
 *
 *  Created on: Mar 22, 2022
 *      Author: Jacoby
 */

#ifndef INC_MRT_SETUP_H_
#define INC_MRT_SETUP_H_


#ifdef __cplusplus
extern "C" {
#endif


//Includes
#include <usart.h> //For uart handler variable
#include <i2c.h>
#include <adc.h>
#include <spi.h>


//**************************************************//
//DEBUGGING
#define DEBUG 0 //If in debug mode, no IWDG
#define DEBUGUART huart8
#define PRINT 1
#define NO_BUZZ 0
#define HARDFAULT_GENERATOR 0

//Testing
#define TESTING_EJECTION 0 //Only testing pressure, not acceleration
#define TESTING_IRIDIUM 0
#define TESTING_SLEEP 0

//FC state
#define FLIGHT_MODE 0 //Put 1 if in flight mode, 0 if not in flight mode (will allow to reset from the start)
#define FORCED_APOGEE 0 //Can only take value of 0 or 1
#define FORCED_EJECTION_STAGE 0 //Can take value from 0 to 1 (boolean)
#define FORCED_STAGE 1 //Can take value from 0 to 4 (only happens if FORCED_EJECTION_STAGE is 1)

//Buzzer
#define BUZZ_SUCCESS_DURATION	75		// ms
#define BUZZ_SUCCESS_REPEATS	2
#define BUZZ_SUCCESS_FREQ		1500	// Hz

#define BUZZ_FAILURE_DURATION	1000 	// ms
#define BUZZ_FAILURE_REPEATS	3
#define	BUZZ_FAILURE_FREQ		220		// Hz



#if DEBUG
#define IWDG_ACTIVE	0
#else
#define IWDG_ACTIVE 1
#endif


//**************************************************//
//CONSTANTS
#define SEA_LEVEL_TEMPERATURE 25.0+273.15 //Sea level temperature in Kelvin
#define SEA_LEVEL_PRESSURE 1014.0 //Sea level pressure hPa
#define  BASE_HEIGHT 1402.08 //In meters (4600ft)
#define M 0.0289644 //Molar mass of earth's air in kg/mol
#define go 9.80665 //Gravitational acceleration constant in m/s^2
#define R 8.31432 //Universal gas constant Nm / mol K




//**************************************************//
//RTC (24h formatting, initial RTC time is always 0 in a wake up)
//TODO Assume Launch at 3pm, sleep from 8pm to 6am
#define ALARM_A_ACTIVE 1
#define ALARM_B_ACTIVE 1


#if TESTING_SLEEP

#define SLEEP_TIME 15 //In seconds

#define PRE_WHEN_SLEEP_TIME_SEC 15  //In seconds
#define PRE_WHEN_SLEEP_TIME_MIN 0  //In minutes
#define PRE_WHEN_SLEEP_TIME_HOURS 0  //In hours

#define POST_WHEN_SLEEP_TIME_SEC 30  //In seconds
#define POST_WHEN_SLEEP_TIME_MIN 0  //In minutes
#define POST_WHEN_SLEEP_TIME_HOURS 0  //In hours

#else
#define SLEEP_TIME 36000 //In seconds (10 hours)

//Alarm times are relative to RTC, which is stopped when asleep
//and continues at the time it went to sleep when waking up

//Launch day sleep time (launch at 3:30pm??)
#define PRE_WHEN_SLEEP_TIME_SEC 0  //In seconds
#define PRE_WHEN_SLEEP_TIME_MIN 30  //In minutes
#define PRE_WHEN_SLEEP_TIME_HOURS 4  //In hours

//Post launch day sleep time (waking up at 6am)
#define POST_WHEN_SLEEP_TIME_SEC ( 0 + PRE_WHEN_SLEEP_TIME_SEC )  //In seconds
#define POST_WHEN_SLEEP_TIME_MIN ( 0 + PRE_WHEN_SLEEP_TIME_MIN )  //In minutes
#define POST_WHEN_SLEEP_TIME_HOURS ( 14 + PRE_WHEN_SLEEP_TIME_HOURS )  //In hours
#endif



//**************************************************//
//THREADS
#define MEMORY_THREAD 1
#define EJECTION_THREAD 1
#define TELEMETRY_THREAD 1
#define SENSORS_THREAD 1
#define PROPULSION_THREAD 1
#define WATCHDOG_THREAD 1

#define NUMBER_OF_THREADS 5 //This excludes the watch dog thread


//**************************************************//
//MEMORY THREAD
#define SD_CARD_	1
#define SD_SPI_HANDLE hspi5
#define DATA_FREQ 10 //Times per second that you want to save data
#define POST_LANDED_DATA_FREQ 1 //Times per second that you want to save data



//**************************************************//
//EJECTION THREAD
//TODO Ejection (just invented variables for the sake of testing)

#define MAIN_DEPLOY_ALT 152.4 //In meters (1500 ft)
#define DROGUE_TO_MAIN_DELAY 4000 //In milli-seconds

//TODO might need a bigger range to account for errors (gotta know what we expect to be our slowest descent speed)
#define LANDING_DIFF_LIMIT 1 //The difference in altitude that needs to be observed to update the landing counter

#define EJECTION_FREQ 10

//For acceleration
#define ACC_LIMIT 1
#define ACC_COUNTER_THRESH 10 //Used to be 30, but the update rate is not fast enough

//For LSL in regression
#define NUM_MEAS_REG 10 //Used to be 50, but the update rate is not fast enough
#define LSL_SLOPE_LIMIT -0.005
#define LSL_COUNTER_THRESHOLD 10


//Not used (other variables used with same values)
#define PAD_STATE 0
#define BOOST_STATE 1
#define DROGUE_STATE 2
#define MAIN_STATE 3
#define LANDED_STATE 4




//**************************************************//
//TELEMETRY_THREAD
#define PRE_APOGEE_SEND_FREQ 100 //Times per second that you want to transmit data before apogee
#define POST_APOGEE_SEND_FREQ 100 //Times per second that you want to transmit data after apogee
#define POST_LANDED_SEND_FREQ 10 //Times per second that you want to transmit data after landing
#define SENSORS_SEND_FREQ_DIVIDER 5 //Number of times that you want to send sensors data relative to propulsion data
									 //(can only be equal or slower). Divides the XXXX_APOGEE_SEND_FREQ

#define HALF_BYTE_ 0

//SRadio
#define SRADIO_ 1
#define SRADIO_SPI hspi2
#define SRADIO_BUFFER_SIZE 256

//XTend
#define XTEND_ 0
#define XTEND_UART huart3
#define XTEND_BUFFER_SIZE 256
#define XTEND_TIMEOUT 0x500

#if XTEND_
#define RADIO_BUFFER_SIZE	XTEND_BUFFER_SIZE
#elif SRADIO_
#define RADIO_BUFFER_SIZE	SRADIO_BUFFER_SIZE
#else
#define RADIO_BUFFER_SIZE	256
#endif

//Iridium
#define IRIDIUM_ 1
#define IRIDIUM_I2C 2 //I2C bus number
#define IRIDIUM_BUFFER_SIZE 10//Keep in mind a 1 credit per 50 bytes message (does it take into account the terminating byte of a string?)
//#define IRIDIUM_FLIGHT_TIMEOUT 1 //Timeout in seconds TODO ONLY TESTED WITH GET TIME
#define IRIDIUM_FLIGHT_TIMEOUT 90 //Timeout in seconds TODO ONLY TESTED WITH GET TIME
#define IRIDIUM_LANDED_TIMEOUT 90
#define IRIDIUM_WAIT_TIME 100 //in milliseconds

#define IRIDIUM_INTERNAL_PRINT 1



//**************************************************//
//SENSORS THREAD
#define PRE_APOGEE_POLL_FREQ 1000 //Times per second that data is polled (pre apogee)
#define POST_APOGEE_POLL_FREQ 1000 //Times per second that data is polled (post apogee)
#define POST_LANDED_POLL_FREQ 10 //Times per second that data is polled (post landing)

//I2C
#define CHECK_I2C 0

//GPS
#define GPS_	1
#define GPS_UART huart6
#define GPS_DATA_BUF_DIM 100

//LSM6DSR
#define LSM6DSR_	1
#define LSM6DSR_I2C hi2c3
#define LSM6DSR_BOOT_TIME 100 //ms
#define MRT_LSM6DSR_ADDRESS	0x6A //Address on i2c bus

//LPS22HH
#define LPS22HH_	1
#define LPS22HH_I2C hi2c3
#define LPS22HH_BOOT_TIME 100 //ms
#define MRT_LPS22HH_ADDRESS	0xB3U //Address on i2c bus




//**************************************************//
//WATCHDOG THREAD
#define WD_FREQ 0.4 //Times per second that you want to check threads
#define WD_BUFFER_SIZE 256//WD thread buffer size to print

#if PRINTING_THREAD
#define THREAD_KEEPER 0 //If you want to check the thread states of no, can't have it if printing thread is there
#endif

#ifndef THREAD_KEEPER
#define THREAD_KEEPER 1 //If you want to check the thread states of no
#endif

#if !IWDG_ACTIVE
#define HAL_IWDG_Refresh(iwdg_handler)	0  //This function now does nothing
#endif




//**************************************************//
//PROPULSION THREAD
#define PAYLOAD_ 1
#define TRANSDUCER_ADC	hadc1

//PAYLOAD
#define TEENSY_ADDRESS 0x01 << 1
#define DATA_REG 0x02 //Don't know it
#define PAYLOAD_BUFFER_SIZE 50

//**************************************************//



//Public Function Prototypes
void MRT_Init(void);
void MRT_Deinit(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_MRT_SETUP_H_ */
