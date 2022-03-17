/*
 * MRT_Helpers.h
 *
 *  Created on: Jan 7, 2022
 *      Author: Jacoby
 */

#ifndef MRT_HELPERS_F4XX_MRT_HELPERS_H_
#define MRT_HELPERS_F4XX_MRT_HELPERS_H_
#endif /* MRT_HELPERS_F4XX_MRT_HELPERS_H_ */

#include "w25qxx.h"
#include "main.h"

/*FLAGS*/
#define NB_OF_FLAGS 3 //Number of flags used
#define FLAGS_OFFSET 0 //Flags offset in sector 1
//Map every variable to their offset in sector 1
#define RESET_FLAG_OFFSET 0
#define WU_FLAG_OFFSET 1
#define IWDG_FLAG_OFFSET 2

//Flags
uint8_t reset_flag; //if 0 -> start from beginning, if 1 -> random watchdog reset (if 2-> reset after wakeup??)
//wu_flag defined in MRT_RTOS.c
uint8_t iwdg_flag; //if 0 -> normal, if 1 -> try to go into standbymode

//Flags read/write buffer
uint8_t flash_flags_buffer[NB_OF_FLAGS];

//Reference list to each flag
uint8_t* flash_flags[NB_OF_FLAGS];


/*RTC time (last recorded)*/
#define RTC_TIME_OFFSET 0 //RTC time offset in sector 2
//Map hours, minutes and seconds to their offsets
#define RTC_HOURS_OFFSET 0
#define RTC_MIN_OFFSET 1
#define RTC_SEC_OFFSET 2

//Time constants (determined at each reset)
uint8_t prev_hours; //Last recorded hours
uint8_t prev_min; //Last recorded minutes
uint8_t prev_sec; //Last recorded seconds

//Time read/write buffer
uint8_t flash_time_buffer[3];

//Reference list to each flag
uint8_t* flash_time[3];


void checkForI2CDevices(UART_HandleTypeDef uart, I2C_HandleTypeDef I2C);
void MRT_externalFlashSetup(UART_HandleTypeDef* uart);
void MRT_freezeWatchDog(void);
void MRT_getFlags(void);
void MRT_resetInfo(UART_HandleTypeDef* uart);
void MRT_saveRTCTime(void);
uint8_t MRT_getContinuity(void);
float MRT_getAltitude(float pressure);

//void tone(uint32_t duration, uint32_t repeats, TIM_HandleTypeDef htim);
