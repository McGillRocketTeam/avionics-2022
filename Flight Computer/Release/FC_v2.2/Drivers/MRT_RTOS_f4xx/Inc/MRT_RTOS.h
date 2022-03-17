/*
 * MRT_RTOS.h
 *
 *  Created on: Jan 5, 2022
 *      Author: Jacoby
 */

/*TODO The only things left to do are
 * -Activate RTC, calendar and internal alarm A (don't forget to activate NVIC EXTI)
 * -Activate freeRTOS
 * -Define what you want in the alarms callback functions
 * -(Optional) Setup alarm A and the clock time in .ioc
 * The rest have been taken care of
 */

#include "cmsis_os.h"
#include <stm32f4xx_hal.h>
#include <MRT_Helpers.h>

#ifndef INC_MRT_RTOS_H_
#define INC_MRT_RTOS_H_
#endif /* INC_MRT_RTOS_H_ */

//Structure to not have to worry about huart
typedef struct MRT_RTOS{
	UART_HandleTypeDef huart;
	uint8_t sleepTime;
};


/*
 * Extern variables
 */
extern struct MRT_RTOS rtos; //Only used for huart
extern uint8_t flagA; //Alarm A usable flag
extern uint8_t flagB; //Alarm B usable flag
extern uint8_t wu_flag;


//For RTC
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
RTC_AlarmTypeDef sAlarm;


//Helper functions
void MRT_WUProcedure(void);
void MRT_ClearFlags(void);
void MRT_DefaultRTC(void);


//User functions
void MRT_SetupRTOS(RTC_HandleTypeDef* hrtc, UART_HandleTypeDef uart,uint8_t sleepT);
void MRT_SetSleepTime(uint8_t sleepT);
void MRT_StandByMode(uint32_t seconds);
void MRT_setAlarmA(uint8_t h, uint8_t m, uint8_t s);
void MRT_setRTC(uint8_t h, uint8_t m, uint8_t s);
