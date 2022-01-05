/*
 * MRT_RTOS.h
 *
 *  Created on: Jan 5, 2022
 *      Author: Jacoby
 */

#include "cmsis_os.h"
#include <stm32f4xx_hal.h>

#ifndef INC_MRT_RTOS_H_
#define INC_MRT_RTOS_H_
#endif /* INC_MRT_RTOS_H_ */

typedef struct MRT_RTOS{
	UART_HandleTypeDef huart;
};

extern struct MRT_RTOS rtos;

extern uint8_t flagA;
extern uint8_t flagB;
extern RTC_HandleTypeDef hrtc;

void MRT_SetupRTOS(UART_HandleTypeDef uart, uint8_t defAlarm);


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);

void MRT_WUProcedure(void);
void MRT_ClearFlags(void);
void MRT_StandByMode(uint32_t seconds);
void MRT_DefaultRTC(void);

void MRT_CustomRTC(int values[]);
