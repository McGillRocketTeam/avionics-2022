/*
 * MRT_RTOS.c
 *
 *  Created on: Jan 5, 2022
 *      Author: Jacoby
 */

#include <MRT_RTOS.h>

RTC_HandleTypeDef hrtc;

struct MRT_RTOS rtos;


uint8_t flagA = 0;
uint8_t flagB = 0;

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
RTC_AlarmTypeDef sAlarm = {0};

/*You cannot put these in the user callbacks section and I don't know why (can put in user begin 4)*/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	HAL_UART_Transmit(&(rtos.huart),(uint8_t*)"AlarmA\r\n", 8, HAL_MAX_DELAY);
	flagA = 1;
}

void HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc){
	HAL_UART_Transmit(&(rtos.huart),(uint8_t*)"AlarmB\r\n", 8, HAL_MAX_DELAY);
	flagB = 1;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){
}




void MRT_SetupRTOS(UART_HandleTypeDef uart,uint8_t sleepT , uint8_t defAlarm){
	HAL_UART_Transmit(&(rtos.huart),"Setting up RTOS\r\n", 17, HAL_MAX_DELAY);
	rtos.huart = uart;
	rtos.sleepTime = sleepT;
	MRT_WUProcedure();
	if (defAlarm==1) MRT_DefaultRTC();
}



void MRT_WUProcedure(void){

	//If WU flag set, wake up procedure
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	{
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  // clear the flag

		/** display  the string **/
		char *str = "Wakeup from the STANDBY MODE\r\n";
		HAL_UART_Transmit(&(rtos.huart), (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

		/** Disable the WWAKEUP PIN **/
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);  // disable PA0

		/** Deactivate the RTC wakeup  **/
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	}

	MRT_ClearFlags();
}

void MRT_ClearFlags(void){
	//Must be after alarm A was activated and before going to sleep

	HAL_UART_Transmit(&(rtos.huart),"Clearing the flags\r\n", 20, HAL_MAX_DELAY);
	  	//Clear alarmA flag
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	while (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != RESET){
		HAL_UART_Transmit(&(rtos.huart),"Clearing alarm A flag\r\n", 23, HAL_MAX_DELAY);
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
	}
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();


  	//Clear alarmB flag
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	while (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRBF) != RESET){
		HAL_UART_Transmit(&(rtos.huart),"Clearing alarm B flag\r\n", 23, HAL_MAX_DELAY);
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRBF);
	}
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();


	/* Clear the WU FLAG */
	//HAL_UART_Transmit(&(rtos.huart),"Clearing WU flag\r\n", 18, HAL_MAX_DELAY);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/* clear the RTC Wake UP (WU) flag */
	//HAL_UART_Transmit(&(rtos.huart),"Clearing RTC WU flag\r\n", 22, HAL_MAX_DELAY);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
}


/*
 * This function is called to enter standby mode
 * @param seconds: time before it wakes up (max of 36 hours)
 */
void MRT_StandByMode( uint32_t seconds){
	HAL_UART_Transmit(&(rtos.huart),"\r\nStandByMode\r\n", 15, HAL_MAX_DELAY);

	/* Enable the WAKEUP PIN
	 * (Needs to be placed BEFORE clearing up the flags or else it wakes up as soon as we enter standby mode)*/
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

	/*Clear the flags so it doesn't wake up as soon as it goes to sleep*/
	MRT_ClearFlags();

	//Setup RTC wake up timer
	HAL_UART_Transmit(&(rtos.huart),"Setting up RTCW\r\n", 17, HAL_MAX_DELAY);


	//THIS ONE DOESN'T ALLOW EXTI INTERRUPT BUT WHEN YOU ADD _IT IT DOES (see next one)
	/*
	if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 120, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	{
	  Error_Handler();
	}
	*/

	char* msg[30+sizeof(uint32_t)];
	sprintf(msg,"Going to sleep for %i seconds\r\n",seconds);
	HAL_UART_Transmit(&(rtos.huart), msg,strlen(msg),HAL_MAX_DELAY);

	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,seconds, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	{
	  Error_Handler();
	}

	HAL_PWR_EnterSTANDBYMode();
}


void MRT_DefaultRTC(void){

	/*Can be setup using the ioc files*/

	  HAL_UART_Transmit(&(rtos.huart),"Initializing default RTC\r\n", 26, HAL_MAX_DELAY);
	  /** Initialize RTC and set the Time and Date
	  */
	  sTime.Hours = 0x0;
	  sTime.Minutes = 0x0;
	  sTime.Seconds = 0x15;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  sDate.Month = RTC_MONTH_JANUARY;
	  sDate.Date = 0x1;
	  sDate.Year = 0x0;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }


	  /** Enable the Alarm A
	  */
	  sAlarm.AlarmTime.Hours = 0x0;
	  sAlarm.AlarmTime.Minutes = 0x0;
	  sAlarm.AlarmTime.Seconds = 0x20;
	  sAlarm.AlarmTime.SubSeconds = 0x0;
	  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	  sAlarm.AlarmDateWeekDay = 0x1;
	  sAlarm.Alarm = RTC_ALARM_A;
	  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
}




void MRT_CustomRTC(int values[]){
	//TODO manually implement a function for custom RTC clock/alarm or simply use default clocks
}


void MRT_SetSleepTime(uint8_t sleepT){
	rtos.sleepTime=sleepT;
}
