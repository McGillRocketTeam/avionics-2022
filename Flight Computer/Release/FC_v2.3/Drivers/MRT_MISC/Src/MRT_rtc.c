/*
 * MRT_rtc.c
 *
 *  Created on: Apr 12, 2022
 *      Author: Jacoby
 */


#include <MRT_setup.h>
#include <MRT_external_flash.h>
#include <rtc.h>



uint8_t flagA = 0; //Dynamic
uint8_t flagB = 0; //Dynamic

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
RTC_AlarmTypeDef sAlarm = {0};

uint8_t int_to_hex_table[60] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9,
								0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
								0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
								0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
								0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
								0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59};


//**************************************************//
//FUNCTIONS

void MRT_rtc_Init(void){

	print("Setting RTC to previous time...");
	MRT_set_rtc(prev_hours,prev_min,prev_sec);
	print("OK\r\n");

	#if ALARM_A_ACTIVE
		print("Setting alarmA...");
		HAL_Delay(2000); //To make sure that when you set the Alarm it doesn't go off automatically
		if (wu_flag == 0){
			MRT_set_alarmA(PRE_WHEN_SLEEP_TIME_HOURS, PRE_WHEN_SLEEP_TIME_MIN, PRE_WHEN_SLEEP_TIME_SEC);
		}
		else{
			MRT_set_alarmA(POST_WHEN_SLEEP_TIME_HOURS, POST_WHEN_SLEEP_TIME_MIN, POST_WHEN_SLEEP_TIME_SEC);
		}
		print("OK\r\n");
	#endif
}


void MRT_check_for_wake_up(void){

	//If WU flag set, wake up procedure
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	{

		wu_flag = 1;

		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  // clear the flag

		print("Wakeup from the STANDBY MODE\r\n");

		/** Disable the WWAKEUP PIN **/
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);  // disable PA0

		/** Deactivate the RTC wakeup  **/
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc); //TODO hrtc from rtc.h (should we pass it as an argument instead?)
	}

	MRT_clear_alarms_flags();
}



void MRT_clear_alarms_flags(void){
	//Must be after alarm A was activated and before going to sleep

	print("Clearing the flags\r\n");

	//Clear alarmA flag
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	while (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != RESET){
		//print("Clearing alarm A flag\r\n");
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
	}
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();


  	//Clear alarmB flag
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	while (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRBF) != RESET){
		//print("Clearing alarm B flag\r\n");
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRBF);
	}
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();

	/* Clear the WU FLAG */
	//print("Clearing wake up flag\r\n");
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/* clear the RTC Wake UP (WU) flag */
	//print("Clearing RTC wake up flag\r\n");
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
}




/*
 * This function is called to enter standby mode
 * @param seconds: time before it wakes up (max of 36 hours)
 */
void MRT_StandByMode(uint32_t seconds){
	print("\r\n/*****StandByMode*****/\r\n");

	/* Enable the WAKEUP PIN
	 * (Needs to be placed BEFORE clearing up the flags or else it wakes up as soon as we enter standby mode)*/
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

	/*Clear the flags so it doesn't wake up as soon as it goes to sleep*/
	MRT_clear_alarms_flags();

	//Setup RTC wake up timer
	println("Setting up RTCW");


	//THIS ONE DOESN'T ALLOW EXTI INTERRUPT BUT WHEN YOU ADD _IT IT DOES (see next one)
	/*
	if (HAL_RTCEx_SetWakeUpTimer(&rtc, 120, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	{
	  Error_Handler();
	}
	*/

	char* msg[30+sizeof(uint32_t)];
	sprintf(msg,"Going to sleep for %i seconds",seconds);
	println(msg);

	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,seconds, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	{
	  Error_Handler();
	}

	HAL_PWR_EnterSTANDBYMode();
}




void MRT_set_rtc(uint8_t h, uint8_t m, uint8_t s){
	  /** Initialize RTC and set the Time and Date
	  */
	  RTC_TimeTypeDef sTime;

	  sTime.Hours = int_to_hex_table[h];
	  sTime.Minutes = int_to_hex_table[m];
	  sTime.Seconds = int_to_hex_table[s];
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
}



void MRT_set_alarmA(uint8_t h, uint8_t m, uint8_t s){
	  /** Enable the Alarm A
	  */
	  sAlarm.AlarmTime.Hours = int_to_hex_table[h];
	  sAlarm.AlarmTime.Minutes = int_to_hex_table[m];
	  sAlarm.AlarmTime.Seconds = int_to_hex_table[s];
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

