/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_memory.h>

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
RTC_AlarmTypeDef sAlarm = {0};

uint8_t int_to_hex_table[60] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9,
								0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
								0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
								0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
								0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
								0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59};

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
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
  sAlarm.AlarmTime.Minutes = 0x1;
  sAlarm.AlarmTime.Seconds = 0x0;
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
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


//**************************************************//
//FUNCTIONS

void MRT_rtc_Init(void){
	println("\r\nMRT RTC Init");

	print("\tSetting RTC to previous time...");
	MRT_set_rtc(prev_hour,prev_min,prev_sec);
	println("OK");

	#if ALARM_A_ACTIVE
		print("\tSetting alarmA...");
		HAL_Delay(2000); //To make sure that when you set the Alarm it doesn't go off automatically
		if (wu_flag == 0){
			MRT_set_alarmA(PRE_WHEN_SLEEP_TIME_HOURS, PRE_WHEN_SLEEP_TIME_MIN, PRE_WHEN_SLEEP_TIME_SEC);
		}
		else{
			MRT_set_alarmA(POST_WHEN_SLEEP_TIME_HOURS, POST_WHEN_SLEEP_TIME_MIN, POST_WHEN_SLEEP_TIME_SEC);
		}
		println("OK");
	#endif
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

	char msg[30+sizeof(uint32_t)];
	sprintf(msg,"Going to sleep for %i seconds",(int) seconds);
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




//**************************************************//
//BACKUP REGISTERS

//Backup registers global variables

//Flags
uint32_t rtc_bckp_reg_reset = 0;
uint32_t rtc_bckp_reg_wu = 0;
uint32_t rtc_bckp_reg_iwdg = 0;
uint32_t rtc_bckp_reg_apogee = 0;
uint32_t rtc_bckp_reg_ejection_stage = 0;

//Time variables
uint32_t rtc_bckp_reg_hour = 0; //Last recorded hours
uint32_t rtc_bckp_reg_min = 0; //Last recorded minutes
uint32_t rtc_bckp_reg_sec = 0; //Last recorded seconds
uint32_t rtc_bckp_reg_subsec = 0; //Last recorded subseconds


//FC states
uint32_t rtc_bckp_reg_vr_power = 0;
uint32_t rtc_bckp_reg_vr_rec = 0;

uint32_t rtc_bckp_reg_alt_pad = 0; // floats will be rounded to int
uint32_t rtc_bckp_reg_pad_time = 0;
uint32_t rtc_bckp_reg_alt_true_apogee = 0;
uint32_t rtc_bckp_reg_true_apogee_time = 0;
uint32_t rtc_bckp_reg_alt_apogee = 0;
uint32_t rtc_bckp_reg_apogee_time = 0;
uint32_t rtc_bckp_reg_alt_main = 0;
uint32_t rtc_bckp_reg_main_time = 0;
uint32_t rtc_bckp_reg_alt_landed = 0;
uint32_t rtc_bckp_reg_landed_time = 0;


//Reference list to each time component (in the same order than typedef enum rtc_backup_reg
uint32_t* rtc_bckp_regs[NB_RTC_BCKP_REGS] = {&rtc_bckp_reg_reset, &rtc_bckp_reg_wu, &rtc_bckp_reg_iwdg, &rtc_bckp_reg_apogee, &rtc_bckp_reg_ejection_stage,
							  &rtc_bckp_reg_hour, &rtc_bckp_reg_min, &rtc_bckp_reg_sec, &rtc_bckp_reg_subsec,
				//TODO		      &rtc_bckp_reg_vr_power, &rtc_bckp_reg_vr_rec,
							  &rtc_bckp_reg_alt_pad, &rtc_bckp_reg_pad_time, &rtc_bckp_reg_alt_true_apogee, &rtc_bckp_reg_true_apogee_time,
							  &rtc_bckp_reg_alt_apogee, &rtc_bckp_reg_apogee_time, &rtc_bckp_reg_alt_main, &rtc_bckp_reg_main_time,
							  &rtc_bckp_reg_alt_landed, &rtc_bckp_reg_landed_time};


//Get all the backup regs values (initialization)
void MRT_RTC_backup_regs_Init(void){
	for (int i = 0; i < NB_RTC_BCKP_REGS; i++){
		*rtc_bckp_regs[i] = MRT_RTC_getBackupReg(i);
	}

	//TODO TESTING
	char buffer[256];
	sprintf(buffer, "ALTITUDES:\r\n\tGround: %i \tTime: %i"
					"\r\n\tTrue Apogee: %i \tTime: %i"
					"\r\n\tApogee: %i \tTime: %i"
					"\r\n\tMain: %i \tTime: %i"
					"\r\n\tLanded: %i \tTime: %i\r\n",
			rtc_bckp_reg_alt_pad, rtc_bckp_reg_pad_time,  rtc_bckp_reg_alt_true_apogee, rtc_bckp_reg_true_apogee_time,
			rtc_bckp_reg_alt_apogee, rtc_bckp_reg_apogee_time, rtc_bckp_reg_alt_main, rtc_bckp_reg_main_time,
			rtc_bckp_reg_alt_landed, rtc_bckp_reg_landed_time);
	print(buffer);

	HAL_Delay(2000);
}

// initializes backup register values to zero
void MRT_RTC_clearBackupRegs(void) {
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	for (uint8_t i = 0; i < 20; i++) {
		HAL_RTCEx_BKUPWrite(&hrtc, i, 0);	// set all backup register values to zero
	}
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
}

// gets the backup register value for the specified state
uint32_t MRT_RTC_getBackupReg(rtc_backup_reg state) {
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	uint32_t val = HAL_RTCEx_BKUPRead(&hrtc, (uint32_t) state);
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	return val;
}

// sets the backup register value for the specified state
void MRT_RTC_setBackupReg(rtc_backup_reg state, uint32_t value) {
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	HAL_RTCEx_BKUPWrite(&hrtc, (uint32_t) state, value);
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
