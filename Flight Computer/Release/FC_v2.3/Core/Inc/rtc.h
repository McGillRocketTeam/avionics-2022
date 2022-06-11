/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern RTC_AlarmTypeDef sAlarm;

//**************************************************//
//FUNCTIONS PROTOTYPES
void MRT_rtc_Init(void);

void MRT_clear_alarms_flags(void);
void MRT_StandByMode(uint32_t seconds);

void MRT_set_rtc(uint8_t h, uint8_t m, uint8_t s);
void MRT_set_alarmA(uint8_t h, uint8_t m, uint8_t s);
void MRT_set_alarmB(uint8_t h, uint8_t m, uint8_t s);


//Backup registers

#define NB_RTC_BCKP_REGS	19 //Max of 20

typedef enum rtc_backup_reg {
	// for simplicity, put them all in separate backup registers
	// if we ever need more, we can pack the bits

	//FC state (essentials)
	FC_STATE_RESET, //if 0 -> start from beginning, if 1 -> random watchdog reset (if 2-> reset after wakeup??)
	FC_STATE_WU, //TODO used to be defined in MRT_RTOS.c
	FC_STATE_IWDG, //if 0 -> normal, if 1 -> try to go into standbymode
	FC_STATE_APOGEE, //if 0 -> pre-apogee, if 1 -> post-apogee
	FC_STATE_FLIGHT, //Also known as ejection_stage_flag

	//RTC time
	RTC_HOUR,
	RTC_MINUTE,
	RTC_SECOND,
	RTC_SUBSEC,

	//FC state (others) //TODO ADD THESE TO EXTERNAL FLASH and GETINFO PROCEDURE (Or nah?)
//	FC_STATE_VR_POWER,
//	FC_STATE_VR_RECORDING,
	FC_STATE_ALT_PAD,			// floats will be rounded to int
	FC_PAD_TIME,
	FC_STATE_TRUE_APOGEE,
	FC_TRUE_APOGEE_TIME,
	FC_STATE_ALT_APOGEE,
	FC_APOGEE_TIME,
	FC_STATE_ALT_MAIN,
	FC_MAIN_TIME,
	FC_STATE_ALT_LANDED,
	FC_LANDED_TIME
} rtc_backup_reg;

//Flags
extern uint32_t rtc_bckp_reg_reset; //In external flash memory
extern uint32_t rtc_bckp_reg_wu; //TODO used to be defined in MRT_RTOS.c
extern uint32_t rtc_bckp_reg_iwdg; //In external flash memory
extern uint32_t rtc_bckp_reg_apogee; //In external flash memory
extern uint32_t rtc_bckp_reg_ejection_stage; //In external flash memory

//Time variables
extern uint32_t rtc_bckp_reg_hour; //Last recorded hours
extern uint32_t rtc_bckp_reg_min; //Last recorded minutes
extern uint32_t rtc_bckp_reg_sec; //Last recorded seconds
extern uint32_t rtc_bckp_reg_subsec; //Last recorded subseconds

//FC states
extern uint32_t rtc_bckp_reg_vr_power;
extern uint32_t rtc_bckp_reg_vr_rec;

extern uint32_t rtc_bckp_reg_alt_pad; // floats will be rounded to int
extern uint32_t rtc_bckp_reg_pad_time;
extern uint32_t rtc_bckp_reg_alt_true_apogee;
extern uint32_t rtc_bckp_reg_true_apogee_time;
extern uint32_t rtc_bckp_reg_alt_apogee;
extern uint32_t rtc_bckp_reg_apogee_time;
extern uint32_t rtc_bckp_reg_alt_main;
extern uint32_t rtc_bckp_reg_main_time;
extern uint32_t rtc_bckp_reg_alt_landed;
extern uint32_t rtc_bckp_reg_landed_time;


//Reference list to each time component (in the same order than typedef enum rtc_backup_reg
extern uint32_t* rtc_bckp_regs[NB_RTC_BCKP_REGS];

void MRT_RTC_backup_regs_Init(void);
void MRT_RTC_clearBackupRegs(void);
uint32_t MRT_RTC_getBackupReg(rtc_backup_reg state);
void MRT_RTC_setBackupReg(rtc_backup_reg state, uint32_t value);

void restore_fc_states(void);





/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
