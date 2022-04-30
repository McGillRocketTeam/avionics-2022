/*
 * MRT_external_flash.h
 *
 *  Created on: Apr 12, 2022
 *      Author: Jacoby
 */

#ifndef MRT_MEMORY_INC_MRT_EXTERNAL_FLASH_H_
#define MRT_MEMORY_INC_MRT_EXTERNAL_FLASH_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "w25qxx.h"


//**************************************************//
//FLAGS

//Flags Defines
#define NB_OF_FLAGS 5 //Number of flags used
#define FLAGS_OFFSET 0 //Flags offset in sector 1
#define FLAGS_SECTOR 1 //Flags sector on exernal flash

//Map every variable to their offset in sector 1
#define RESET_FLAG_OFFSET 0
#define WU_FLAG_OFFSET 1
#define IWDG_FLAG_OFFSET 2
#define APOGEE_FLAG_OFFSET 3
#define EJECTION_STAGE_FLAG_OFFSET 4


//Flags Variables
extern uint8_t ext_flash_reset; //if 0 -> start from beginning, if 1 -> random watchdog reset (if 2-> reset after wakeup??)
extern uint8_t ext_flash_wu; //TODO used to be defined in MRT_RTOS.c
extern uint8_t ext_flash_iwdg; //if 0 -> normal, if 1 -> try to go into standbymode
extern uint8_t ext_flash_apogee; //if 0 -> pre-apogee, if 1 -> post-apogee
extern uint8_t ext_flash_ejection_stage; //if 0-> pad, if 1->boost, if 2->drogue descent, if 3->main descent, if 4-> landed

//Flags read/write buffer
extern uint8_t ext_flash_flags_buffer[NB_OF_FLAGS];

//Reference list to each flag
extern uint8_t* ext_flash_flags[NB_OF_FLAGS];



//**************************************************//
//RTC TIME FLAGS

//RTC Time Defines
#define RTC_NB_OF_VAR 4 //Number of RTC variables
#define RTC_TIME_OFFSET 0 //RTC time offset in sector 2
#define RTC_SECTOR 2 //Flags sector on exernal flash

//Map hours, minutes and seconds to their offsets
#define RTC_HOURS_OFFSET 0
#define RTC_MIN_OFFSET 1
#define RTC_SEC_OFFSET 2
#define RTC_SUBSEC_OFFSET 3

//RTC Time variables
extern uint8_t ext_flash_hour; //Last recorded hours
extern uint8_t ext_flash_min; //Last recorded minutes
extern uint8_t ext_flash_sec; //Last recorded seconds
extern uint32_t ext_flash_subsec; //Last recorded subseconds

//Time read/write buffer
extern uint8_t ext_flash_time_buffer[RTC_NB_OF_VAR];

//Reference list to each flag
extern uint8_t* ext_flash_time[RTC_NB_OF_VAR];

//Null buffer values for when clearing time
extern uint8_t RTC_TIME_NULL_BUFFER[RTC_NB_OF_VAR];


//**************************************************//
//FUNCTION PROTOTYPES
void MRT_external_flash_Init(void);
void MRT_updateExternalFlashBuffers(void);
void MRT_updateExternalFlashValues(void);
void MRT_clearExternalFlash(void);


#ifdef __cplusplus
}
#endif

#endif /* MRT_MEMORY_INC_MRT_EXTERNAL_FLASH_H_ */
