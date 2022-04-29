/*
 * MRT_memory.h
 *
 *  Created on: Apr 29, 2022
 *      Author: Jacoby
 */

#ifndef MRT_MEMORY_INC_MRT_MEMORY_H_
#define MRT_MEMORY_INC_MRT_MEMORY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <rtc.h>
#include <MRT_external_flash.h>
#include <sd_card.h>

//Flags Variables
extern uint8_t reset_flag; //if 0 -> start from beginning, if 1 -> random watchdog reset (if 2-> reset after wakeup??)
extern uint8_t wu_flag; //TODO used to be defined in MRT_RTOS.c
extern uint8_t iwdg_flag; //if 0 -> normal, if 1 -> try to go into standbymode
extern uint8_t apogee_flag; //if 0 -> pre-apogee, if 1 -> post-apogee
extern uint8_t ejection_stage_flag; //if 0-> pad, if 1->boost, if 2->drogue descent, if 3->main descent, if 4-> landed


//RTC Time variables
extern uint8_t prev_hour; //Last recorded hours
extern uint8_t prev_min; //Last recorded minutes
extern uint8_t prev_sec; //Last recorded seconds
extern uint32_t prev_subsec; //Last recorded subseconds


//Public functions
void MRT_MEMORY_Init(void);
void MRT_saveFlagValue(rtc_backup_reg state);
void MRT_resetFlags(void);
void MRT_saveTotalTime(void);
void MRT_resetTotalTime(void);




#ifdef __cplusplus
}
#endif

#endif /* MRT_MEMORY_INC_MRT_MEMORY_H_ */
