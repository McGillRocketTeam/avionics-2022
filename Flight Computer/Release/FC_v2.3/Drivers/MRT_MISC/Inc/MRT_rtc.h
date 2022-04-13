/*
 * MRT_rtc.h
 *
 *  Created on: Apr 12, 2022
 *      Author: Jacoby
 */

#ifndef MRT_MISC_INC_MRT_RTC_H_
#define MRT_MISC_INC_MRT_RTC_H_



#endif /* MRT_MISC_INC_MRT_RTC_H_ */


extern uint8_t flagA; //Dynamic
extern uint8_t flagB; //Dynamic


//**************************************************//
//FUNCTIONS PROTOTYPES
void MRT_rtc_Init(void);

void MRT_check_for_wake_up(void);
void MRT_clear_alarms_flags(void);
void MRT_StandByMode(uint32_t seconds);

void MRT_set_rtc(uint8_t h, uint8_t m, uint8_t s);
void MRT_set_alarmA(uint8_t h, uint8_t m, uint8_t s);
