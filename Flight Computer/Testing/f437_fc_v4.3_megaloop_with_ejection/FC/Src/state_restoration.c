/*
 * state_restoration.c
 *
 *  Created on: Apr 5, 2022
 *      Author: jasper
 */


#include "main.h"
#include "rtc.h"
#include "state_restoration.h"
#include "radio_commands.h"
#include "video_recorder.h"
#include "ejection.h"

extern RTC_HandleTypeDef hrtc;

// public variables in main to be reset
extern volatile uint32_t flash_write_address;

// initializes backup register values to zero
void init_backup_regs(void) {
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

	for (uint8_t i = 0; i < 20; i++) {
		HAL_RTCEx_BKUPWrite(&hrtc, i, 0);	// set all backup register values to zero
	}

	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
}

// gets backup register values and performs the appropriate
// restoration action (e.g. arming, altitudes, etc.)
void restore_fc_states(void) {
	// flight state
	state_flight = (uint8_t) (get_backup_state(FC_STATE_FLIGHT));
	if (state_flight > EJ_STATE_BOOST_COAST) {
		// disable 12 V regulator (which is active low)
		HAL_GPIO_WritePin(PM_12V_EN_GPIO_Port, PM_12V_EN_Pin, SET);

		// no power to dump valve
		HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET);
	}

	// arming
	if (get_backup_state(FC_STATE_ARM_PROP)) {
		arming_propulsion(); // sets state_arm_prop = 1;
	} else {
		disarm_propulsion();
	}

	if (get_backup_state(FC_STATE_ARM_RCOV)) {
		arming_recovery();
	} else {
		disarm_recovery();
	}

	// video recorder
	if (get_backup_state(FC_STATE_VR_POWER)) {
		VR_Power_On();
	} else {
		VR_Power_Off();
	}

	if (get_backup_state(FC_STATE_VR_RECORDING)) {
		VR_Start_Rec();
	} else {
		VR_Stop_Rec();
	}

	// altitudes
	alt_ground = (float) (get_backup_state(FC_STATE_ALT_GROUND));
	alt_apogee = (float) (get_backup_state(FC_STATE_ALT_APOGEE));
	alt_prev = (float) (get_backup_state(FC_STATE_ALT_PREV));

	// flash
	flash_write_address = get_backup_state(FC_STATE_FLASH_WRITE_ADDRESS);

}

// gets the backup register value for the specified state
uint32_t get_backup_state(fc_state_backup state) {
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	uint32_t val = HAL_RTCEx_BKUPRead(&hrtc, (uint32_t) state);
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	return val;
}

// sets the backup register value for the specified state
void set_backup_state(fc_state_backup state, uint32_t value) {
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	HAL_RTCEx_BKUPWrite(&hrtc, (uint32_t) state, value);
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
}
