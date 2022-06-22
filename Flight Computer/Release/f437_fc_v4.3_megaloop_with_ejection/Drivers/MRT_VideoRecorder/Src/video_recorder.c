/*
 * video_recorder.c
 *
 *  Created on: Feb 11, 2022
 *      Author: jasper
 */


#include "main.h"
#include "state_restoration.h"
#include "video_recorder.h"

volatile uint8_t vr_is_stop = 0;

void VR_Power_On(void) {
	HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, SET);
	set_backup_state(FC_STATE_VR_POWER, 1);

	// note that runcam needs around 3-5 seconds to fully power on!
//	HAL_Delay(5000);
}

void VR_Power_Off(void) {
	HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, RESET);
	set_backup_state(FC_STATE_VR_POWER, 0);
}

void VR_Start_Rec(void) {
//	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET); // HIGH = start record
	set_backup_state(FC_STATE_VR_RECORDING, 1);
}

void VR_Stop_Rec(void) {
//	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET); // LOW = stop recording
	vr_is_stop = 1;
	set_backup_state(FC_STATE_VR_RECORDING, 0);
}
