/*
 * video_recorder.c
 *
 *  Created on: Feb 11, 2022
 *      Author: jasper
 */


#include "main.h"


void VR_Power_On(void) {
	HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, SET);
	// note that runcam needs around 3-5 seconds to fully power on
	HAL_Delay(5000);
}

void VR_Power_Off(void) {
	HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, RESET);
}

void VR_Start_Rec(void) {

	// specific sequence of SET/RESET to start recording
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);

}

void VR_Stop_Rec(void) {

	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);
}
