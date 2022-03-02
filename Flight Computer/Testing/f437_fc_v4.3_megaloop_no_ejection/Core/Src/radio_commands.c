/*
 * radio_commands.c
 *
 *  Created on: Mar 1, 2022
 *      Author: jasper
 */


#include "radio_commands.h"
#include "main.h" // for pins

extern volatile char xtend_rx_buf[10]; // dma buffer

radio_command xtend_parse_dma_command(void) {

	if (strcmp(xtend_rx_buf, "lnch") == 0) { // launch command
		return LAUNCH;
	}
	else if (strcmp(xtend_rx_buf, "arpr") == 0) { // arm propulsion
		return ARM_PROP;
	}
	else if (strcmp(xtend_rx_buf, "arrc") == 0) { // arm recovery
		return ARM_RCOV;
	}
	else if (strcmp(xtend_rx_buf, "vron") == 0) { // vr power on
		return VR_POWER_ON;
	}
	else if (strcmp(xtend_rx_buf, "vrs1") == 0) { // s1 = start
		return VR_REC_START;
	}
	else if (strcmp(xtend_rx_buf, "vrs2") == 0) { // s2 = stop
		return VR_REC_STOP;
	}
	else if (strcmp(xtend_rx_buf, "vrof") == 0) { // vr power off
		return VR_POWER_OFF;
	}

	// all other commands are invalid, ignore.
}

void rocket_launch(void) {
	// just to be safe, set arming pin high to ensure pyro channels are armed
	HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, SET);

	// open valve by firing the prop pyro ejection channels
	HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, SET);
	HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, SET);
}

void arming_propulsion(void) {
	// arm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, SET);
}

void arming_recovery(void) {
	// arm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET);
}
