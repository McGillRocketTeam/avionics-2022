/*
 * radio_commands.c
 *
 *  Created on: Mar 1, 2022
 *      Author: jasper
 */


#include "radio_commands.h"
#include "video_recorder.h"
#include "main.h" // for pins
#include "string.h" // strcmp

extern UART_HandleTypeDef huart8;

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
	else if (strcmp(xtend_rx_buf, "dapr") == 0) { // disarm propulsion
		return DISARM_PROP;
	}
	else if (strcmp(xtend_rx_buf, "darc") == 0) {
		return DISARM_RCOV;
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

void execute_parsed_command(radio_command cmd) {
	// TODO: decide whether we want to send an ack back to ground station, maybe as special event message
	switch (cmd) {
	case LAUNCH:
		rocket_launch();
		HAL_UART_Transmit(&huart8, "launch\r\n", 8, HAL_MAX_DELAY);
		break;

	case ARM_PROP:
		arming_propulsion();
		HAL_UART_Transmit(&huart8, "arm pr\r\n", 8, HAL_MAX_DELAY);
		break;

	case ARM_RCOV:
		arming_recovery();
		HAL_UART_Transmit(&huart8, "arm rc\r\n", 8, HAL_MAX_DELAY);
		break;

	case DISARM_PROP:
		disarm_propulsion();
		break;

	case DISARM_RCOV:
		disarm_recovery();
		break;

	case VR_POWER_ON:	// TODO: figure out how to make non-blocking
		VR_Power_On();
		break;

	case VR_REC_START:	// TODO: figure out how to make non-blocking
		VR_Start_Rec();
		break;

	case VR_REC_STOP:	// TODO: figure out how to make non-blocking
		VR_Stop_Rec();
		break;

	case VR_POWER_OFF:
		VR_Power_Off();
		break;

	default:
		break;
	}
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

void disarm_propulsion(void) {
	// disarm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, RESET);

	// also reset the gates in case they were high
	HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
	HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);
}

void disarm_recovery(void) {
	// disarm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, RESET);

	// also reset the gates in case they were high
	HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
	HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);
}
