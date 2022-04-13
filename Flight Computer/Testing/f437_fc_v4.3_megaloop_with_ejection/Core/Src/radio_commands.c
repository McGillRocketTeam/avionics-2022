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
#include "state_restoration.h"

extern UART_HandleTypeDef huart8;
extern volatile uint8_t state_arm_rcov;
extern volatile uint8_t state_arm_prop;

extern volatile char xtend_rx_buf[10]; // dma buffer

radio_command xtend_parse_dma_command(void) {

	if (strcmp(xtend_rx_buf, "lr") == 0) { // launch command
		return LAUNCH;
	}
	else if (strcmp(xtend_rx_buf, "ap") == 0) { // arm propulsion
		return ARM_PROP;
	}
	else if (strcmp(xtend_rx_buf, "ar") == 0) { // arm recovery
		return ARM_RCOV;
	}
	else if (strcmp(xtend_rx_buf, "dp") == 0) { // disarm propulsion
		return DISARM_PROP;
	}
	else if (strcmp(xtend_rx_buf, "dr") == 0) { // disarm recovery
		return DISARM_RCOV;
	}
	else if (strcmp(xtend_rx_buf, "v1") == 0) { // vr power on
		return VR_POWER_ON;
	}
	else if (strcmp(xtend_rx_buf, "v2") == 0) { // vr start
		return VR_REC_START;
	}
	else if (strcmp(xtend_rx_buf, "v3") == 0) { // vr stop
		return VR_REC_STOP;
	}
	else if (strcmp(xtend_rx_buf, "v4") == 0) { // vr power off
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
		HAL_UART_Transmit(&huart8, "disarm pr\r\n", 11, HAL_MAX_DELAY);
		break;

	case DISARM_RCOV:
		disarm_recovery();
		HAL_UART_Transmit(&huart8, "disarm rc\r\n", 11, HAL_MAX_DELAY);
		break;

	case VR_POWER_ON:
		VR_Power_On();
		HAL_UART_Transmit(&huart8, "vr on\r\n", 7, HAL_MAX_DELAY);
		break;

	case VR_REC_START:
		VR_Start_Rec();
		HAL_UART_Transmit(&huart8, "vr start\r\n", 10, HAL_MAX_DELAY);
		break;

	case VR_REC_STOP:
		VR_Stop_Rec();
		HAL_UART_Transmit(&huart8, "vr stop\r\n", 9, HAL_MAX_DELAY);
		break;

	case VR_POWER_OFF:
		VR_Power_Off();
		HAL_UART_Transmit(&huart8, "vr off\r\n", 8, HAL_MAX_DELAY);
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
	state_arm_prop = 1;
	set_backup_state(FC_STATE_ARM_PROP, state_arm_prop);
}

void arming_recovery(void) {
	// arm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET);
	state_arm_rcov = 1;
	set_backup_state(FC_STATE_ARM_RCOV, state_arm_rcov);
}

void disarm_propulsion(void) {
	// disarm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, RESET);

	// also reset the gates in case they were high
	HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
	HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);

	state_arm_prop = 0;
	set_backup_state(FC_STATE_ARM_PROP, state_arm_prop);
}

void disarm_recovery(void) {
	// disarm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, RESET);

	// also reset the gates in case they were high
	HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
	HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);

	state_arm_rcov = 0;
	set_backup_state(FC_STATE_ARM_RCOV, state_arm_rcov);
}
