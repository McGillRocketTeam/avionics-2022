/*
 * radio_commands.c
 *
 *  Created on: Mar 1, 2022
 *      Author: jasper
 */

#include <MRT_setup.h>
#include "MRT_helpers.h"
#include "radio_commands.h"
#include "video_recorder.h"
#include "main.h" // for pins
#include "string.h" // strcmp

//#include "state_restoration.h" TODO NOT USED YET

//extern volatile uint8_t state_arm_rcov; TODO NOT USED YET
//extern volatile uint8_t state_arm_prop; TODO NOT USED YET

//extern volatile char rx_buf[10]; // dma buffer NOT IN USE YET

radio_command radio_parse_command(char* rx_buf) {

	#if DEBUG
	char radio_buffer[10];
	for (int i=0; i<strlen(rx_buf);i++){
		memset(radio_buffer,0,10);
		sprintf(radio_buffer,"%i",rx_buf[i]);
		println(radio_buffer);
	}
	#endif

	if (strcmp(rx_buf, "lr") == 0) { // launch command
		return LAUNCH;
	}
	else if (strcmp(rx_buf, "ap") == 0) { // arm propulsion
		return ARM_PROP;
	}
	else if (strcmp(rx_buf, "ar") == 0) { // arm recovery
		return ARM_RCOV;
	}
	else if (strcmp(rx_buf, "dp") == 0) { // disarm propulsion
		return DISARM_PROP;
	}
	else if (strcmp(rx_buf, "dr") == 0) { // disarm recovery
		return DISARM_RCOV;
	}
	else if (strcmp(rx_buf, "v1") == 0) { // vr power on
		return VR_POWER_ON;
	}
	else if (strcmp(rx_buf, "v2") == 0) { // vr start
		return VR_REC_START;
	}
	else if (strcmp(rx_buf, "v3") == 0) { // vr stop
		return VR_REC_STOP;
	}
	else if (strcmp(rx_buf, "v4") == 0) { // vr power off
		return VR_POWER_OFF;
	}

	// all other commands are invalid, ignore.
	else{
		return -1;
	}
}

void execute_parsed_command(radio_command cmd) {
	// TODO: decide whether we want to send an ack back to ground station, maybe as special event message
	switch (cmd) {
	case LAUNCH:
		rocket_launch();
		println((char*) "launch");
		break;

	case ARM_PROP:
		arming_propulsion();
		println((char*) "arm pr");
		break;

	case ARM_RCOV:
		arming_recovery();
		println((char*) "arm rc");
		break;

	case DISARM_PROP:
		disarm_propulsion();
		println((char*) "disarm pr");
		break;

	case DISARM_RCOV:
		disarm_recovery();
		println((char*) "disarm rc");
		break;

	case VR_POWER_ON:
		VR_Power_On();
		println((char*) "vr on");
		break;

	case VR_REC_START:
		VR_Start_Rec();
		println((char*) "vr start");
		break;

	case VR_REC_STOP:
		VR_Stop_Rec();
		println((char*) "vr stop");
		break;

	case VR_POWER_OFF:
		VR_Power_Off();
		println((char*) "vr off");
		break;

	default:
		break;
	}
}

void rocket_launch(void) {
	// just to be safe, set arming pin high to ensure pyro channels are armed
	HAL_GPIO_WritePin(OUT_PyroValve_Arming_GPIO_Port, OUT_PyroValve_Arming_Pin, SET);

	// open valve by firing the prop pyro ejection channels
	HAL_GPIO_WritePin(OUT_PyroValve_Gate_1_GPIO_Port, OUT_PyroValve_Gate_1_Pin, SET);
	HAL_GPIO_WritePin(OUT_PyroValve_Gate_2_GPIO_Port, OUT_PyroValve_Gate_2_Pin, SET);
}

void arming_propulsion(void) {
	// arm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(OUT_PyroValve_Arming_GPIO_Port, OUT_PyroValve_Arming_Pin, SET);
	//state_arm_prop = 1;
	//set_backup_state(FC_STATE_ARM_PROP, //state_arm_prop);
}

void arming_recovery(void) {
	// arm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, SET);
	//state_arm_rcov = 1;
	//set_backup_state(FC_STATE_ARM_RCOV, //state_arm_rcov);
}

void disarm_propulsion(void) {
	// disarm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(OUT_PyroValve_Arming_GPIO_Port, OUT_PyroValve_Arming_Pin, RESET);

	// also reset the gates in case they were high
	HAL_GPIO_WritePin(OUT_PyroValve_Gate_1_GPIO_Port, OUT_PyroValve_Gate_1_Pin, RESET);
	HAL_GPIO_WritePin(OUT_PyroValve_Gate_2_GPIO_Port, OUT_PyroValve_Gate_2_Pin, RESET);

	//state_arm_prop = 0;
	//set_backup_state(FC_STATE_ARM_PROP, //state_arm_prop);
}

void disarm_recovery(void) {
	// disarm, TODO: decide whether to add feedback/check on arming status
	HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, RESET);

	// also reset the gates in case they were high
	HAL_GPIO_WritePin(OUT_EJ_Drogue_Gate_GPIO_Port, OUT_EJ_Drogue_Gate_Pin, RESET);
	HAL_GPIO_WritePin(OUT_EJ_Main_Gate_GPIO_Port, OUT_EJ_Main_Gate_Pin, RESET);

	//state_arm_rcov = 0;
	//set_backup_state(FC_STATE_ARM_RCOV, //state_arm_rcov);
}
