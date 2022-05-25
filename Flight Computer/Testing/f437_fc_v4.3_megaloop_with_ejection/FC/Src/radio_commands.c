/*
 * radio_commands.c
 *
 *  Created on: Mar 1, 2022
 *      Author: jasper
 */


#include "radio_commands.h"
#include "main.h" // for pins
#include "telemetry.h"
#include "xtend.h"
#include "video_recorder.h"
#include "state_restoration.h"
#include "ejection.h"
#include "helpers.h"

#include <string.h> // strcmp
#include <stdio.h>  // sprintf

extern volatile uint8_t xtend_is_transmitting;

const uint32_t ARR_VALS_ARR[] = {
		ARR_PAD,
		ARR_BOOST_COAST,
		ARR_DROGUE_DESCENT,
		ARR_MAIN_DESCENT,
		ARR_LANDED
};

radio_command parse_radio_command(volatile char *buf) {

	if (strcmp(buf, "lr") == 0) { // launch command
		rocket_launch();
		debug_tx_uart((uint8_t *)"launch\r\n");
		return LAUNCH;
	}
	else if (strcmp(buf, "ap") == 0) { // arm propulsion
		arming_propulsion();
		debug_tx_uart((uint8_t *)"arm pr\r\n");
		return ARM_PROP;
	}
	else if (strcmp(buf, "ar") == 0) { // arm recovery
		arming_recovery();
		debug_tx_uart((uint8_t *)"arm rc\r\n");
		return ARM_RCOV;
	}
	else if (strcmp(buf, "dp") == 0) { // disarm propulsion
		disarm_propulsion();
		debug_tx_uart((uint8_t *)"disarm pr\r\n");
		return DISARM_PROP;
	}
	else if (strcmp(buf, "dr") == 0) { // disarm recovery
		disarm_recovery();
		debug_tx_uart((uint8_t *)"disarm rc\r\n");
		return DISARM_RCOV;
	}
	else if (strcmp(buf, "d1") == 0) { // dump valve power on
		dump_power_on();
		debug_tx_uart((uint8_t *)"dump pwr on\r\n");
		return DUMP_POWER_ON;
	}
	else if (strcmp(buf, "d2") == 0) { // dump valve power off
		dump_power_off();
		debug_tx_uart((uint8_t *)"dump pwr off\r\n");
		return DUMP_POWER_OFF;
	}
	else if (strcmp(buf, "v1") == 0) { // vr power on
		VR_Power_On();
		debug_tx_uart((uint8_t *)"vr on\r\n");
		return VR_POWER_ON;
	}
	else if (strcmp(buf, "v2") == 0) { // vr start
		// ASSUMING VR IS CONTROLLED BY ATTINY AND VR COMMANDS ARE NON BLOCKING
		VR_Start_Rec();
		debug_tx_uart((uint8_t *)"vr off\r\n");
		return VR_REC_START;
	}
	else if (strcmp(buf, "v3") == 0) { // vr stop
		VR_Stop_Rec();
		debug_tx_uart((uint8_t *)"vr stop\r\n");
		return VR_REC_STOP;
	}
	else if (strcmp(buf, "v4") == 0) { // vr power off
		VR_Power_Off();
		debug_tx_uart((uint8_t *)"vr off\r\n");
		return VR_POWER_OFF;
	}

#ifdef TEST_EJECTION_FIRING
	else if (strcmp(buf, "f1") == 0) { // fire drogue
		HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
		return FIRE_RCOV_DROGUE;
	}
	else if (strcmp(buf, "f2") == 0) { // fire main
		HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
		return FIRE_RCOV_MAIN;
	}
	else if (strcmp(buf, "f3") == 0) { // fire prop 1
		HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, SET);
		return FIRE_PROP_1;
	}
	else if (strcmp(buf, "f4") == 0) { // fire prop 2
		HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, SET);
		return FIRE_PROP_2;
	}
#endif

	else {
		return COMMAND_INVALID;
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
	HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, SET); // close dump valve
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
	HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET); // open dump valve

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

void dump_power_on(void) {
	HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, SET);
}

void dump_power_off(void) {
	HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET);
}
// sends an avionics or propulsion string depending on the situation
void xtend_transmit_telemetry(volatile uint8_t state) {

	switch (state) {
	case EJ_STATE_PAD:

		// send av
		if (num_radio_transmissions % 2 == 0) {
			xtend_tx(msg_buffer_av, strlen((const char *)msg_buffer_av));
		}

		// send prop
		else {
			xtend_tx(msg_buffer_pr, strlen((const char *)msg_buffer_pr));
		}
		break;

	case EJ_STATE_BOOST_COAST:
		// transmit avionics and prop at equal priority
		if (num_radio_transmissions % 2 == 0) {
			xtend_tx(msg_buffer_av, strlen((const char *)msg_buffer_av));
		}
		else {
			xtend_tx(msg_buffer_pr, strlen((const char *)msg_buffer_pr));
		}
		break;

	default:
		xtend_tx(msg_buffer_av, strlen((const char *)msg_buffer_av));
		break;
	}
}

// updates settings for XTEND_TIM depending on the state of the flight.
// XTEND_TIM controls the rate of XTend radio transmission
void update_radio_timer_params(volatile uint8_t state) {

	uint32_t arr_val = ARR_VALS_ARR[state];
	uint8_t rate = 10000/arr_val;

	XTEND_TIM->ARR = arr_val;
	XTEND_TIM->EGR |= TIM_EGR_UG;

#ifdef DEBUG
	char buf[20];
	sprintf(buf, "radio rate = %d Hz\r\n", rate);
	debug_tx_uart((uint8_t *)buf);
#endif

}

