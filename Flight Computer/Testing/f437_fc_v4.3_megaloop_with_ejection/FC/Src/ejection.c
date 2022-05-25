/*
 * ejection.c
 *
 *  Created on: Mar 18, 2021
 *      Author: a cat, modified by jasper
 */

#include "ejection.h"
#include "sd_card.h"			// saving messages on state change
#include "state_restoration.h"
#include "video_recorder.h"
#include "gpio.h"

#include "radio_commands.h" 	// for testing e-match firing

#define DEBUG_MODE

uint32_t prevTick = 0;
float T;						// sampling period, time of each loop
extern float SmoothData;
extern float alt_ground;
extern float alt_filtered;
float LPF_Beta = 0.8;
uint8_t currElem = 0;

// Public variables
float alt_previous[NUM_MEAS_REG];
float time_previous[NUM_MEAS_REG];
float timalt_previous[NUM_MEAS_REG];
float timsqr_previous[NUM_MEAS_REG];

// TODO: put in struct?
volatile ejection_state state_flight;
volatile uint8_t state_arm_rcov;
volatile uint8_t state_arm_prop;

// tracking altitude for state of flight changing
float alt_ground;
float alt_current;
float alt_prev;
float alt_apogee;

uint8_t num_descending_samples = 0;

void init_ejection_states(void) {
	state_arm_rcov = 0;
	state_arm_prop = 0;
	state_flight = EJ_STATE_PAD;

	alt_ground = 0;
	alt_current = 0;
	alt_prev = 0;
	alt_apogee = 0;
}

// Private functions
void storeAltitude(float new_altitude, float cTime);

// Public function implementation
float runAltitudeMeasurements(uint32_t currTick, uint16_t currAlt){
  T = (float)(currTick - prevTick);

  prevTick = currTick;
  float alt_meas = currAlt - alt_ground; // Measures AGL altitude in feet
//  alt_filtered = filterAltitude(alt_meas);
  float alt_filtered = alt_meas; // disable filtering for now
  storeAltitude(alt_filtered, currTick);
  return alt_filtered;
}

// -- Private function implementation --

// Low-pass filter - rocket at high speeds pressure fluctuates and affects altitude reading,
// usually at a high frequency, so low pass filter filters those high frequency changes out
// and keeps just the overall, low frequency changes (caused by altitude change)
float filterAltitude(float altitude) {

	// found from some janky website
	SmoothData -= LPF_Beta * (SmoothData - altitude);
	return SmoothData;

	// TODO: implement the low-pass filter properly
	// https://drive.google.com/drive/folders/0Bzst7Dr29_BkZjcwM2U1OFVzOW8?resourcekey=0-1zHXZZZvJblOq9rc6TyEeg
//	float wc = 2*3.14*500; // 500 Hz cutoff
}

void storeAltitude(float new_altitude, float cTime) {

	alt_previous[currElem] = new_altitude;
	time_previous[currElem] = cTime;
	timalt_previous[currElem] = cTime * new_altitude;
	timsqr_previous[currElem] = cTime * cTime;

	// circular buffer
	if (currElem == (NUM_MEAS_REG - 1)) {
		currElem = 0;
	}
	else {
		currElem += 1;
	}
}

float LSLinRegression() {
	float xsum = 0, ysum = 0, xy = 0, xsqr = 0;

	for (uint8_t i = 0; i < NUM_MEAS_REG; i++) {
		xsum += time_previous[i];
	    ysum += alt_previous[i];
	    xy += timalt_previous[i];
	    xsqr += timsqr_previous[i];
	}

	return (float)(NUM_MEAS_REG*xy - (xsum*ysum))/(NUM_MEAS_REG*xsqr - (xsum*xsum));
}

uint8_t get_continuity(void) {
	// read pins
	GPIO_PinState drogue = HAL_GPIO_ReadPin(Rcov_Cont_Drogue_GPIO_Port, Rcov_Cont_Drogue_Pin);
	GPIO_PinState main = HAL_GPIO_ReadPin(Rcov_Cont_Main_GPIO_Port, Rcov_Cont_Main_Pin);
	GPIO_PinState prop_1 = HAL_GPIO_ReadPin(Prop_Cont_1_GPIO_Port, Prop_Cont_1_Pin);
	GPIO_PinState prop_2 = HAL_GPIO_ReadPin(Prop_Cont_2_GPIO_Port, Prop_Cont_2_Pin);

	uint8_t continuity = (drogue) + (main * 2) + (prop_1 * 4) + (prop_2 * 8);
	return continuity;
}

// logic to change states of flight
void check_ejection_state(void) {
	switch (state_flight) {
	case EJ_STATE_PAD: // launch pad, waiting. prioritize prop data

		// check current state
		if (alt_current - alt_ground > LAUNCH_ALT_CHANGE_THRESHOLD) { // launched
			num_descending_samples += 1;

			if (num_descending_samples > LAUNCH_NUM_DESCENDING_SAMPLES) {
				state_flight = EJ_STATE_BOOST_COAST;

				#ifdef DEBUG_MODE
					HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, SET);
				#endif

//				sd_write(&fil, (uint8_t *)"launched\r\n");

				// generate software interrupt to change TIM3 update rate
				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}

		}
		else {
			num_descending_samples = 0;
		}

		break;

	case EJ_STATE_BOOST_COAST: // pre-apogee, waiting for ejection and drogue deployment

		if (LSLinRegression() < 0) {
			num_descending_samples += 1;

			if (num_descending_samples > APOGEE_NUM_DESCENDING_SAMPLES) {
				// *** EJECTION AND DROGUE DEPLOYMENT *** //
//				HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET); // can't hurt right? in case arming failed on the pad
//				HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
//				HAL_Delay(DROGUE_DELAY);
//				HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
				// *** ------------------------------ *** //

				state_flight = EJ_STATE_DROGUE_DESCENT; // passed apogee
				num_descending_samples = 0;

//				sd_write(&fil, (uint8_t *)"apogee\r\n");

				#ifdef DEBUG_MODE
					HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, SET);
				#endif

				// generate software interrupt to change TIM3 update rate
				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		}
		else {
			num_descending_samples = 0;
		}

		break;

	case EJ_STATE_DROGUE_DESCENT: // post-apogee, waiting for main parachute deployment

		// check current state
		if (alt_current - alt_ground < MAIN_DEPLOY_ALTITUDE) {
			num_descending_samples++;

			if (num_descending_samples > MAIN_NUM_DESCENDING_SAMPLES) {
				// *** DEPLOYING MAIN PARACHUTE *** //
//				HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET); // can't hurt right? in case arming failed on the pad
//				HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
//				HAL_Delay(MAIN_DELAY);
//				HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);
				// *** ------------------------ *** //

				state_flight = EJ_STATE_MAIN_DESCENT;
				alt_prev = alt_current; // in next stage we need to know the previous altitude
				num_descending_samples = 0;

//				sd_write(&fil, (uint8_t *)"main deployed\r\n");

				#ifdef DEBUG_MODE
					HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
				#endif

				// generate software interrupt to change TIM3 update rate
				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		} else {
			num_descending_samples = 0;
		}

		break;

	case EJ_STATE_MAIN_DESCENT:
		// post main deploy, want to transmit data fast to maximize possibility of getting good GPS coordinates
		; // empty statement to avoid 'a label can only be part of a statement and a declaration is not a statement' compiler error

		// check current state
		float alt_diff = alt_current - alt_prev;
		if (alt_diff < 0) {
			alt_diff *= -1; // absolute value
		}

		if (alt_diff < LANDING_ALT_CHANGE_THRESHOLD) {
			num_descending_samples++;

			if (num_descending_samples > LANDING_NUM_DESCENDING_SAMPLES) {
				state_flight = EJ_STATE_LANDED;
				num_descending_samples = 0;

//				sd_write(&fil, (uint8_t *)"landed\r\n");

				#ifdef DEBUG_MODE
					HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
				#endif

				// generate software interrupt to change TIM3 update rate
				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		} else {
			num_descending_samples = 0;
		}

		alt_prev = alt_current;
		break;

	case EJ_STATE_LANDED: // landed

		// stop video recorder
		if (get_backup_state(FC_STATE_VR_RECORDING) || get_backup_state(FC_STATE_VR_POWER)) {
			VR_Stop_Rec();
			set_backup_state(FC_STATE_VR_RECORDING, 0);
			HAL_Delay(1000);
			VR_Power_Off();
			set_backup_state(FC_STATE_VR_POWER, 0);
		}

		#ifdef DEBUG_MODE
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			HAL_GPIO_TogglePin(LEDF_GPIO_Port, LEDF_Pin);
		#endif

		break;

	default:

		break;
	}
}

#ifdef DEBUG_MODE
extern volatile uint8_t button_pressed;

void test_ematch_firing(void) {
	button_pressed = 0;
	while (1) {

		while (!button_pressed);

		HAL_Delay(100);
		arming_propulsion();
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		arming_recovery();
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		disarm_propulsion();
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		disarm_recovery();
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		arming_propulsion();
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		arming_recovery();
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
		HAL_Delay(100);
		button_pressed = 0;

		while (!button_pressed);

		HAL_Delay(100);
		HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);
		HAL_Delay(100);
		button_pressed = 0;

		disarm_propulsion();
		disarm_recovery();

	}
}
#endif
