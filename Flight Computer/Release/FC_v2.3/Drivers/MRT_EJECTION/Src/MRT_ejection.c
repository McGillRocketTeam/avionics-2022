/*
 * MRT_ejection.c
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#include <MRT_setup.h>
#include <MRT_ejection.h>
#include <MRT_i2c_sensors.h>
#include <MRT_memory.h>
#include <stdio.h> //sprintf
#include <string.h> //memset
#include <math.h>
#include <gpio.h>


//Global variables
uint8_t gates_continuity = 0;
char msg_buffer_av[200];


/*
 * Gets the altitude using temperature, pressure and sea-level pressure
 *https://www.mide.com/air-pressure-at-altitude-calculator
 */
float MRT_getAltitude(float pressure){
	return BASE_HEIGHT+(SEA_LEVEL_TEMPERATURE/-0.0065)*(pow(pressure/SEA_LEVEL_PRESSURE,0.190263236)-1); //(-R*-0.0065/(go*M)) = 0.190263236
}



/*
 * Checks the continuity of the gates
 *
 * returns a binary number in its decimal form. Each bit is the state of a gate.
 * bit3 bit2 bit1 bit0 = drogue1 drogue2 prop1 prop2
 */
uint8_t MRT_getContinuity(void){
	uint8_t drogue1 = HAL_GPIO_ReadPin(IN_EJ_Drogue_Cont_GPIO_Port, IN_EJ_Drogue_Cont_Pin);
	uint8_t drogue2 = HAL_GPIO_ReadPin(IN_EJ_Main_Cont_GPIO_Port, IN_EJ_Main_Cont_Pin);
	uint8_t prop1 = HAL_GPIO_ReadPin(IN_PyroValve_Cont_1_GPIO_Port, IN_PyroValve_Cont_1_Pin);
	uint8_t prop2 = HAL_GPIO_ReadPin(IN_PyroValve_Cont_2_GPIO_Port, IN_PyroValve_Cont_2_Pin);
	uint8_t continuity = 8*drogue1 + 4*drogue2 + 2*prop1 + prop2;
	return continuity;
}


// formats avionics telemetry string using sprintf
void MRT_formatAvionics(void) {
	memset(msg_buffer_av, 0, 200);
	sprintf(msg_buffer_av, "S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.7f,%03.7f,%02d,%02d,%lu,%d,%d,E\r\n",
		hlsm6dsr.acceleration_mg[0],	hlsm6dsr.acceleration_mg[1],	hlsm6dsr.acceleration_mg[2],
		hlsm6dsr.angular_rate_mdps[0],	hlsm6dsr.angular_rate_mdps[1],	hlsm6dsr.angular_rate_mdps[2],
		hlps22hh.pressure_hPa,	hgps.latitude,	hgps.longitude,
		prev_min, prev_sec, prev_subsec,
		ejection_stage_flag, gates_continuity);
}





/*Normal acceleration*/
float MRT_getAccNorm(void){
	float acc_x2 = pow(hlsm6dsr.acceleration_mg[0]/100,2);
	float acc_y2 = pow(hlsm6dsr.acceleration_mg[1]/100,2);
	float acc_z2 = pow(hlsm6dsr.acceleration_mg[2]/100,2);
	return sqrtf(acc_x2 + acc_y2 + acc_z2);
}



/*LSL in regression */

//PFP
float filterAltitude(float altitude);
void storeAltitude(float new_altitude, float cTime);



//For LSL in regression

//50 initial values to ensure that if the board wakes up mid-flight, it doesn't have to poll
//initial values (in case it's going down we don't have much time to act)
//and allow to not store "initial ground values". All the same so we can simply do ctrl-f and replace
#if NUM_MEAS_REG == 50
float alt_previous[NUM_MEAS_REG] = {-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0};
float time_previous[NUM_MEAS_REG] = {-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0};
float timalt_previous[NUM_MEAS_REG] = {-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0};
float timsqr_previous[NUM_MEAS_REG] = {-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,
									-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0};
#else
float alt_previous[NUM_MEAS_REG] = {-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0};
float time_previous[NUM_MEAS_REG] = {-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0};
float timalt_previous[NUM_MEAS_REG] = {-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0};
float timsqr_previous[NUM_MEAS_REG] = {-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0,-9999999.0};
#endif

uint32_t prevTick = 0;
float T;						// sampling period, time of each loop
extern float SmoothData;
extern float alt_ground;
extern float alt_filtered;
float LPF_Beta = 0.8;
uint8_t currElem = 0;



float LSLinRegression(void) {
	float xsum = 0, ysum = 0, xy = 0, xsqr = 0;

	for (uint8_t i = 0; i < NUM_MEAS_REG; i++) {
		xsum += time_previous[i];
	    ysum += alt_previous[i];
	    xy += timalt_previous[i];
	    xsqr += timsqr_previous[i];
	}

	return (float)(NUM_MEAS_REG*xy - (xsum*ysum))/(NUM_MEAS_REG*xsqr - (xsum*xsum));
}



float runAltitudeMeasurements(uint32_t currTick, uint16_t currAlt){
  T = (float)(currTick - prevTick);

  prevTick = currTick;
  //TODO JASPER'S CODE
// float alt_meas = currAlt - alt_ground; // Measures AGL altitude in feet
//  alt_filtered = filterAltitude(alt_meas);
//  float alt_filtered = alt_meas; // disable filtering for now

  //TODO I SIMPLY USE THE ALTITUDE AND DON'T CONSIDER GROUND
  float alt_filtered = currAlt;
  storeAltitude(alt_filtered, currTick);
  return alt_filtered;
}



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
