/*
 * ejection.c
 *
 *  Created on: Mar 18, 2021
 *      Author: a cat
 */

#include "ejection.h"

uint32_t prevTick = 0;
float T;						// sampling period, time of each loop
extern float SmoothData;
extern float alt_ground;
extern float alt_filtered;
float LPF_Beta = 0.8;
uint8_t currElem = 0;

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
	SmoothData -= LPF_Beta * (SmoothData - altitude);
	return SmoothData;
}

void storeAltitude(float new_altitude, float cTime) {

	alt_previous[currElem] = new_altitude;
	time_previous[currElem] = cTime;
	timalt_previous[currElem] = cTime * new_altitude;
	timsqr_previous[currElem] = cTime * cTime;

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
