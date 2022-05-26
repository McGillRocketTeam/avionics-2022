/*
 * MRT_ejection.h
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#ifndef MRT_EJECTION_INC_MRT_EJECTION_H_
#define MRT_EJECTION_INC_MRT_EJECTION_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef enum ejection_state_t {
	PAD = 0,
	BOOST,
	DROGUE_DESCENT,
	MAIN_DESCENT,
	LANDED,

} ejection_state_t;


extern uint8_t gates_continuity;
extern char msg_buffer_av[200];

float MRT_getAltitude(float pressure);
uint8_t MRT_getContinuity(void);
void MRT_formatAvionics(void);

//LSL in regression
float LSLinRegression(void);
float runAltitudeMeasurements(uint32_t currTick, uint16_t currAlt);


#ifdef __cplusplus
}
#endif

#endif /* MRT_EJECTION_INC_MRT_EJECTION_H_ */
