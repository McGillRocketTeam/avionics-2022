/*
 * MRT_propulsion.h
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#ifndef MRT_PROPULSION_INC_MRT_PROPULSION_H_
#define MRT_PROPULSION_INC_MRT_PROPULSION_H_

#ifdef __cplusplus
extern "C" {
#endif

extern float thermocouple_temperature;
extern float transducer_voltage;
extern uint8_t valve_status;

void MRT_pollPropulsion(void);
void MRT_getThermoTemp(void);
void MRT_getTransducerVoltage(void);
void MRT_getValveStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* MRT_PROPULSION_INC_MRT_PROPULSION_H_ */
