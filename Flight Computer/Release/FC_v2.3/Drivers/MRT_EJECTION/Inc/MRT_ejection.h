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

extern uint8_t gates_continuity;

float MRT_getAltitude(float pressure);
uint8_t MRT_getContinuity(void);


#ifdef __cplusplus
}
#endif

#endif /* MRT_EJECTION_INC_MRT_EJECTION_H_ */
