/*
 * MRT_ejection.c
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#include <MRT_setup.h>
#include <MRT_ejection.h>
#include <math.h>
#include <gpio.h>


//Global variables
uint8_t gates_continuity = 0;



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
