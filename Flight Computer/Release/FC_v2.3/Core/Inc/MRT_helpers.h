/*
 * MRT_helpers.h
 *
 *  Created on: Apr 20, 2022
 *      Author: Jacoby
 */

#ifndef INC_MRT_HELPERS_H_
#define INC_MRT_HELPERS_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdio.h>
#include <string.h> //memset


//Function prototypes
void println(char* s);
void print(char* s);
void tone_freq(uint32_t duration, uint32_t repeats, uint32_t freq);
void buzz_success(void);
void buzz_failure(void);
void buzz_startup_success(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#ifdef __cplusplus
}
#endif


#endif /* INC_MRT_HELPERS_H_ */

