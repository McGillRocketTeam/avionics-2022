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

//Watch out for double-evaluation
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))


//Function prototypes
void println(char* s);
void print(char* s);
void no_print(char* s); //Buffer function for iridium
void tone_freq(uint32_t duration, uint32_t repeats, uint32_t freq);
void buzz_success(void);
void buzz_failure(void);
void buzz_fc_on(void);
void buzz_startup_success(void);


#ifdef __cplusplus
}
#endif


#endif /* INC_MRT_HELPERS_H_ */

