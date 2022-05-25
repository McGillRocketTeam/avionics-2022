/*
 * helpers.h
 *
 *  Created on: Apr 5, 2022
 *      Author: jasper
 */

#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#define BUZZER_TIM_PWM	htim2
#define BUZZER_TIM_CH	TIM_CHANNEL_3

#define DEBUG_UART		huart8

// buzzer durations
#define BUZZ_SUCCESS_DURATION	100		// ms
#define BUZZ_SUCCESS_REPEATS	2
#define BUZZ_SUCCESS_FREQ		1046	// Hz
#define BUZZ_STARTUP_DELAY		1000	// ms

#define BUZZ_FAILURE_DURATION	1000 	// ms
#define BUZZ_FAILURE_REPEATS	3
#define	BUZZ_FAILURE_FREQ		220		// Hz

// public buzzer functions
void buzz_success(void);
void buzz_failure(void);
void buzz_startup_success(void);

void debug_tx_uart(uint8_t *msg);

#endif /* INC_HELPERS_H_ */
