/*
 * helpers.c
 *
 *  Created on: Apr 5, 2022
 *      Author: jasper
 */

#include "main.h"
#include "helpers.h"

#include "iwdg.h"

#include <string.h>

extern TIM_HandleTypeDef 	BUZZER_TIM_PWM;
extern UART_HandleTypeDef	DEBUG_UART;

// buzz at particular frequency
void tone_freq(uint32_t duration, uint32_t repeats, uint32_t freq) {
	// TIM2 base frequency is 90 MHz, PSC = 90-1
	// can calculate required ARR value
	TIM2->ARR = 1000000 / freq;
	TIM2->EGR |= TIM_EGR_UG;

	for (uint32_t i = 0; i < repeats; i++) {
		HAL_TIM_PWM_Start(&BUZZER_TIM_PWM, BUZZER_TIM_CH);
		HAL_GPIO_WritePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin, SET);
		HAL_Delay(duration);
		HAL_TIM_PWM_Stop(&BUZZER_TIM_PWM, BUZZER_TIM_CH);
		HAL_GPIO_WritePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin, RESET);
		if (repeats > 1)
			HAL_Delay(duration);
	}
}

void buzz_success(void) {
	tone_freq(BUZZ_SUCCESS_DURATION, BUZZ_SUCCESS_REPEATS, BUZZ_SUCCESS_FREQ);
}

void buzz_failure(void) {
	tone_freq(BUZZ_FAILURE_DURATION, BUZZ_FAILURE_REPEATS, BUZZ_FAILURE_FREQ);
}

void buzz_startup_success(void) {
	for (uint8_t i = 0; i < 3; i++) {
		buzz_success();
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
		HAL_Delay(BUZZ_STARTUP_DELAY);
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
	}
}

void debug_tx_uart(uint8_t *msg) {
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)msg, strlen((const char *)msg), HAL_MAX_DELAY);
}
