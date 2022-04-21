/*
 * MRT_helpers.c
 *
 *  Created on: Apr 20, 2022
 *      Author: Jacoby
 */

#include <MRT_helpers.h>
#include <MRT_setup.h>
#include <tim.h>

void println(char* s){
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) s, strlen(s), HAL_MAX_DELAY);
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);
}

void print(char* s){
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t*) s, strlen(s), HAL_MAX_DELAY);
}

// buzz at particular frequency
void tone_freq(uint32_t duration, uint32_t repeats, uint32_t freq) {
	// TIM2 base frequency is 90 MHz, PSC = 90-1
	// can calculate required ARR value
	TIM2->ARR = 1000000 / freq;
	TIM2->EGR |= TIM_EGR_UG;

	for (uint32_t i = 0; i < repeats; i++) {
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin, SET);
		HAL_Delay(duration);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin, RESET);
		if (repeats > 1)
			HAL_Delay(duration);
	}
}
void buzz_success(void) { tone_freq(BUZZ_SUCCESS_DURATION, BUZZ_SUCCESS_REPEATS, BUZZ_SUCCESS_FREQ); }
void buzz_failure(void) { tone_freq(BUZZ_FAILURE_DURATION, BUZZ_FAILURE_REPEATS, BUZZ_FAILURE_FREQ); }
void buzz_startup_success(void) {
	for (uint8_t i = 0; i < 3; i++) {
		buzz_success();
		HAL_Delay(1000);
	}
}



/*
 * Gets the altitude using temperature, pressure and sea-level pressure
 *https://www.mide.com/air-pressure-at-altitude-calculator
 */
float MRT_get_altitude(float pressure){
	return BASE_HEIGHT+(SEA_LEVEL_TEMPERATURE/-0.0065)*(pow(pressure/SEA_LEVEL_PRESSURE,0.190263236)-1); //(-R*-0.0065/(go*M)) = 0.190263236
}
