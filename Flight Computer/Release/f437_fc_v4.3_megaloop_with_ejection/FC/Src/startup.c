/*
 * startup.c
 *
 *  Created on: Apr 6, 2022
 *      Author: jasper
 */

#include "startup.h"
#include "main.h"

// initialize gpio states at startup.
// is_watchdog_restart = 1 if watchdog reset occurred
void init_gpio_startup(uint8_t is_watchdog_restart) {
	// turn on LED near vent hole to show that FC is on
	HAL_GPIO_WritePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin, SET);

	// FLASH set CS, WP and IO3 pins high
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);
	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, SET);
	HAL_GPIO_WritePin(FLASH_IO3_GPIO_Port, FLASH_IO3_Pin, SET);

	// set other SPI CS pins high
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, SET);
	HAL_GPIO_WritePin(TH_CS_GPIO_Port, TH_CS_Pin, SET);

	// reset LEDs
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
	HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, RESET);

	if (!is_watchdog_restart) {
		// reset recovery pyro pins
		HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, RESET);
		HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
		HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);

		// reset prop pyro pins
		HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, RESET);
		HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
		HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);
	}

	// enable 12 V regulator (which is active high)
	HAL_GPIO_WritePin(PM_12V_EN_GPIO_Port, PM_12V_EN_Pin, SET);

	// no power to dump valve
	HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET);

	// reset payload EN signal
	HAL_GPIO_WritePin(Payload_EN_GPIO_Port, Payload_EN_Pin, RESET);

	// set power off for VR
	HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, RESET);
	HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);
}
