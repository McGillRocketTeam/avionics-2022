/*
 * payload.c
 *
 *  Created on: June 10, 2022
 *      Author: thomas
 */

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_payload.h>
#include <MRT_telemetry.h>
#include <MRT_iridium.h>
#include <iwdg.h>
#include <string.h>
#include <MRT_i2c_sensors.h> //Format Iridium
#include "cmsis_os2.h" //osDelay

uint8_t payload_init_success = 0;
uint8_t payload_init_counter = 0;
uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];

void MRT_payloadInit(void) {
	#if PAYLOAD_
	print((char*) "Payload Init...");
	payload_init_counter++;
	if(payload_init_counter==6){
		payload_init_success = 1;
	}
	HAL_StatusTypeDef ret;
	uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
	HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, SET);
	osDelay(100);
	payload_buffer[0] = DATA_REG;
	ret = HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDRESS, payload_buffer, PAYLOAD_BUFFER_SIZE, 100);
	if (ret != HAL_OK){
		println((char*) "Error1\r\n");
	}else{
		ret = HAL_I2C_Master_Receive(&hi2c2, TEENSY_ADDRESS, payload_buffer, 1, 100);
		if (ret != HAL_OK){
			println((char*) "Error2\r\n");
		}else{
			println(payload_buffer);
			if(strcmp('h',(char)payload_buffer[0])){
				println((char*) "OK");
				payload_init_success = 1;
			} else {
				println((char*) "PAYLOAD ERROR!");
			}
		}
	}
	HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, RESET);
	#else
	payload_init_success = 1;
	#endif
}

uint8_t MRT_payloadPoll(void) {
	#if PAYLOAD_
	HAL_StatusTypeDef ret;
	HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, SET);
	osDelay(100);
	ret = HAL_I2C_Master_Receive(&hi2c2, TEENSY_ADDRESS, payload_buffer, 25, 100);
	if (ret != HAL_OK){
		println("Error2\r\n");
		HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, RESET);
		return 0;
	}
	ret = HAL_I2C_Master_Receive(&hi2c2, TEENSY_ADDRESS, &payload_buffer[25], 25, 100);
	if (ret != HAL_OK){
		println("Error2\r\n");
		HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, RESET);
		return 0;
	}
	HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, RESET);
	memcpy(iridium_buffer,payload_buffer, 50);
	return 1;
	#else
	return 0;
	#endif
}
