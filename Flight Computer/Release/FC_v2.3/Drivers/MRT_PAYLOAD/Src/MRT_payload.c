/*
 * payload.c
 *
 *  Created on: June 10, 2022
 *      Author: thomas
 */

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_telemetry.h>
#include <MRT_iridium.h>
#include <iwdg.h>
#include <string.h>
#include <MRT_i2c_sensors.h> //Format Iridium

void MRT_payloadInit(void) {
	HAL_StatusTypeDef ret;
	uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
	HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, SET);
	HAL_Delay(100);
	payload_buffer[0] = DATA_REG;
	ret = HAL_I2C_Master_Transmit(&hi2c2, TEENSY_ADDRESS, payload_buffer, 50, 100);
	if (ret != HAL_OK){
		println("Error1\r\n");
	}else{
		ret = HAL_I2C_Master_Receive(&hi2c2, TEENSY_ADDRESS, payload_buffer, 1, 100);
		if (ret != HAL_OK){
			println("Error2\r\n");
		}else{
			println(payload_buffer);
			if(strcmp('h',(char)payload_buffer[0])){
				println("PAYLOAD SETUP SUCCESFULL");
			} else {
				println("PAYLOAD ERROR!");
			}
		}
	}
	HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, RESET);
}

uint8_t MRT_payloadPoll(void) {
	HAL_StatusTypeDef ret;
	uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
	HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, SET);
	HAL_Delay(100);
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
}
