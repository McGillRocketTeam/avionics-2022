/*
 * xtend.c
 *
 * provides a wrapper for transmitting data over UART using the XTend.
 * currently uses UART DMA to transfer the data to the XTend.
 * does not provide flow control through RTS and CTS pins. perhaps a feature for the future.
 *
 * callback functions for DMA and timer interrupts are not in this file.
 *
 * consider further refactoring in the future.
 *
 *  Created on: Apr 6, 2022
 *      Author: jasper
 */

#include "xtend.h"
#include "helpers.h"
#include "main.h"
#include "string.h"

extern TIM_HandleTypeDef XTEND_TIM_H;

// bidirectional xtend communication
volatile char xtend_rx_buf[10];
const char xtend_ack_msg[11] = "xtend_ack"; // first part of ack message

volatile uint8_t num_radio_transmissions = 0;

HAL_StatusTypeDef xtend_init(void) {
	memset(xtend_rx_buf, 0, 10);
	return HAL_UART_Receive_DMA(&huart3, (uint8_t *)xtend_rx_buf, XTEND_RX_DMA_CMD_LEN);
}

// reception using UART DMA
void xtend_tx(uint8_t *msg_buffer, uint16_t size) {

	// TODO:
	//  - add RTS/CTS signals from xtend for flow control
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	#if XTEND_USE_DMA
		HAL_UART_Transmit_DMA(&XTEND_UART, msg_buffer, size);
	#else
		num_radio_transmissions++;
		if (num_radio_transmissions == 10) {
			num_radio_transmissions = 0;
		}
		HAL_UART_Transmit(&huart3, msg_buffer, size, 95);

	#endif

	#ifdef DEBUG_MODE
	debug_tx_uart(msg_buffer);
	#endif
}

void xtend_sleep(void) {
	HAL_GPIO_WritePin(XTend_SLEEP_GPIO_Port, XTend_SLEEP_Pin, RESET);
}

void xtend_wake(void) {
	HAL_GPIO_WritePin(XTend_SLEEP_GPIO_Port, XTend_SLEEP_Pin, SET);
}


