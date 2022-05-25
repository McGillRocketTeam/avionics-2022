/*
 * xtend.h
 *
 *  Created on: Apr 6, 2022
 *      Author: jasper
 */

#ifndef MRT_XTEND_INC_XTEND_H_
#define MRT_XTEND_INC_XTEND_H_

#include <stdint.h>
#include "usart.h"
#include "stm32f4xx.h"

#define XTEND_UART	huart3
#define XTEND_TIM	TIM3	// timer generating interrupts to send telemetry
#define XTEND_TIM_H	htim3	// timer handle

#define XTEND_USE_DMA	0

extern volatile char xtend_rx_buf[10];	// public for DMA
extern const char xtend_ack_msg[11];	// public for callback
extern volatile uint8_t num_radio_transmissions;

HAL_StatusTypeDef xtend_init(void);
void xtend_tx(uint8_t *msg_buffer, uint16_t size);
void xtend_sleep(void);
void xtend_wake(void);

#endif /* MRT_XTEND_INC_XTEND_H_ */
