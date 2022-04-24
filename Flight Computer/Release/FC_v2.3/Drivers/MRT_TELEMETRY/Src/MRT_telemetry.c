/*
 * MRT_telemetry.c
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#include <MRT_telemetry.h>
#include <MRT_helpers.h>
#include <MRT_iridium.h>
#include <MRT_setup.h>
#include <iwdg.h>
#include <string.h>
#include <sx126x.h>


void MRT_radio_tx(char* buffer){
	#if XTEND_ //Xtend send
		if (strlen(buffer) < XTEND_BUFFER_SIZE)	XTend_Transmit(buffer);
	#elif SRADIO_ //SRadio send TODO
		if (strlen(buffer) < SRADIO_BUFFER_SIZE){
			//sx126x_set_tx(); TODO
			TxProtocol(buffer, strlen(buffer));
		}
	#endif
}


void MRT_radio_rx(char* buffer, uint8_t size){
	#if XTEND_ //Xtend receive
		if (size < XTEND_BUFFER_SIZE){
			HAL_UART_Receive(&XTEND_UART, buffer, sizeof(char) * size, 0x500); //Timeout is about 1.2 sec (should be less than 5 sec)
		}
	#elif SRADIO_ //SRadio receive TODO
		if (size < SRADIO_BUFFER_SIZE){
			//sx126x_set_rx(); TODO??
			RxProtocol(buffer)
		}
	#endif
}



void MRT_radio_Init(void){
	#if XTEND_
	HAL_GPIO_WritePin(XTend_CTS_Pin, GPIO_PIN_10, GPIO_PIN_RESET);
	#elif SRADIO_
	set_hspi(SRADIO_SPI);
	// SPI2_SX_CS_GPIO_Port TODO ???
	set_NSS_pin(SPI2_SX_CS_GPIO_Port, SPI2_SX_CS_Pin);
	set_BUSY_pin(SX_BUSY_GPIO_Port, SX_BUSY_Pin);
	set_NRESET_pin(SX_RST_GPIO_Port, SX_RST_Pin);
	set_DIO1_pin(SX_DIO_GPIO_Port, SX_DIO_Pin);
	Tx_setup();
	#endif
}


void MRT_TELEMETRY_Init(void){

	MRT_radio_Init();

	HAL_IWDG_Refresh(&hiwdg);

	#if IRIDIUM_
	HAL_GPIO_WritePin(Iridium_RST_GPIO_Port, Iridium_RST_Pin, SET);
	hiridium = MRT_Iridium_Init(IRIDIUM_TIMEOUT, IRIDIUM_I2C, print);
	#endif
}

