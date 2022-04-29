/*
 * MRT_telemetry.c
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_telemetry.h>
#include <MRT_iridium.h>
#include <iwdg.h>
#include <string.h>
#include <sx126x.h>


void MRT_radio_tx(char* buffer){
	#if XTEND_ //Xtend send
		if (strlen(buffer) < XTEND_BUFFER_SIZE)	HAL_UART_Transmit(&XTEND_UART,(uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
	#elif SRADIO_ //SRadio send TODO
		if (strlen(buffer) < SRADIO_BUFFER_SIZE){
			//sx126x_set_tx(); TODO
			TxProtocol(buffer, strlen(buffer));
		}
	#endif

	print((char*) "Radio sending:\t");
	println(buffer);
}


void MRT_radio_rx(char* buffer, uint8_t size, uint16_t timeout){
	#if XTEND_ //Xtend receive
		if (size < XTEND_BUFFER_SIZE){
			HAL_UART_Receive(&XTEND_UART,(uint8_t*) buffer, sizeof(char) * size, timeout);
		}
	#elif SRADIO_ //SRadio receive TODO
		if (size < SRADIO_BUFFER_SIZE){
			//sx126x_set_rx(); TODO??
			RxProtocol(buffer)
		}
	#endif

	print((char*) "Radio receiving:\t");
	println(buffer);
}



void MRT_radio_Init(void){
	println("\r\nRadio Init");
	#if XTEND_
	print("\tXTEND Init...");
	HAL_GPIO_WritePin(XTend_CTS_Pin, GPIO_PIN_10, GPIO_PIN_RESET);
	println("OK");
	#elif SRADIO_
	print("\tSRADIO Init...");
	set_hspi(SRADIO_SPI);
	// SPI2_SX_CS_GPIO_Port TODO ???
	set_NSS_pin(SPI2_SX_CS_GPIO_Port, SPI2_SX_CS_Pin);
	set_BUSY_pin(SX_BUSY_GPIO_Port, SX_BUSY_Pin);
	set_NRESET_pin(SX_RST_GPIO_Port, SX_RST_Pin);
	set_DIO1_pin(SX_DIO_GPIO_Port, SX_DIO_Pin);
	Tx_setup();
	println("OK");
	#else
	println("\tNo radio currently in use");
	#endif
}


void MRT_TELEMETRY_Init(void){

	MRT_radio_Init();

	HAL_IWDG_Refresh(&hiwdg);

	#if IRIDIUM_
		HAL_GPIO_WritePin(Iridium_RST_GPIO_Port, Iridium_RST_Pin, SET);
		#if IRIDIUM_INTERNAL_PRINT
		hiridium = MRT_Iridium_Init(IRIDIUM_TIMEOUT, IRIDIUM_I2C, print);
		#else
		hiridium = MRT_Iridium_Init(IRIDIUM_TIMEOUT, IRIDIUM_I2C, no_print); //Doesn't print the internal commands
		#endif
	#endif
}







