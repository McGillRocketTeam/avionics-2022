/*
 * MRT_telemetry.c
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_telemetry.h>
#include <half_byte_encoder.h>
#include <MRT_iridium.h>
#include <iwdg.h>
#include <string.h>
#include <sx126x.h>
#include <MRT_i2c_sensors.h> //Format Iridium
#include <math.h> //pow function
#include <MRT_memory.h> //ejection_stage_flag
#include <MRT_ejection.h>

char iridium_buffer[IRIDIUM_BUFFER_SIZE];
uint8_t current_pos = 1; //Starting after first delimiting character

uint8_t encoded_buf[RADIO_BUFFER_SIZE];
uint8_t encoded_buf_len;
uint8_t decoded_buf[RADIO_BUFFER_SIZE];
uint8_t decoded_buf_len;

void MRT_float_to_4char(float f, char* receiving_buffer);


void MRT_radio_tx(char* buffer){

	#if RADIO_PRINT
	print((char*) "Radio sending:\t");
	println(buffer);
	#endif

	#if HALF_BYTE_
	//Encode the buffer to send
	memset(encoded_buf,0,RADIO_BUFFER_SIZE);
	hb_encode_string((uint8_t*) buffer, strlen(buffer), encoded_buf, &encoded_buf_len);
	#else
	memset(encoded_buf,0,RADIO_BUFFER_SIZE);
	memcpy(encoded_buf, buffer, strlen(buffer));
	encoded_buf_len = strlen(encoded_buf);
	#endif

	#if XTEND_ //Xtend send
		if (strlen(encoded_buf) < XTEND_BUFFER_SIZE)	HAL_UART_Transmit(&XTEND_UART,(uint8_t*) encoded_buf, encoded_buf_len, HAL_MAX_DELAY);
	#elif SRADIO_ //SRadio send
		if (strlen(encoded_buf) < SRADIO_BUFFER_SIZE){
			//sx126x_set_tx(&SRADIO_SPI, 1000, SRADIO_BUFFER_SIZE);
			if(ejection_stage_flag == PAD) Tx_setup(); //Only necessary when doing bidirectionnal
			TxProtocol((uint8_t*) encoded_buf, encoded_buf_len);
		}
	#endif
}


void MRT_radio_rx(char* buffer, uint8_t size, uint16_t timeout){
	#if XTEND_ //Xtend receive
		if (size < XTEND_BUFFER_SIZE){
			HAL_UART_Receive(&XTEND_UART,(uint8_t*) buffer, sizeof(char) * size, timeout);
		}
	#elif SRADIO_ //SRadio receive
		if (size < SRADIO_BUFFER_SIZE){
			//sx126x_set_rx(&SRADIO_SPI,5000);
			Rx_setup();

			//Note: The last character is always random and needs to be removed
			char temp_buf[size];
			memset(temp_buf,0,size);
			RxProtocol((uint8_t*) temp_buf);
			//memcpy(buffer,temp_buf,strlen(temp_buf)-1);
			memcpy(buffer,temp_buf,size);
		}
	#endif

	#if HALF_BYTE_
	//Decode the buffer received
	memset(decoded_buf,0,RADIO_BUFFER_SIZE);
	hb_decode_string((uint8_t*) buffer, strlen(buffer), decoded_buf, &decoded_buf_len);
	memcpy(buffer, decoded_buf, decoded_buf_len);
	#endif

	#if RADIO_PRINT
	print((char*) "Radio receiving:\t");
	println((char*) buffer);
	#endif
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
	set_NSS_pin(SPI2_SX_CS_GPIO_Port, SPI2_SX_CS_Pin);
	set_BUSY_pin(SX_BUSY_GPIO_Port, SX_BUSY_Pin);
	set_NRESET_pin(SX_RST_GPIO_Port, SX_RST_Pin);
	set_DIO1_pin(SX_DIO_GPIO_Port, SX_DIO_Pin);
	//  set_DIO2_pin(DIO2_1_GPIO_Port, DIO2_1_Pin);
	//  set_DIO3_pin(DIO3_1_GPIO_Port, DIO3_1_Pin);
	Tx_setup();
	//Rx_setup();
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
		hiridium = MRT_Iridium_Init(IRIDIUM_FLIGHT_TIMEOUT, IRIDIUM_I2C, print);
		#else
		hiridium = MRT_Iridium_Init(IRIDIUM_FLIGHT_TIMEOUT, IRIDIUM_I2C, no_print); //Doesn't print the internal commands
		#endif
	#endif
}



void MRT_radio_send_ack(radio_command cmd){
	if(cmd>=0 && cmd<=9){
		char buffer[20];
		sprintf(buffer, "radio_ack_%i\r\n",cmd);
		MRT_radio_tx(buffer);
	}
}


int MRT_formatIridium(void){
	//We know that the GPS coordinates will always have 7 digits after the coma.
	//The max extrema value for latitude are -90 and 90 and it's -180 and 180 for longitude
	//The max value for longitude in binary is therefore (sign bit)(0 padded)110101 10100100 11101001 00000000
	//The max value for latitude in binary is therefore (sign bit)1101011 01001001 11010010 00000000
	//We can fit the pair inside 2x 4 bytes. This reprensents a maximum of 6 pairs inside one message (with 2 bytes left)¸
	iridium_buffer[0] = 'S';
	iridium_buffer[IRIDIUM_BUFFER_SIZE-1] = 'E';
	memcpy(iridium_buffer+current_pos,&hgps.latitude,4);
	memcpy(iridium_buffer+current_pos+4,&hgps.longitude,4);
	current_pos += 8;

	if(current_pos >= IRIDIUM_BUFFER_SIZE-7){
		current_pos = 1;
		return 1;
	}
	return -1;
}

