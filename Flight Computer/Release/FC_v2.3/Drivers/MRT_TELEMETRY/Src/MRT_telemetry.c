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
#include <MRT_i2c_sensors.h> //Format Iridium
#include <math.h> //pow function

char iridium_buffer[IRIDIUM_BUFFER_SIZE];

void MRT_float_to_4char(float f, char* receiving_buffer);


void MRT_radio_tx(char* buffer){
	#if XTEND_ //Xtend send
		if (strlen(buffer) < XTEND_BUFFER_SIZE)	HAL_UART_Transmit(&XTEND_UART,(uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
	#elif SRADIO_ //SRadio send TODO
		if (strlen(buffer) < SRADIO_BUFFER_SIZE){
			//sx126x_set_tx(&SRADIO_SPI, 1000, SRADIO_BUFFER_SIZE);
			Tx_setup();
			TxProtocol((uint8_t*) buffer, strlen(buffer));
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
	/*
	//xxx.xxxxxxx format of coordinates %03.7f (need 11 bytes per latitude/longitude, thus 22 bytes per message.)
	//This means that we can only send two pairs for 1 credit and have 6 bytes left for payload (50 bytes per credit)
	//What about null terminating byte of string? should we allow only 5 bytes for payload?
	char buffer[22];
	sprintf(buffer, "%03.7f%03.7f", hgps.latitude, hgps.longitude);
	for(int i=0; i<IRIDIUM_BUFFER_SIZE; i+=22){
		if (iridium_buffer[i]==0){
			//If you find a free spot, insert data and return
			memcpy(iridium_buffer[i],buffer,22);
			return 1;
		}
	}
	//TODO add something for payload data
	return -1;
	*/

	//We know that the GPS coordinates will always have 7 digits after the coma.
	//The max extrema value for latitude are -90 and 90 and it's -180 and 180 for longitude
	//The max value for longitude in binary is therefore (sign bit)(0 padded)110101 10100100 11101001 00000000
	//The max value for latitude in binary is therefore (sign bit)1101011 01001001 11010010 00000000
	//We can fit the pair inside 2x 4 bytes. This reprensents a maximum of 6 pairs inside one message (with 2 bytes left)
	char lat[4];
	char lng[4];
	MRT_float_to_4char(hgps.latitude, lat);
	MRT_float_to_4char(hgps.longitude, lng);

	for(int i=0; i<IRIDIUM_BUFFER_SIZE; i+=8){
		if (iridium_buffer[i]==0){
			//If you find a free spot, insert data and return
			memcpy(iridium_buffer+i,lat,4);
			memcpy(iridium_buffer+i+4,lng,8);
			return 1;
		}
	}
	//TODO add something for payload data
	return -1;
}


void MRT_float_to_4char(float f, char* receiving_buffer){

	uint8_t binary[32];
	binary[0] = 0; //Sign

	uint32_t num;

	num = f*10000000; //Integer representation of the float (only keep 7 first digits after comma)
	if (num<0) binary[0] = 1; //Set sign if applicable

	//Convert integer to a 32 bits binary number
	for (uint8_t i=32; i>0; i--){ //Excludes sign part
		binary[i] = num % 2;
		num = num/2;
	}

	//Split binary number in 4 integers (really not efficient)
	uint8_t char1 = pow(2,7)*binary[0] + pow(2,6)*binary[1] + pow(2,5)*binary[2] + pow(2,4)*binary[3] + pow(2,3)*binary[4] + pow(2,2)*binary[5] + pow(2,1)*binary[6] + binary[7];
	uint8_t char2 = pow(2,7)*binary[8] + pow(2,6)*binary[9] + pow(2,5)*binary[10] + pow(2,4)*binary[11] + pow(2,3)*binary[12] + pow(2,2)*binary[13] + pow(2,1)*binary[14] + binary[15];
	uint8_t char3 = pow(2,7)*binary[16] + pow(2,6)*binary[17] + pow(2,5)*binary[18] + pow(2,4)*binary[19] + pow(2,3)*binary[20] + pow(2,2)*binary[21] + pow(2,1)*binary[22] + binary[23];
	uint8_t char4 = pow(2,7)*binary[24] + pow(2,6)*binary[25] + pow(2,5)*binary[26] + pow(2,4)*binary[27] + pow(2,3)*binary[28] + pow(2,2)*binary[29] + pow(2,1)*binary[30] + binary[31];

	//Convert to 4 characters string
	char buffer[5]; //Account for terminating bit (will remove later)
	sprintf(buffer,"%c%c%c%c",char1,char2,char3,char4);
	memcpy(receiving_buffer,buffer,4); //Copy only the 4 characters
}


