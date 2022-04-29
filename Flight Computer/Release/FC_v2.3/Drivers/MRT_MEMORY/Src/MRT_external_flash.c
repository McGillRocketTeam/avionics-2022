/*
 * MRT_external_flash.c
 *
 *  Created on: Apr 12, 2022
 *      Author: Jacoby
 */


#include <MRT_external_flash.h>


//**************************************************//
//FLAGS

uint8_t ext_flash_reset = 0; //In external flash memory
uint8_t ext_flash_wu = 0; //TODO used to be defined in MRT_RTOS.c
uint8_t ext_flash_iwdg = 0; //In external flash memory
uint8_t ext_flash_apogee = 0; //In external flash memory
uint8_t ext_flash_ejection_stage = 0; //In external flash memory

//Flags read/write buffer
uint8_t ext_flash_flags_buffer[NB_OF_FLAGS];

//Reference array to each flag
uint8_t* ext_flash_flags[NB_OF_FLAGS] = {&ext_flash_reset, &ext_flash_wu, &ext_flash_iwdg, &ext_flash_apogee, &ext_flash_ejection_stage};

//Null buffer values for when clearing flags
uint8_t FLAGS_NULL_BUFFER[NB_OF_FLAGS];


//**************************************************//
//RTC TIME FLAGS

/*RTC time*/
//Time variables
uint8_t ext_flash_hour = 0; //Last recorded hours
uint8_t ext_flash_min = 0; //Last recorded minutes
uint8_t ext_flash_sec = 0; //Last recorded seconds
uint32_t ext_flash_subsec = 0; //Last recorded subseconds

//Time read/write buffer
uint8_t ext_flash_time_buffer[RTC_NB_OF_VAR];

//Reference list to each time component
uint8_t* ext_flash_time[RTC_NB_OF_VAR] = {&ext_flash_hour, &ext_flash_min, &ext_flash_sec, &ext_flash_subsec}; //TODO ext_flash_subsec value will overflow

//Null buffer values for when clearing time
uint8_t RTC_TIME_NULL_BUFFER[RTC_NB_OF_VAR] = {0,0,0,0};



//**************************************************//
//FUNCTIONS

void MRT_external_flash_Init(void){

	for (int i = 0; i < NB_OF_FLAGS; i++){
		FLAGS_NULL_BUFFER[i] = 0; //Setup the flags null buffer for the correct number of values
	}

	if (!W25qxx_Init()) {
		Error_Handler(); // hangs and blinks LEDF
	}

	//Retrieve flags
	W25qxx_ReadSector(ext_flash_flags_buffer, FLAGS_SECTOR, FLAGS_OFFSET, NB_OF_FLAGS);

	//Retrieve RTC time (last recorded)
	W25qxx_ReadSector(ext_flash_time_buffer, RTC_SECTOR, RTC_TIME_OFFSET, RTC_NB_OF_VAR);

	//Assign each value read to their variable
	MRT_updateExternalFlashValues();
}


void MRT_updateExternalFlashBuffers(void){
	for (int i = 0; i < NB_OF_FLAGS; i++){
		ext_flash_flags_buffer[i] = *ext_flash_flags[i];
	}
	for (int i = 0; i < RTC_NB_OF_VAR; i++){
		ext_flash_time_buffer[i] = *ext_flash_time[i];
	}
}


void MRT_updateExternalFlashValues(void){
	for (int i = 0; i < NB_OF_FLAGS; i++){
		*ext_flash_flags[i] = ext_flash_flags_buffer[i];
	}
	for (int i = 0; i < RTC_NB_OF_VAR; i++){
		*ext_flash_time[i] = ext_flash_time_buffer[i];
	}
}
