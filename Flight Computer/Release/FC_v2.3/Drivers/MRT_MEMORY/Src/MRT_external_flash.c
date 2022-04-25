/*
 * MRT_external_flash.c
 *
 *  Created on: Apr 12, 2022
 *      Author: Jacoby
 */


#include <MRT_external_flash.h>
#include <rtc.h>


//**************************************************//
//FLAGS

uint8_t reset_flag = 0; //In external flash memory
uint8_t wu_flag = 0; //TODO used to be defined in MRT_RTOS.c
uint8_t iwdg_flag = 0; //In external flash memory
uint8_t apogee_flag = 0; //In external flash memory
uint8_t ejection_state_flag = 0; //In external flash memory

//Flags read/write buffer
uint8_t flash_flags_buffer[NB_OF_FLAGS];

//Reference array to each flag
uint8_t* flash_flags[NB_OF_FLAGS] = {&reset_flag, &wu_flag, &iwdg_flag, &apogee_flag, &ejection_state_flag};

//Null buffer values for when clearing flags
uint8_t FLAGS_NULL_BUFFER[NB_OF_FLAGS];


//**************************************************//
//RTC TIME FLAGS

/*RTC time*/
//Time constants (determined at each reset)
uint8_t prev_hours = 0; //Last recorded hours
uint8_t prev_min = 0; //Last recorded minutes
uint8_t prev_sec = 0; //Last recorded seconds

//Time read/write buffer
uint8_t flash_time_buffer[3];

//Reference list to each time component
uint8_t* flash_time[3] = {&prev_hours, &prev_min, &prev_sec};

//Null buffer values for when clearing time
uint8_t RTC_TIME_NULL_BUFFER[3] = {0,0,0};



//**************************************************//
//FUNCTIONS

void MRT_external_flash_Init(void){

	for (int i = 0; i < NB_OF_FLAGS; i++){
		FLAGS_NULL_BUFFER[i] = 0; //Setup the flags null buffer for the correct number of values
	}

	if (!W25qxx_Init()) {
		Error_Handler(); // hangs and blinks LEDF
	}
	MRT_check_for_wake_up(); //Needs to be called before getFlags() and after the W25xx_Init()
	MRT_get_flags();
}




void MRT_get_flags(void){

	//Retrieve flags
	W25qxx_ReadSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);

	//Retrieve RTC time (last recorded)
	W25qxx_ReadSector(flash_time_buffer, 2, RTC_TIME_OFFSET, 3);

	//If RTC detected a wake up, update the flash memory
	if (wu_flag == 1){
		//Write the new number of wake up to external flash
		flash_flags_buffer[WU_FLAG_OFFSET] = flash_flags_buffer[WU_FLAG_OFFSET] + 1; //Update number of wake up
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}

	//Assign each value read to their variable
	 MRT_update_flags_values();


	//Check flags values
	//Reset flag
	if (reset_flag != 0 && reset_flag !=1){ //If random value (none was written)
		reset_flag = 0;
		flash_flags_buffer[RESET_FLAG_OFFSET] = reset_flag;
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}

	//Wake up flag
	if (wu_flag != 0 && wu_flag !=1 && wu_flag !=2){ //If random value (none was written)
		wu_flag = 0;
		flash_flags_buffer[WU_FLAG_OFFSET] = wu_flag;
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}

	//IWDG flag
	if (iwdg_flag != 0 && iwdg_flag !=1){ //If random value (none was written)
		iwdg_flag = 0;
		flash_flags_buffer[IWDG_FLAG_OFFSET] = iwdg_flag;
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}

	//Apogee flag
	if (apogee_flag != 0 && apogee_flag !=1){ //If random value (none was written)
		apogee_flag = 0;
		flash_flags_buffer[APOGEE_FLAG_OFFSET] = apogee_flag;
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}

	//Ejection state flag
	if (!(ejection_state_flag >= 0 && ejection_state_flag <=4)){ //If random value (none was written)
		ejection_state_flag = 0;
		flash_flags_buffer[EJECTION_STATE_FLAG_OFFSET] = ejection_state_flag;
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}


	//Check RTC time values
	//Hours
	if (!(prev_hours >= 0 && prev_hours < 24)){ //If random value (none was written)
		prev_hours = 0;
		flash_time_buffer[RTC_HOURS_OFFSET] = prev_hours;
		W25qxx_EraseSector(2);
		W25qxx_WriteSector(flash_time_buffer, 2, RTC_TIME_OFFSET, 3);
	}

	//Minutes
	if (!(prev_min >= 0 && prev_min < 60)){ //If random value (none was written)
		prev_min = 0;
		flash_time_buffer[RTC_MIN_OFFSET] = prev_min;
		W25qxx_EraseSector(2);
		W25qxx_WriteSector(flash_time_buffer, 2, RTC_TIME_OFFSET, 3);
	}

	//Seconds
	if (!(prev_sec >= 0 && prev_sec < 60)){ //If random value (none was written)
		prev_sec = 0;
		flash_time_buffer[RTC_SEC_OFFSET] = prev_sec;
		W25qxx_EraseSector(2);
		W25qxx_WriteSector(flash_time_buffer, 2, RTC_TIME_OFFSET, 3);
	}
}



void MRT_update_external_flash_buffers(void){
	for (int i = 0; i < NB_OF_FLAGS; i++){
		flash_flags_buffer[i] = *flash_flags[i];
	}
	for (int i = 0; i < 3; i++){
		flash_time_buffer[i] = *flash_time[i];
	}
}


void MRT_update_flags_values(void){
	for (int i = 0; i < NB_OF_FLAGS; i++){
		*flash_flags[i] = flash_flags_buffer[i];
	}
	for (int i = 0; i < 3; i++){
		*flash_time[i] = flash_time_buffer[i];
	}
}
