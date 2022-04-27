/*
 * MRT_Helpers.c
 *
 *  Created on: Jan 7, 2022
 *      Author: Jacoby
 */
#include <stm32f4xx_hal.h>
#include <MRT_RTOS.h>
#include <math.h>
//#include <IridiumSBD_Static_API.h> TODO why should we include this??

/*Flags*/
uint8_t reset_flag = 0; //In external flash memory
//wu_flag defined in MRT_RTOS.c
uint8_t iwdg_flag = 0; //In external flash memory
uint8_t apogee_flag = 0; //In external flash memory
uint8_t ejection_state_flag = 0; //In external flash memory

//Flags read/write buffer
uint8_t flash_flags_buffer[NB_OF_FLAGS];

//Reference array to each flag
uint8_t* flash_flags[NB_OF_FLAGS] = {&reset_flag, &wu_flag, &iwdg_flag, &apogee_flag, &ejection_state_flag};

/*
//Offset of flags in external flash sector 1 (can go and change NB_OF_FLAGS as needed)
uint16_t VirtAddVarTab[NB_OF_FLAGS];
*/

//Null buffer values for when clearing flags
uint8_t FLAGS_NULL_BUFFER[NB_OF_FLAGS];



/*RTC time*/
//Time constants (determined at each reset)
uint8_t prev_hours = 0; //Last recorded hours
uint8_t prev_min = 0; //Last recorded minutes
uint8_t prev_sec = 0; //Last recorded seconds
uint8_t prev_subsec = 0; //Last recorded subseconds

//Time read/write buffer
uint8_t flash_time_buffer[RTC_NB_OF_VAR];

//Reference list to each time component
uint8_t* flash_time[RTC_NB_OF_VAR] = {&prev_hours, &prev_min, &prev_sec, &prev_subsec};

//Null buffer values for when clearing time
uint8_t RTC_TIME_NULL_BUFFER[RTC_NB_OF_VAR] = {0,0,0,0};





/*
 * User functions
 */
void MRT_externalFlashSetup(UART_HandleTypeDef* uart){

	for (int i = 0; i < NB_OF_FLAGS; i++){
		FLAGS_NULL_BUFFER[i] = 0; //Setup the flags null buffer for the correct number of values
	}

	if (!W25qxx_Init()) {
		Error_Handler(); // hangs and blinks LEDF
	}
	MRT_WUProcedure(); //Needs to be called before getFlags() and after the W25xx_Init()
	MRT_getFlags();
	MRT_resetInfo(uart);
}



void checkForI2CDevices(UART_HandleTypeDef uart, I2C_HandleTypeDef I2C ){
	uint8_t Buffer[25] = {0};
	uint8_t Space[] = " - ";
	uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
	uint8_t EndMSG[] = "Done! \r\n\r\n";

    uint8_t i = 0, ret;

    HAL_Delay(1000);

    /*-[ I2C Bus Scanning ]-*/
    HAL_UART_Transmit(&uart, StartMSG, sizeof(StartMSG), HAL_MAX_DELAY);
    for(i=1; i<128; i++)
    {
        ret = HAL_I2C_IsDeviceReady(&I2C, (uint16_t)(i<<1), 3, 5);
        if (ret != HAL_OK) /* No ACK Received At That Address */
        {
            HAL_UART_Transmit(&uart, Space, sizeof(Space), HAL_MAX_DELAY);
        }
        else if(ret == HAL_OK)
        {
            sprintf(Buffer, "0x%X", i);
            HAL_UART_Transmit(&uart, Buffer, sizeof(Buffer), HAL_MAX_DELAY);
        }
    }
    HAL_UART_Transmit(&uart, EndMSG, sizeof(EndMSG), HAL_MAX_DELAY);
    /*--[ Scanning Done ]--*/

}


/*
 * Helper functions
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == IN_Button_Pin){
		//Manual reset from external button
		MRT_resetFromStart();
	}

}


void MRT_resetFromStart(void){
	//Clear flags
	W25qxx_EraseSector(1);
	W25qxx_WriteSector(FLAGS_NULL_BUFFER, 1, FLAGS_OFFSET, NB_OF_FLAGS);

	//Clear RTC time (last recorded)
	W25qxx_EraseSector(2);
	W25qxx_WriteSector(RTC_TIME_NULL_BUFFER, 2, RTC_TIME_OFFSET, RTC_NB_OF_VAR);

	//Shutdown Iridium
	MRT_Static_Iridium_Shutdown();

	//Reset function
	NVIC_SystemReset();
}


void MRT_updateExternalFlashBuffers(void){
	for (int i = 0; i < NB_OF_FLAGS; i++){
		flash_flags_buffer[i] = *flash_flags[i];
	}
	for (int i = 0; i < RTC_NB_OF_VAR; i++){
		flash_time_buffer[i] = *flash_time[i];
	}
}


void MRT_updateFlagsValues(void){
	for (int i = 0; i < NB_OF_FLAGS; i++){
		*flash_flags[i] = flash_flags_buffer[i];
	}
	for (int i = 0; i < RTC_NB_OF_VAR; i++){
		*flash_time[i] = flash_time_buffer[i];
	}
}


void MRT_getFlags(void){

	//Retrieve flags
	W25qxx_ReadSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);

	//Retrieve RTC time (last recorded)
	W25qxx_ReadSector(flash_time_buffer, 2, RTC_TIME_OFFSET, RTC_NB_OF_VAR);

	//If RTC detected a wake up, update the flash memory
	if (wu_flag == 1){
		//Write the new number of wake up to external flash
		flash_flags_buffer[WU_FLAG_OFFSET] = flash_flags_buffer[WU_FLAG_OFFSET] + 1; //Update number of wake up
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}

	//Assign each value read to their variable
	MRT_updateFlagsValues();


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
		W25qxx_WriteSector(flash_time_buffer, 2, RTC_TIME_OFFSET, RTC_NB_OF_VAR);
	}

	//Minutes
	if (!(prev_min >= 0 && prev_min < 60)){ //If random value (none was written)
		prev_min = 0;
		flash_time_buffer[RTC_MIN_OFFSET] = prev_min;
		W25qxx_EraseSector(2);
		W25qxx_WriteSector(flash_time_buffer, 2, RTC_TIME_OFFSET, RTC_NB_OF_VAR);
	}

	//Seconds
	if (!(prev_sec >= 0 && prev_sec < 60)){ //If random value (none was written)
		prev_sec = 0;
		flash_time_buffer[RTC_SEC_OFFSET] = prev_sec;
		W25qxx_EraseSector(2);
		W25qxx_WriteSector(flash_time_buffer, 2, RTC_TIME_OFFSET, RTC_NB_OF_VAR);
	}

	//Sub-Seconds
	if (!(prev_subsec >= 0 && prev_subsec < 100)){ //If random value (none was written)
		prev_subsec = 0;
		flash_time_buffer[RTC_SUBSEC_OFFSET] = prev_subsec;
		W25qxx_EraseSector(2);
		W25qxx_WriteSector(flash_time_buffer, 2, RTC_TIME_OFFSET, RTC_NB_OF_VAR);
	}
}


void MRT_resetInfo(UART_HandleTypeDef* uart){

	  char buffer[100];
	  sprintf(buffer,"Reset: %i,  WU: %i,  IWDG: %i\r\nPrevious RTC time: %i:%i:%i ::%i\r\n",reset_flag, wu_flag, iwdg_flag, prev_hours, prev_min, prev_sec, prev_subsec);
	  HAL_UART_Transmit(uart, buffer, strlen(buffer), HAL_MAX_DELAY);

	  //Check if IWDG is being deactivated
	  if (iwdg_flag==1){
		  HAL_UART_Transmit(uart, "Deactivating IWDG\r\n", 19, HAL_MAX_DELAY);

		  iwdg_flag = 0; //Flip flag

		  //Write new flag to flash memory
		  flash_flags_buffer[IWDG_FLAG_OFFSET] = iwdg_flag;
		  W25qxx_EraseSector(1);
		  W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);

		  HAL_Delay(1000);

		  //Go to sleep
		  MRT_StandByMode(SLEEP_TIME);
	  }


	  //Check if we are after waking up (and at which wake up we are at)
	  if (wu_flag>0){
		  char buf[20];
		  sprintf(buf, "FC wake up %i\r\n", wu_flag);
		  HAL_UART_Transmit(uart, buf, strlen(buf), HAL_MAX_DELAY);

		  HAL_UART_Transmit(uart, "Resetting RTC time\r\n", 20, HAL_MAX_DELAY);

		  //Clear RTC time (last recorded)
		  W25qxx_EraseSector(2);
		  W25qxx_WriteSector(RTC_TIME_NULL_BUFFER, 2, RTC_TIME_OFFSET, 3);

		  //Update variables (to 0)
		  for (int i = 0; i < 3; i++){
			  *flash_time[i] = 0x0;
		  }

	  }


	  //Check if we start from the beginning
	  if (reset_flag==0){
		  HAL_UART_Transmit(uart, "FC restarted\r\n", 14, HAL_MAX_DELAY);

		  reset_flag = 1; //Flip flag

		  //Write new flag to flash memory
	      flash_flags_buffer[RESET_FLAG_OFFSET] = reset_flag;
		  W25qxx_EraseSector(1);
		  W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	  }


	  //Check if before or after apogee
	  if (apogee_flag==0){
		  HAL_UART_Transmit(uart, "Pre-apogee\r\n", 12, HAL_MAX_DELAY);
	  }
	  else if(apogee_flag==1){
		  HAL_UART_Transmit(uart, "Post-apogee\r\n", 13, HAL_MAX_DELAY);
	  }


	  //Check ejection state
	  if (ejection_state_flag==0){
		  HAL_UART_Transmit(uart, "Ejection State: Pad\r\n", 21, HAL_MAX_DELAY);
	  }
	  else if(ejection_state_flag==1){
		  HAL_UART_Transmit(uart, "Ejection State: Boost\r\n", 23, HAL_MAX_DELAY);
	  }
	  else if(ejection_state_flag==2){
		  HAL_UART_Transmit(uart, "Ejection State: Drogue descent\r\n", 32, HAL_MAX_DELAY);
	  }
	  else if(ejection_state_flag==3){
		  HAL_UART_Transmit(uart, "Ejection State: Main descent\r\n", 30, HAL_MAX_DELAY);
	  }
	  else if(ejection_state_flag==4){
		  HAL_UART_Transmit(uart, "Ejection State: Landed\r\n", 24, HAL_MAX_DELAY);
	  }
}


/*
 * Update and save the RTC time in external flash memory
 */
void MRT_saveRTCTime(void){
	MRT_updateExternalFlashBuffers();

	//Write new RTC time to flash memory
	W25qxx_EraseSector(2);
	W25qxx_WriteSector(flash_time_buffer, 2, RTC_TIME_OFFSET, RTC_NB_OF_VAR);
}


/*
 * Checks the continuity of the gates
 *
 * returns a binary number in its decimal form. Each bit is the state of a gate.
 * bit3 bit2 bit1 bit0 = drogue1 drogue2 prop1 prop2
 */
uint8_t MRT_getContinuity(void){
	uint8_t drogue1 = HAL_GPIO_ReadPin(IN_EJ_Drogue_Cont_GPIO_Port, IN_EJ_Drogue_Cont_Pin);
	uint8_t drogue2 = HAL_GPIO_ReadPin(IN_EJ_Main_Cont_GPIO_Port, IN_EJ_Main_Cont_Pin);
	uint8_t prop1 = HAL_GPIO_ReadPin(IN_PyroValve_Cont_1_GPIO_Port, IN_PyroValve_Cont_1_Pin);
	uint8_t prop2 = HAL_GPIO_ReadPin(IN_PyroValve_Cont_2_GPIO_Port, IN_PyroValve_Cont_2_Pin);
	uint8_t continuity = 8*drogue1 + 4*drogue2 + 2*prop1 + prop2;
	return continuity;
}


/*
 * Get the pressure transducer voltage (poll ADC)
 */
float MRT_prop_poll_pressure_transducer(ADC_HandleTypeDef* hadc) {
	// reading adc
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	uint32_t pressure_sensor_raw = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);

	float voltage = (float) (pressure_sensor_raw / 4095.0) * 3.3; // assuming 12 bits

	return voltage;
}


/*
 * Gets the altitude using temperature, pressure and sea-level pressure
 *https://www.mide.com/air-pressure-at-altitude-calculator
 */
float MRT_getAltitude(float pressure){
	return BASE_HEIGHT+(SEA_LEVEL_TEMPERATURE/-0.0065)*(pow(pressure/SEA_LEVEL_PRESSURE,0.190263236)-1); //(-R*-0.0065/(go*M)) = 0.190263236
}



extern TIM_HandleTypeDef htim2;

// buzz at particular frequency
void tone_freq(uint32_t duration, uint32_t repeats, uint32_t freq) {
	// TIM2 base frequency is 90 MHz, PSC = 90-1
	// can calculate required ARR value
	TIM2->ARR = 1000000 / freq;
	TIM2->EGR |= TIM_EGR_UG;

	for (uint32_t i = 0; i < repeats; i++) {
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin, SET);
		HAL_Delay(duration);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin, RESET);
		if (repeats > 1)
			HAL_Delay(duration);
	}
}

void buzz_success(void) { tone_freq(BUZZ_SUCCESS_DURATION, BUZZ_SUCCESS_REPEATS, BUZZ_SUCCESS_FREQ); };
void buzz_failure(void) { tone_freq(BUZZ_FAILURE_DURATION, BUZZ_FAILURE_REPEATS, BUZZ_FAILURE_FREQ); };

void buzz_startup_success(void) {
	for (uint8_t i = 0; i < 3; i++) {
		buzz_success();
		HAL_Delay(1000);
	}
};
