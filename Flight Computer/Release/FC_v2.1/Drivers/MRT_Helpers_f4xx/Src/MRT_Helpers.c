/*
 * MRT_Helpers.c
 *
 *  Created on: Jan 7, 2022
 *      Author: Jacoby
 */
#include <stm32f4xx_hal.h>
#include <MRT_RTOS.h>
#include <IridiumSBD_Static_API.h>

uint8_t reset_flag = 0;
uint8_t iwdg_flag = 0;

//Offset of flags in external flash sector 1 (can go and change NB_OF_FLAGS as needed)
uint16_t VirtAddVarTab[NB_OF_FLAGS];

//Flags read/write buffer
uint8_t flash_flags_buffer[NB_OF_FLAGS];

//TODO Reference array to each flag
uint8_t* flash_flags[NB_OF_FLAGS] = {&reset_flag, &wu_flag, &iwdg_flag};

//Null buffer values for when clearing flags
uint8_t NULL_BUFFER[NB_OF_FLAGS];

/*
 * User functions
 */
void MRT_externalFlashSetup(UART_HandleTypeDef* uart){

	for (int i = 0; i < NB_OF_FLAGS; i++){
		NULL_BUFFER[i] = 0; //Setup the null buffer for the correct number of values
	}

	if (!W25qxx_Init()) {
		Error_Handler(); // hangs and blinks LEDF
	}
	MRT_getFlags();
	MRT_resetInfo(uart);
}

/*
void MRT_freezeWatchDog(void){
	 FLASH_OBProgramInitTypeDef pOBInit;

	 HAL_FLASH_Unlock();
	 //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); // Clear the FLASH's pending flags.
	 __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR); // Clear the FLASH's pending flags.
	 HAL_FLASH_OB_Unlock();

	 HAL_FLASHEx_OBGetConfig(&pOBInit); // Get the Option bytes configuration.

	 pOBInit.OptionType = OPTIONBYTE_USER;
	 //pOBInit.USERType = OB_USER_IWDG_STOP;
	 pOBInit.USERConfig = OB_IWDG_STOP_FREEZE;
	 pOBInit.USERConfig = IWDG_STOP_FREEZE;
	 HAL_FLASHEx_OBProgram(&pOBInit);

	 HAL_FLASH_OB_Lock();
	 HAL_FLASH_Lock();
}
*/



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
void tone(uint32_t duration, uint32_t repeats, TIM_HandleTypeDef htim)
{
	for (uint32_t i = 0; i < repeats; i++)
	{
		HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3);
		HAL_Delay(duration); // wait so i can probe voltage
		HAL_TIM_PWM_Stop(&htim, TIM_CHANNEL_3);
		HAL_Delay(duration);
	}
}
*/





/*
 * Helper functions
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == IN_Button_Pin){
		//Manual reset
		MRT_resetFromStart();
	}

}


void MRT_resetFromStart(void){
	//Clear all saved data of stages
	//TODO

	//Clear wakeup and reset flags
	W25qxx_EraseSector(1);
	W25qxx_WriteSector(NULL_BUFFER, 1, FLAGS_OFFSET, NB_OF_FLAGS);

	//Shutdown Iridium
	MRT_Static_Iridium_Shutdown();

	//Reset function
	NVIC_SystemReset();
}


void MRT_updateExternalFlashBuffer(void){
	for (int i = 0; i < NB_OF_FLAGS; i++){
		flash_flags_buffer[i] = *flash_flags[i];
	}
}


void MRT_getFlags(void){

	//Retrieve flags
	W25qxx_ReadSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);

	//If RTC detected a wake up, update the flash memory
	if (wu_flag == 1){
		flash_flags_buffer[WU_FLAG_OFFSET] = wu_flag;
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}

	//Assign each value read to their variable
	for (int i = 0; i < NB_OF_FLAGS; i++){
		*flash_flags[i] = flash_flags_buffer[i];
	}


	//Check flags values

	//Reset flag
	if (reset_flag != 0 && reset_flag !=1){ //If random value (none was written)
		reset_flag = 0;
		flash_flags_buffer[RESET_FLAG_OFFSET] = reset_flag;
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);
	}

	//Wake up flag
	if (wu_flag != 0 && wu_flag !=1){ //If random value (none was written)
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
}


void MRT_resetInfo(UART_HandleTypeDef* uart){

	  char buffer[50];
	  sprintf(buffer,"Reset: %i,  WU: %i,  IWDG: %i\r\n",reset_flag, wu_flag, iwdg_flag);
	  HAL_UART_Transmit(uart, buffer, strlen(buffer), HAL_MAX_DELAY);

	  //Check if IWDG is being deactivated
	  if (iwdg_flag==1){
		  HAL_UART_Transmit(uart, "Deactivating IWDG\r\n", 19, HAL_MAX_DELAY);

		  iwdg_flag = 0; //Flip flag

		  //Write new flag to flash memory
		  flash_flags_buffer[IWDG_FLAG_OFFSET] = iwdg_flag;
		  W25qxx_EraseSector(1);
		  W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);

		  //Disable alarm A only
		  MRT_setAlarmA(0,0,0);
		  MRT_ClearFlags();

		  HAL_Delay(1000);


		  //MRT_Static_Iridium_Shutdown(); TODO

		  //Go to sleep
		  MRT_StandByMode(SLEEP_TIME);
	  }


	  //Check if we are after waking up
	  if (wu_flag==1){
		  HAL_UART_Transmit(uart, "FC wake up\r\n", 12, HAL_MAX_DELAY);

		  //Deactivate alarm interrupts
		  HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
		  __HAL_RTC_ALARM_EXTI_DISABLE_IT();
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
}


uint8_t MRT_getContinuity(void){
	uint8_t drogue1 = HAL_GPIO_ReadPin(IN_EJ_Drogue_Cont_GPIO_Port, IN_EJ_Drogue_Cont_Pin);
	uint8_t drogue2 = HAL_GPIO_ReadPin(IN_EJ_Main_Cont_GPIO_Port, IN_EJ_Main_Cont_Pin);
	uint8_t prop1 = HAL_GPIO_ReadPin(IN_PyroValve_Cont_1_GPIO_Port, IN_PyroValve_Cont_1_Pin);
	uint8_t prop2 = HAL_GPIO_ReadPin(IN_PyroValve_Cont_2_GPIO_Port, IN_PyroValve_Cont_2_Pin);
	uint8_t continuity = 8*drogue1 + 4*drogue2 + 2*prop1 + prop2;
	return continuity;
}



