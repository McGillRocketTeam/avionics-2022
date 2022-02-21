/*
 * MRT_Helpers.c
 *
 *  Created on: Jan 7, 2022
 *      Author: Jacoby
 */
#include <stm32f4xx_hal.h>
#include <main.h>
#include <MRT_Helpers.h>

extern uint8_t reset_flag = 0;
extern uint8_t wakeup_flag = 0;

//TODO VirtAddVarTab NEED TO BE DEFINED (can go and change NB_OF_VAR as needed)
/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};


/*
 * User functions
 */
void MRT_getFlags(void){

	HAL_FLASH_Unlock();


	//Reset flag
	if((EE_ReadVariable(RESET_FLAG_ADDRESS,  &reset_flag)) != HAL_OK)
	{
	  //Error_Handler(); Variable doesn't exist so try writing it?
		if((EE_WriteVariable(RESET_FLAG_ADDRESS, reset_flag)) != HAL_OK)
		{
		  Error_Handler();
		}
	}
	if (reset_flag != 0 && reset_flag !=1){ //If random value (none was written)
		reset_flag = 0;
		if((EE_WriteVariable(RESET_FLAG_ADDRESS, reset_flag)) != HAL_OK)
		{
		  Error_Handler();
		}
	}


	//Wakeup flag
	if((EE_ReadVariable(WAKEUP_FLAG_ADDRESS,  &wakeup_flag)) != HAL_OK)
	{
	   //Error_Handler(); Variable doesn't exist so try writing it?
		 if((EE_WriteVariable(WAKEUP_FLAG_ADDRESS,  wakeup_flag)) != HAL_OK)
		 {
		   Error_Handler();
		 }
	}
	if (wakeup_flag != 0 && wakeup_flag !=1){ //If random value (none was written)
		wakeup_flag = 0;
		if((EE_WriteVariable(WAKEUP_FLAG_ADDRESS,  wakeup_flag)) != HAL_OK)
		{
		  Error_Handler();
		}
	}

	HAL_FLASH_Lock();

}


void MRT_resetInfo(UART_HandleTypeDef* uart){


	  char buffer[50];
	  sprintf(buffer,"Reset: %i,  WU: %i\r\n",reset_flag, wakeup_flag);
	  HAL_UART_Transmit(uart, buffer, strlen(buffer), HAL_MAX_DELAY);
	  HAL_Delay(1000);


	  //Check if we start from the beginning
	  if (reset_flag==0){
		  HAL_UART_Transmit(uart, "FC restarted\r\n", 14, HAL_MAX_DELAY);

		  reset_flag = 1; //Flip flag

		  //Write new flag to flash memory
		  HAL_FLASH_Unlock();
		  if((EE_WriteVariable(RESET_FLAG_ADDRESS, reset_flag)) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  HAL_FLASH_Lock();
	  }
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

		MRT_ResetFromStart();
	}

}


void MRT_ResetFromStart(void){

	//Clear all saved data of stages


	//Clear wakeup and reset flag

	HAL_FLASH_Unlock();

	reset_flag = 0;
	if((EE_WriteVariable(RESET_FLAG_ADDRESS, reset_flag)) != HAL_OK)
	{
	  Error_Handler();
	}

	wakeup_flag = 0;
	if((EE_WriteVariable(WAKEUP_FLAG_ADDRESS, wakeup_flag)) != HAL_OK)
	{
	  Error_Handler();
	}
	HAL_FLASH_Lock();


	//Reset function
	NVIC_SystemReset();
}
