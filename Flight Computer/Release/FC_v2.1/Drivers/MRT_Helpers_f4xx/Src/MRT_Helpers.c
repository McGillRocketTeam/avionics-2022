/*
 * MRT_Helpers.c
 *
 *  Created on: Jan 7, 2022
 *      Author: Jacoby
 */
#include <stm32f4xx_hal.h>


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
