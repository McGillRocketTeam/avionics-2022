/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_i2c_sensors.h> //TODO REMOVE AND PUT IN PROPULSION FIRST LOOP IN PROPULSION.H
#include <MRT_iridium.h> //TODO REMOVE AND PUT IN PROPULSION FIRST LOOP IN PROPULSION.H

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void MRT_STM_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int main(void){
	MRT_STM_Init();
	println("\r\n\r\nSTM Init...OK");

	MRT_Init();

	println("\r\n/****Starting FC****/\r\n");

	//FOR TESTING

	#define TX_BUF_DIM 256
	char buffer[TX_BUF_DIM];


		while(1){
			HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, SET);
			HAL_Delay(1000);

			  //GPS
			  hgps.pollAll();

		  	  //LSM6DSR
		  	  hlsm6dsr.pollAll();

			  //LPS22HH
			  hlps22hh.pollAll();
			  //altitude_m = MRT_getAltitude(hlps22hh.pressure_hPa); //Update altitude TODO put somewhere else


			  //GPS
			  memset(buffer, 0, TX_BUF_DIM);
			  sprintf(buffer,"Alt: %.2f   Long: %.2f   Time: %.0f\r\n",hgps.latitude, hgps.longitude, hgps.time);
			  print(buffer);

			  //LSM6DSR
			  memset(buffer, 0, TX_BUF_DIM);
			  sprintf(buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
					  hlsm6dsr.acceleration_mg[0], hlsm6dsr.acceleration_mg[1], hlsm6dsr.acceleration_mg[2]);
			  print(buffer);

			  memset(buffer, 0, TX_BUF_DIM);
			  sprintf(buffer,"Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
					  hlsm6dsr.angular_rate_mdps[0], hlsm6dsr.angular_rate_mdps[1], hlsm6dsr.angular_rate_mdps[2]);
			  print(buffer);

			  memset(buffer, 0, TX_BUF_DIM);
			  sprintf(buffer, "Temperature [degC]:%6.2f\r\n", hlsm6dsr.temperature_degC);
			  print(buffer);


			  //LPS22HH
			  memset(buffer, 0, TX_BUF_DIM);
			  sprintf(buffer,"Pressure [hPa]:%6.2f\r\n",hlps22hh.pressure_hPa);
			  print(buffer);

			  memset(buffer, 0, TX_BUF_DIM);
			  sprintf(buffer, "Temperature [degC]:%6.2f\r\n", hlps22hh.temperature_degC);
			  print(buffer);


			  //Iridium
			  hiridium.getTime();

			HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, RESET);
			HAL_Delay(1000);
			HAL_IWDG_Refresh(&hiwdg);
		}


	return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void MRT_STM_Init(void){
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_I2C2_Init();
	MX_I2C3_Init();
	MX_SPI2_Init();
	MX_SPI4_Init();
	MX_SPI5_Init();
	MX_TIM2_Init();
	MX_UART8_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_RTC_Init();
	//MX_IWDG_Init(); TODO ADDED IN MRT_Init()
	MX_FATFS_Init();
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/