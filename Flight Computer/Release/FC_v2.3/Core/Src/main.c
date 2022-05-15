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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <MRT_setup.h>
#include <MRT_helpers.h>

//For the wait for launch
#include <MRT_memory.h>
#include <MRT_ejection.h>
#include <MRT_propulsion.h>
#include <MRT_telemetry.h>

//TODO for testing
#include <MRT_i2c_sensors.h>

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
void MRT_waitForLaunch(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int main(void){
	MRT_STM_Init();
	__HAL_DBGMCU_FREEZE_IWDG();	// pause IWDG and RTC during debugging
	__HAL_FREEZE_RTC_DBGMCU();
	println("\r\n\r\nSTM Init...OK");

	MRT_Init();

	println("\r\n\r\n/****Starting FC****/\r\n\r\n");
	HAL_IWDG_Refresh(&hiwdg);
	buzz_startup_success();

	MRT_waitForLaunch();

	//TODO I2C SENSORS (lsm and lps) SOMETIMES DON'T WANT TO WORK ANYMORE -> NEED TO RESET THE POWER (Enter quick standByMode?)
	//check hardfault_handler

	//Initialize the os
	MX_FREERTOS_Init();

	//Starting the os
	println("\r\n/****Starting the OS****/\r\n");
	osKernelStart();

	println("SOMETHING WENT HORRIBLY WRONG, WAITING FOR WATCH DOG RESET");
	HAL_IWDG_Refresh(&hiwdg);
	MRT_Deinit();
	while (1){}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

void MRT_waitForLaunch(void){

	println("Waiting for launch command from ground station\r\n");

	char radio_buffer[RADIO_BUFFER_SIZE];
	radio_command cmd = -1;

	//Poll propulsion until launch command sent
	while((XTEND_ || SRADIO_) && ejection_stage_flag == PAD){
		HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, SET);

		HAL_IWDG_Refresh(&hiwdg);

		//Get RTC time
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		//Update global variables
		prev_hour = sTime.Hours;
		prev_min = sTime.Minutes;
		prev_sec = sTime.Seconds;
		if (__HAL_RTC_SHIFT_GET_FLAG(&hrtc, RTC_FLAG_SHPF)) prev_sec++; //Adjust following the user manual
		prev_subsec = sTime.SubSeconds;

		//Save the RTC time
	    MRT_saveTotalTime();

		//Poll propulsion sensors
		MRT_pollPropulsion();

		//Send propulsion data
		memset(radio_buffer, 0, RADIO_BUFFER_SIZE);
		sprintf(radio_buffer,"P,%.2f,%.2f, %i,E\r\n",transducer_voltage,thermocouple_temperature,(int) valve_status);
		MRT_radio_tx(radio_buffer);


		// Save to SD card
		#if SD_CARD_
		fres = sd_open_file(filename);
		MRT_formatPropulsion();
		sd_write(&fil, msg_buffer_pr);
		f_close(&fil);
		#endif


		//Check for launch command
		memset(radio_buffer, 0, RADIO_BUFFER_SIZE);
		MRT_radio_rx(radio_buffer, 2, 0x500); //Timeout is about 1.2 sec (should be less than 5 sec)
		cmd = radio_parse_command(radio_buffer);

		if (cmd == LAUNCH){
			//Update ejection stage flag and save it
			ejection_stage_flag = BOOST;
			rtc_bckp_reg_ejection_stage = BOOST;
			ext_flash_ejection_stage = BOOST;
			MRT_saveFlagValue(FC_STATE_FLIGHT);
		}
		execute_parsed_command(cmd);

		HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);

		//Reset IWDG timer
		HAL_IWDG_Refresh(&hiwdg);

		HAL_Delay(1000/PRE_APOGEE_SEND_FREQ);
	}


	//Todo to test ejection
	hlps22hh.getPressure();
	rtc_bckp_reg_alt_pad = MRT_getAltitude(hlps22hh.pressure_hPa);
	MRT_RTC_setBackupReg(FC_STATE_ALT_PAD, rtc_bckp_reg_alt_pad);
	rtc_bckp_reg_pad_time = 100*prev_min + prev_sec;
	MRT_RTC_setBackupReg(FC_PAD_TIME, rtc_bckp_reg_pad_time);

	//Send acknowledgement
	MRT_radio_tx((char*) "LAUNCH COMMAND RECEIVED"); //TODO CHECK AT WHAT JASPER DID FOR ACK MESSAGES
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
