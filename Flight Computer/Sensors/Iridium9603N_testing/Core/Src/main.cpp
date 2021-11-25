/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <IridiumSBD.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * Bug concerning HAL_transmit:invalid conversion from 'const char*' to 'uint8_t*' {aka 'unsigned char*'} [-fpermissive]
 * https://community.st.com/s/question/0D53W000005qNDr/signature-issue-in-hal-function-haluarttransmit-how-to-file-a-bugchange-request
 */
#define DIAGNOSTICS false

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

IridiumSBD GPS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

static uint8_t setup(IridiumSBD I);
static void iridiumErrorMessage(uint8_t error);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  setup(GPS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

static uint8_t setup(IridiumSBD I){
	HAL_UART_Transmit(&huart3,(uint8_t*) "\r\nChecking for the device...", 30, HAL_MAX_DELAY);
	while(!I.isConnected()){
		HAL_UART_Transmit(&huart3,(uint8_t*) "Check if the device is connected. Trying again in\r\n", 53, HAL_MAX_DELAY);
		HAL_Delay(500);
		HAL_UART_Transmit(&huart3,(uint8_t*) "\r3", 3, HAL_MAX_DELAY);
		HAL_Delay(1000);
		HAL_UART_Transmit(&huart3,(uint8_t*) "\r2", 3, HAL_MAX_DELAY);
		HAL_Delay(1000);
		HAL_UART_Transmit(&huart3,(uint8_t*) "\r1\r\n", 7, HAL_MAX_DELAY);
		HAL_Delay(1000);
	}
	HAL_UART_Transmit(&huart3,(uint8_t*) "Done\r\n", 8, HAL_MAX_DELAY);

	/*
	//Check if device is connected (adress 0x63)
	HAL_UART_Transmit(&huart3, "Checking for the device...\n\r", 30, HAL_MAX_DELAY);
	ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(63<<1), 3, 5);
	while (ret != HAL_OK) {
		HAL_UART_Transmit(&huart3,(uint8_t*) "Check if the device is connected. Trying again in\n\r", 53, HAL_MAX_DELAY);
		HAL_Delay(500);
		HAL_UART_Transmit(&huart3,(uint8_t*) "3\r", 3, HAL_MAX_DELAY);
		HAL_Delay(1000);
		HAL_UART_Transmit(&huart3,(uint8_t*) "2\r", 3, HAL_MAX_DELAY);
		HAL_Delay(1000);
		HAL_UART_Transmit(&huart3,(uint8_t*) "1\r", 3, HAL_MAX_DELAY);
		HAL_Delay(1000);
	}
	HAL_UART_Transmit(&huart3,(uint8_t*) "The device was found!\n\r", 25, HAL_MAX_DELAY);
	*/

	//Activate the superchargers
	HAL_UART_Transmit(&huart3,(uint8_t*) "Activating the superchargers...", 31, HAL_MAX_DELAY);
	I.enableSuperCapCharger(true);
	HAL_UART_Transmit(&huart3,(uint8_t*) "Done\r\n", 8, HAL_MAX_DELAY);


	//Wait for the supercapacitors to charge
	while (!I.checkSuperCapCharger()){
		HAL_UART_Transmit(&huart3,(uint8_t*) "\rWaiting for the supercapacitors to charge.\r", 46, HAL_MAX_DELAY);
		HAL_Delay(333);
		HAL_UART_Transmit(&huart3,(uint8_t*) "Waiting for the supercapacitors to charge..\r", 45, HAL_MAX_DELAY);
		HAL_Delay(333);
		HAL_UART_Transmit(&huart3,(uint8_t*) "Waiting for the supercapacitors to charge...", 44, HAL_MAX_DELAY);
		HAL_Delay(333);
	}
	HAL_UART_Transmit(&huart3,(uint8_t*) "Done\r\n", 8, HAL_MAX_DELAY);


	//Enable power for the 9603N
	HAL_UART_Transmit(&huart3,(uint8_t*) "Enabling 9603N power...", 23, HAL_MAX_DELAY);
	I.enable9603Npower(true);
	HAL_UART_Transmit(&huart3,(uint8_t*) "Done\r\n", 8, HAL_MAX_DELAY);


	/*
	 * Begin satellite modem operation
	 */

	//Power on the rockblock
	HAL_UART_Transmit(&huart3,(uint8_t*) "Starting Modem...", 17, HAL_MAX_DELAY);
	int err = I.begin();
	if (err != ISBD_SUCCESS)
	  {
		HAL_UART_Transmit(&huart3,(uint8_t*) "Failed: ", 8, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart3,(uint8_t*) err, 4, HAL_MAX_DELAY);
		iridiumErrorMessage(err);

	    return HAL_ERROR;
	  }
	HAL_UART_Transmit(&huart3,(uint8_t*) "Done\r\n", 8, HAL_MAX_DELAY);

	return HAL_OK;
}

static void iridiumErrorMessage(uint8_t error){
	HAL_UART_Transmit(&huart3,(uint8_t*) "\r\nError:\t", 12, HAL_MAX_DELAY);
	if (error == ISBD_ALREADY_AWAKE){
	   	HAL_UART_Transmit(&huart3,(uint8_t*) "Already Awake\r\n", 17, HAL_MAX_DELAY);
	}
	else if (error == ISBD_SERIAL_FAILURE){
	   	HAL_UART_Transmit(&huart3,(uint8_t*) "Serial Failure\r\n", 18, HAL_MAX_DELAY);
	}
	else if (error == ISBD_PROTOCOL_ERROR){
		HAL_UART_Transmit(&huart3,(uint8_t*) "Protocol Error\r\n", 18, HAL_MAX_DELAY);
	}
	else if (error == ISBD_CANCELLED){
	   	HAL_UART_Transmit(&huart3,(uint8_t*) "\r\nCancelled", 13, HAL_MAX_DELAY);
	}
	else if (error == ISBD_NO_MODEM_DETECTED){
	   	HAL_UART_Transmit(&huart3,(uint8_t*) "\r\nNo modem detected: check wiring.", 36, HAL_MAX_DELAY);
	}
	else if (error == ISBD_SBDIX_FATAL_ERROR){
	   	HAL_UART_Transmit(&huart3,(uint8_t*) "SDBIX Fatal Error\r\n", 21, HAL_MAX_DELAY);
	}
	else if (error == ISBD_SENDRECEIVE_TIMEOUT){
	   	HAL_UART_Transmit(&huart3,(uint8_t*) "Send-Receive Timeout\r\n", 24, HAL_MAX_DELAY);
	}
	else if (error == ISBD_RX_OVERFLOW){
		HAL_UART_Transmit(&huart3,(uint8_t*) "RX Overflow\r\n", 15, HAL_MAX_DELAY);
	}
	else if (error == ISBD_REENTRANT){
		HAL_UART_Transmit(&huart3,(uint8_t*) "REENTRANT\r\n", 13, HAL_MAX_DELAY);
	}
	else if (error == ISBD_IS_ASLEEP){
	   	HAL_UART_Transmit(&huart3,(uint8_t*) "Is Asleep\r\n", 13, HAL_MAX_DELAY);
	}
	else if (error == ISBD_NO_SLEEP_PIN){
	   	HAL_UART_Transmit(&huart3,(uint8_t*) "No Sleep Pin\r\n", 16, HAL_MAX_DELAY);
	}
	else if(error == 20){
		HAL_UART_Transmit(&huart3,(uint8_t*) "DEBUG LINE REACHED\r\n", 22, HAL_MAX_DELAY);
	}
	else{
		HAL_UART_Transmit(&huart3,(uint8_t*) "UNKNOWN\r\n", 11, HAL_MAX_DELAY);
	}

/*
#define ISBD_SUCCESS             0
#define ISBD_ALREADY_AWAKE       1
#define ISBD_SERIAL_FAILURE      2
#define ISBD_PROTOCOL_ERROR      3
#define ISBD_CANCELLED           4
#define ISBD_NO_MODEM_DETECTED   5
#define ISBD_SBDIX_FATAL_ERROR   6
#define ISBD_SENDRECEIVE_TIMEOUT 7
#define ISBD_RX_OVERFLOW         8
#define ISBD_REENTRANT           9
#define ISBD_IS_ASLEEP           10
#define ISBD_NO_SLEEP_PIN        11
*/
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
