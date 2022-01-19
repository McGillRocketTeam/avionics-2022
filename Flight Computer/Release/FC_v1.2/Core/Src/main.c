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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <MRT_RTOS.h>
#include <IridiumSBD_Static_API.h>
#include <MRT_Helpers.h>
#include "ism330dlc_reg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DIAGNOSTICS false

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;

/* Definitions for Ejection1 */
osThreadId_t Ejection1Handle;
const osThreadAttr_t Ejection1_attributes = {
  .name = "Ejection1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Telemetry2 */
osThreadId_t Telemetry2Handle;
const osThreadAttr_t Telemetry2_attributes = {
  .name = "Telemetry2",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Sensors3 */
osThreadId_t Sensors3Handle;
const osThreadAttr_t Sensors3_attributes = {
  .name = "Sensors3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Propulsion4 */
osThreadId_t Propulsion4Handle;
const osThreadAttr_t Propulsion4_attributes = {
  .name = "Propulsion4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Memory0 */
osThreadId_t Memory0Handle;
const osThreadAttr_t Memory0_attributes = {
  .name = "Memory0",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */

//General
osThreadId_t threadID[5]; //TODO
bool wakingUp;

//Ejection
uint8_t MIN_APOGEE = 20;
uint8_t MAX_APOGEE = 30;
uint8_t DEPLOY_ALT_MIN = 60;
uint8_t DEPLOY_ALT_MAX = 65;
uint8_t GROUND_LEVEL = 20;

//Telemetry
uint8_t SLEEP_TIME = 10;
uint8_t DATA_FREQ = 1; //Times per second that you want to save and transmit data

//MUTEXES
uint8_t sensorsPolling;
uint8_t dataWriting;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
void StartEjection1(void *argument);
void StartTelemetry2(void *argument);
void StartSensors3(void *argument);
void StartPropulsion4(void *argument);
void StartMemory0(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


  checkForI2CDevices(huart3,hi2c1);

  //TODO
  /*
   * In general:
   *-Activate I2C1 and use pins PB8 and PB9
   *-Activate usart3 and use pins PD9 and PD8 and set the baud rate
   *-Activate freeRTOS
   *-Change SysTic to any other timer
   *-Include the path to all includes folders of the drivers (for C and C++ linkers)
   */

  /*
   * For Iridium:
   * -Set the project as c++
   */
   wakingUp = MRT_Static_Iridium_Setup(huart3);

  /*
   * For ISM330DLC
   *-Enable float formatting for sprintf (go to Project->Properties->C/C++ Build->Settings->MCU Settings->Check the box "Use float with printf")
   */
   MRT_ISM330DLC_Setup(&dev_ctx,&hi2c1,&huart3);

  /*
   * For RTOS
   * -Activate RTC, calendar and internal alarm A (don't forget to activate NVIC EXTI)
   * -Define what you want in the alarms callback functions (check the .h file)
   * -(Optional) Setup alarm A and the clock time in .ioc
   * The rest have been taken care of
   * You can access the flag of both alarm A and B with the variables flagA and flagB
   */
  MRT_SetupRTOS(huart3,10);

  HAL_UART_Transmit(&huart3,"\r\n\r\nStarting FC\r\n\r\n",19,HAL_MAX_DELAY);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Ejection1 */
  Ejection1Handle = osThreadNew(StartEjection1, NULL, &Ejection1_attributes);

  /* creation of Telemetry2 */
  Telemetry2Handle = osThreadNew(StartTelemetry2, NULL, &Telemetry2_attributes);

  /* creation of Sensors3 */
  Sensors3Handle = osThreadNew(StartSensors3, NULL, &Sensors3_attributes);

  /* creation of Propulsion4 */
  Propulsion4Handle = osThreadNew(StartPropulsion4, NULL, &Propulsion4_attributes);

  /* creation of Memory0 */
  Memory0Handle = osThreadNew(StartMemory0, NULL, &Memory0_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x30;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUILTIN_GPIO_Port, BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUILTIN_Pin */
  GPIO_InitStruct.Pin = BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUILTIN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartEjection1 */
/**
  * @brief  Function implementing the Ejection1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEjection1 */
void StartEjection1(void *argument)
{
  /* USER CODE BEGIN 5 */

	//Add thread id to the list
	threadID[1]=osThreadGetId();

	if (wakingUp) osThreadExit();

	uint8_t pressure;
	uint8_t altitude;
	char* buffer = (char*) pvPortMalloc(TX_BUF_DIM);

  /* Infinite loop */
  for(;;)
  {
	  sensorsPolling = true;
	  //Poll altitude (poll pressure)

	  //https://stackoverflow.com/questions/38782389/freertos-stm32-thread-memory-overflow-with-malloc
	  //https://nadler.com/embedded/newlibAndFreeRTOS.html


	  memset(buffer, 0, TX_BUF_DIM);
	  MRT_ISM330DLC_getTemperature(data_raw_temperature,temperature_degC);
	  sprintf((char *)buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC[0] );sprintf((char *)buffer, "Temperature [degC]:%f\r\n", temperature_degC[0] );
	  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);

	  pressure = ((uint8_t) temperature_degC[0]);

	  sensorsPolling = false;


	  if (MIN_APOGEE <= pressure && MAX_APOGEE > pressure){


		  //Eject drogue (PG11/PG12)
		  HAL_UART_Transmit(&huart3, "Eject Drogue\r\n", 15, HAL_MAX_DELAY);


		  for(;;){

			  memset(buffer, 0, TX_BUF_DIM);
			  MRT_ISM330DLC_getAcceleration(data_raw_acceleration,acceleration_mg);
			  sprintf((char *)buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
			  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);

			  memset(buffer, 0, TX_BUF_DIM);
			  MRT_ISM330DLC_getAngularRate(data_raw_angular_rate,angular_rate_mdps);
			  sprintf((char *)buffer,"Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
			  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);

			  altitude = acceleration_mg[0];

			  //We reached main deployment altitude
			  if (altitude>DEPLOY_ALT_MIN && altitude<DEPLOY_ALT_MAX){

				  vPortFree(buffer);

				  //Eject main ()
				  HAL_UART_Transmit(&huart3, "Eject Main\r\n", 13, HAL_MAX_DELAY);

				  for(;;){

					  sensorsPolling = true;
					  //Poll altitude
					  sensorsPolling = false;

					  if (altitude < GROUND_LEVEL)  osThreadExit();

					  osDelay(100);
				  }
			  }

			  osDelay(100);
		  }
	  }

    osDelay(100);
  }

  //In case it leaves the infinite loop
  HAL_UART_Transmit(&huart3,"Something went wrong with thread 1\r\n",36,HAL_MAX_DELAY);
  vPortFree(buffer);
  osThreadExit();

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTelemetry2 */
/**
* @brief Function implementing the Telemetry2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetry2 */
void StartTelemetry2(void *argument)
{
  /* USER CODE BEGIN StartTelemetry2 */

	//Add thread id to the list
	threadID[2]=osThreadGetId();


	//Make the thread joinable?


  /* Infinite loop */
  for(;;)
  {
	  //Poll sensor data in other thread
	  while(sensorsPolling){osDelay(1);}
		  dataWriting = true;

		  //Write data to sd and flash



		  //Send data via radios


		  //Radio send

		  //Iridium send
		  MRT_Static_Iridium_getTime();

		  //Check if it's sleep time
		  if (flagA==1){
			  MRT_Static_Iridium_Shutdown();

			  MRT_StandByMode(SLEEP_TIME);
		  }
		  dataWriting = false;

    osDelay(1000/DATA_FREQ);
  }

  //In case it leaves the infinite loop
  HAL_UART_Transmit(&huart3,"Something went wrong with thread 2\r\n",36,HAL_MAX_DELAY);
  osThreadExit();
  /* USER CODE END StartTelemetry2 */
}

/* USER CODE BEGIN Header_StartSensors3 */
/**
* @brief Function implementing the Sensors3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensors3 */
void StartSensors3(void *argument)
{
  /* USER CODE BEGIN StartSensors3 */

	osThreadExit();

	//Add thread id to the list
	threadID[3]=osThreadGetId();

  for(;;)
  {
    osDelay(1);
  }

  //In case it leaves the infinite loop
  HAL_UART_Transmit(&huart3,"Something went wrong with thread 3\r\n",36,HAL_MAX_DELAY);
  osThreadExit();
  /* USER CODE END StartSensors3 */
}

/* USER CODE BEGIN Header_StartPropulsion4 */
/**
* @brief Function implementing the Propulsion4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPropulsion4 */
void StartPropulsion4(void *argument)
{
  /* USER CODE BEGIN StartPropulsion4 */
	osThreadExit();

	//Add thread id to the list
	threadID[4]=osThreadGetId();

	if (wakingUp) osThreadExit();

  for(;;)
  {
	  sensorsPolling = true;
	  //Poll sensor data (burnout level)
	  sensorsPolling = false;

	  //Write to SD and SEND

    osDelay(1);
  }

  //In case it leaves the infinite loop
  HAL_UART_Transmit(&huart3,"Something went wrong with thread 4\r\n",36,HAL_MAX_DELAY);
  osThreadExit();
  /* USER CODE END StartPropulsion4 */
}

/* USER CODE BEGIN Header_StartMemory0 */
/**
* @brief Function implementing the Memory0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMemory0 */
void StartMemory0(void *argument)
{
  /* USER CODE BEGIN StartMemory0 */

	osThreadExit();

	//Add thread id to the list
	threadID[0]=osThreadGetId();

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }

  //In case it leaves the infinite loop
  HAL_UART_Transmit(&huart3,"Something went wrong with thread 0\r\n",36,HAL_MAX_DELAY);
  osThreadExit();
  /* USER CODE END StartMemory0 */
}

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
