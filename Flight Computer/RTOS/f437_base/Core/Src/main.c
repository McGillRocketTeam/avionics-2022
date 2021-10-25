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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for Sensor */
osThreadId_t SensorHandle;
uint32_t SensorBuffer[ 128 ];
osStaticThreadDef_t SensorControlBlock;
const osThreadAttr_t Sensor_attributes = {
  .name = "Sensor",
  .cb_mem = &SensorControlBlock,
  .cb_size = sizeof(SensorControlBlock),
  .stack_mem = &SensorBuffer[0],
  .stack_size = sizeof(SensorBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Telemetry */
osThreadId_t TelemetryHandle;
uint32_t TelemetryBuffer[ 128 ];
osStaticThreadDef_t TelemetryControlBlock;
const osThreadAttr_t Telemetry_attributes = {
  .name = "Telemetry",
  .cb_mem = &TelemetryControlBlock,
  .cb_size = sizeof(TelemetryControlBlock),
  .stack_mem = &TelemetryBuffer[0],
  .stack_size = sizeof(TelemetryBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Ejection */
osThreadId_t EjectionHandle;
uint32_t EjectionBuffer[ 128 ];
osStaticThreadDef_t EjectionControlBlock;
const osThreadAttr_t Ejection_attributes = {
  .name = "Ejection",
  .cb_mem = &EjectionControlBlock,
  .cb_size = sizeof(EjectionControlBlock),
  .stack_mem = &EjectionBuffer[0],
  .stack_size = sizeof(EjectionBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */
// rtc variables
//RTC_DateTypeDef sdatestructureget;
//RTC_TimeTypeDef stimestructureget;
//RTC_AlarmTypeDef sAlarmA;
//RTC_AlarmTypeDef sAlarmB;

// flag to indicate alarmA interrupt occurred
volatile uint8_t alarmAOccurred = 0;
char msg[100];

float acceleration;
uint32_t numberOfFriends = 0; // :(
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartSensor(void *argument);
void StartTelemetry(void *argument);
void StartEjection(void *argument);

/* USER CODE BEGIN PFP */
void myprintf(char *buffer) {
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, 100, 100);
}
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

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
  /* creation of Sensor */
  SensorHandle = osThreadNew(StartSensor, NULL, &Sensor_attributes);

  /* creation of Telemetry */
  TelemetryHandle = osThreadNew(StartTelemetry, NULL, &Telemetry_attributes);

  /* creation of Ejection */
  EjectionHandle = osThreadNew(StartEjection, NULL, &Ejection_attributes);

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
	while (1) {
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensor */
/**
 * @brief  Function implementing the Sensor thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensor */
void StartSensor(void *argument)
{
  /* USER CODE BEGIN 5 */
	float accels[10] = { 0.0, 0.23, 120.0, 420.0, 69.42, 10.3, 20.5, 12.12,
			42.42, 0.0 };
	uint32_t friends[10] = { 0, 1, 2, 3, 4, 5, 4, 3, 2, 1 };
	/* Infinite loop */
	uint32_t i = 0;
	char *buffer = (char*) malloc(100);
	memset(buffer, 0, 100);
	for (;;) {

//		acceleration = accels[i % 10];
//		numberOfFriends = friends[i % 10];
//		sprintf(buffer, "IN Acceleration: %f\r\n", acceleration);
//		myprintf(buffer);
//		sprintf(buffer, "IN Number of friends: %lu\r\n", numberOfFriends);
//		myprintf(buffer);
		++i;
		sprintf(buffer, "1\r\n");
		myprintf(buffer);
		osDelay(2000);

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTelemetry */
/**
 * @brief Function implementing the Telemetry thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTelemetry */
void StartTelemetry(void *argument)
{
  /* USER CODE BEGIN StartTelemetry */
	char* buffer = (char*) malloc(100);
	sprintf(buffer, "OUT Acceleration");
	memset(buffer, 0, 100);
	for (;;) {

		sprintf(buffer, "OUT Acceleration: %f\r\n", acceleration);
		myprintf(buffer);
		sprintf(buffer, "OUT Number of friends: %li\r\n", numberOfFriends);
		myprintf(buffer);
		osDelay(1000);

	}
  /* USER CODE END StartTelemetry */
}

/* USER CODE BEGIN Header_StartEjection */
/**
 * @brief Function implementing the Ejection thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartEjection */
void StartEjection(void *argument)
{
  /* USER CODE BEGIN StartEjection */
	char *buffer = (char*) malloc(100);
	sprintf(buffer, "OUT Acceleration");
	memset(buffer, 0, 100);
	for (;;) {

		if (numberOfFriends == 5) {
			sprintf(buffer, "EJECT or smthg\r\n");
			myprintf(buffer);
		}
		osDelay(1000);

	}
  /* USER CODE END StartEjection */
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
	while (1) {
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
