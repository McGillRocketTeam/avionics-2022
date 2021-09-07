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
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float acceleration;
uint32_t numberOfFriends = 0; // :(


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* Definitions for fakeEjection */
osThreadId_t fakeEjectionHandle;
uint32_t fakeEjectionBuffer[ 500 ];
osStaticThreadDef_t fakeEjectionControlBlock;
const osThreadAttr_t fakeEjection_attributes = {
  .name = "fakeEjection",
  .cb_mem = &fakeEjectionControlBlock,
  .cb_size = sizeof(fakeEjectionControlBlock),
  .stack_mem = &fakeEjectionBuffer[0],
  .stack_size = sizeof(fakeEjectionBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for fakeSensors */
osThreadId_t fakeSensorsHandle;
uint32_t fakeSensorsBuffer[ 500 ];
osStaticThreadDef_t fakeSensorsControlBlock;
const osThreadAttr_t fakeSensors_attributes = {
  .name = "fakeSensors",
  .cb_mem = &fakeSensorsControlBlock,
  .cb_size = sizeof(fakeSensorsControlBlock),
  .stack_mem = &fakeSensorsBuffer[0],
  .stack_size = sizeof(fakeSensorsBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fakeTelemetry */
osThreadId_t fakeTelemetryHandle;
uint32_t fakeTelemetryBuffer[ 500 ];
osStaticThreadDef_t fakeTelemetryControlBlock;
const osThreadAttr_t fakeTelemetry_attributes = {
  .name = "fakeTelemetry",
  .cb_mem = &fakeTelemetryControlBlock,
  .cb_size = sizeof(fakeTelemetryControlBlock),
  .stack_mem = &fakeTelemetryBuffer[0],
  .stack_size = sizeof(fakeTelemetryBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartFakeEjection(void *argument);
void StartFakeSensors(void *argument);
void StartFakeTelemetry(void *argument);

/* USER CODE BEGIN PFP */
/*
void myprintf(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart3, (uint8_t*) buffer, len, -1);
}
*/
void myprintf(char* buffer) {
	HAL_UART_Transmit(&huart3, (uint8_t*) buffer, 100, 100);
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
  MX_USART3_UART_Init();
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
  /* creation of fakeEjection */
  fakeEjectionHandle = osThreadNew(StartFakeEjection, NULL, &fakeEjection_attributes);

  /* creation of fakeSensors */
  fakeSensorsHandle = osThreadNew(StartFakeSensors, NULL, &fakeSensors_attributes);

  /* creation of fakeTelemetry */
  fakeTelemetryHandle = osThreadNew(StartFakeTelemetry, NULL, &fakeTelemetry_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartFakeEjection */
/**
  * @brief  Function implementing the fakeEjection thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFakeEjection */
void StartFakeEjection(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	char* buffer = (char*)malloc(100);
	memset(buffer, 0, 100);
  for(;;)
  {
	  if(numberOfFriends == 5)
	  {
		  sprintf(buffer, "EJECT or smthg\r\n");
		  myprintf(buffer);
	  }
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartFakeSensors */
/**
* @brief Function implementing the fakeSensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFakeSensors */
void StartFakeSensors(void *argument)
{
  /* USER CODE BEGIN StartFakeSensors */
	float accels[10] = {0.0, 0.23, 120.0, 420.0, 69.42, 10.3, 20.5, 12.12, 42.42, 0.0};
	uint32_t friends[10] = {0, 1, 2, 3, 4, 5, 4, 3, 2, 1};
  /* Infinite loop */
	uint32_t i = 0;
	char* buffer = (char*)malloc(100);
	memset(buffer, 0, 100);
  for(;;)
  {
	  acceleration = accels[i%10];
	  numberOfFriends = friends[i%10];
	  sprintf(buffer, "IN Acceleration: %f\r\n", acceleration);
	  myprintf(buffer);
	  sprintf(buffer, "IN Number of friends: %lu\r\n", numberOfFriends);
	  myprintf(buffer);
	  ++i;
    osDelay(2000);
  }
  /* USER CODE END StartFakeSensors */
}

/* USER CODE BEGIN Header_StartFakeTelemetry */
/**
* @brief Function implementing the fakeTelemetry thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFakeTelemetry */
void StartFakeTelemetry(void *argument)
{
  /* USER CODE BEGIN StartFakeTelemetry */
  /* Infinite loop */
	char* buffer1 = (char*)malloc(100);
	memset(buffer1, 0, 100);
  for(;;)
  {
	  sprintf(buffer1, "OUT Acceleration: %f\r\n", acceleration);
	  myprintf(buffer1);
	  sprintf(buffer1, "OUT Number of friends: %li\r\n", numberOfFriends);
	  myprintf(buffer1);
    osDelay(5000);
  }
  /* USER CODE END StartFakeTelemetry */
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
