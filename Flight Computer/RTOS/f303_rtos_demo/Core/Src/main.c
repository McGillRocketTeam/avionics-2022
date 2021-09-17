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
#include "lps22hh_reg.h"
#include "lsm6dsr_reg.h"
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
I2C_HandleTypeDef hi2c3;

stmdev_ctx_t dev_ctx_lsm;
stmdev_ctx_t dev_ctx_lps;
float acceleration[] = {0, 0, 0};
float angular_rate[]= {0, 0, 0};
float pressure = 0;
float temperature = 0;

UART_HandleTypeDef huart2;

/* Definitions for fakeEjection */
osThreadId_t fakeEjectionHandle;
uint32_t fakeEjectionBuffer[ 1000 ];
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
uint32_t fakeSensorsBuffer[ 1000 ];
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
uint32_t fakeTelemetryBuffer[ 1000 ];
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
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
void StartFakeEjection(void *argument);
void StartFakeSensors(void *argument);
void StartFakeTelemetry(void *argument);

/* USER CODE BEGIN PFP */

// LSM6DSR functions
extern stmdev_ctx_t lsm6dsr_init(void);
extern void get_acceleration(stmdev_ctx_t dev_ctx, float *acceleration_mg);
extern void get_angvelocity(stmdev_ctx_t dev_ctx, float *angular_rate_mdps);
// LPS22HH functions
extern stmdev_ctx_t lps22hh_init(void);
extern void get_pressure(stmdev_ctx_t dev_ctx,  float *pressure);
extern void get_temperature(stmdev_ctx_t dev_ctx,  float *temperature);
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
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  dev_ctx_lsm = lsm6dsr_init();
  dev_ctx_lps = lps22hh_init();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x2000090E;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();

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
	  if(temperature > 30)
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
  /* Infinite loop */
	uint32_t i = 0;
	char* buffer = (char*)malloc(100);
	memset(buffer, 0, 100);
  for(;;)
  {
	  get_pressure(dev_ctx_lps, &pressure);
	  get_temperature(dev_ctx_lps,  &temperature);
	  get_acceleration(dev_ctx_lsm, acceleration);
	  get_angvelocity(dev_ctx_lsm, angular_rate);
	  sprintf(buffer, "IN acceleration: %f,%f,%f\r\n", acceleration[0], acceleration[1], acceleration[2]);
	  myprintf(buffer);
	  memset(buffer, 0, 100);
	  sprintf(buffer, "IN angular_rate: %f,%f,%f\r\n", angular_rate[0], angular_rate[1], angular_rate[2]);
	  myprintf(buffer);
	  memset(buffer, 0, 100);
	  sprintf(buffer, "IN pressure: %f\r\n", pressure);
	  myprintf(buffer);
	  memset(buffer, 0, 100);
	  sprintf(buffer, "IN temperature: %f\r\n", temperature);
	  myprintf(buffer);
	  memset(buffer, 0, 100);
	  ++i;
    osDelay(500);
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
	  sprintf(buffer1, "OUT acceleration: %f,%f,%f\r\n", acceleration[0], acceleration[1], acceleration[2]);
	  myprintf(buffer1);
	  memset(buffer1, 0, 100);
	  sprintf(buffer1, "OUT angular_rate: %f,%f,%f\r\n", angular_rate[0], angular_rate[1], angular_rate[2]);
	  myprintf(buffer1);
	  memset(buffer1, 0, 100);
	  sprintf(buffer1, "OUT pressure: %f\r\n", pressure);
	  myprintf(buffer1);
	  memset(buffer1, 0, 100);
	  sprintf(buffer1, "OUT temperature: %f\r\n", temperature);
	  myprintf(buffer1);
	  memset(buffer1, 0, 100);
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
