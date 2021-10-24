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
 * the License. You may obtain a copy of the License at:-
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* Definitions for fakeSensors */
osThreadId_t fakeSensorsHandle;
uint32_t fakeSensorsBuffer[256];
osStaticThreadDef_t fakeSensorsControlBlock;
const osThreadAttr_t fakeSensors_attributes = { .name = "fakeSensors", .cb_mem =
		&fakeSensorsControlBlock, .cb_size = sizeof(fakeSensorsControlBlock),
		.stack_mem = &fakeSensorsBuffer[0], .stack_size =
				sizeof(fakeSensorsBuffer), .priority =
				(osPriority_t) osPriorityNormal, };
/* Definitions for fakeTelemetry */
osThreadId_t fakeTelemetryHandle;
uint32_t fakeTelemetryBuffer[256];
osStaticThreadDef_t fakeTelemetryControlBlock;
const osThreadAttr_t fakeTelemetry_attributes = { .name = "fakeTelemetry",
		.cb_mem = &fakeTelemetryControlBlock, .cb_size =
				sizeof(fakeTelemetryControlBlock), .stack_mem =
				&fakeTelemetryBuffer[0], .stack_size =
				sizeof(fakeTelemetryBuffer), .priority =
				(osPriority_t) osPriorityLow, };
/* Definitions for fakeEjection */
osThreadId_t fakeEjectionHandle;
uint32_t fakeEjectionBuffer[256];
osStaticThreadDef_t fakeEjectionControlBlock;
const osThreadAttr_t fakeEjection_attributes = { .name = "fakeEjection",
		.cb_mem = &fakeEjectionControlBlock, .cb_size =
				sizeof(fakeEjectionControlBlock), .stack_mem =
				&fakeEjectionBuffer[0],
		.stack_size = sizeof(fakeEjectionBuffer), .priority =
				(osPriority_t) osPriorityLow, };
/* USER CODE BEGIN PV */
// rtc variables
RTC_DateTypeDef sdatestructureget;
RTC_TimeTypeDef stimestructureget;
RTC_AlarmTypeDef sAlarmA;
RTC_AlarmTypeDef sAlarmB;

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
static void MX_TIM16_Init(void);
static void MX_RTC_Init(void);
void StartFakeSensors(void *argument);
void StartFakeTelemetry(void *argument);
void StartFakeEjection(void *argument);

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
int main(void) {
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
	MX_TIM16_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim16);

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
	/* creation of fakeSensors */
	fakeSensorsHandle = osThreadNew(StartFakeSensors, NULL,
			&fakeSensors_attributes);

	/* creation of fakeTelemetry */
	fakeTelemetryHandle = osThreadNew(StartFakeTelemetry, NULL,
			&fakeTelemetry_attributes);

	/* creation of fakeEjection */
	fakeEjectionHandle = osThreadNew(StartFakeEjection, NULL,
			&fakeEjection_attributes);

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_TIM16;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };
	RTC_AlarmTypeDef sAlarm = { 0 };

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
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x29;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/** Enable the Alarm A
	 */
	sAlarm.AlarmTime.Hours = 0x0;
	sAlarm.AlarmTime.Minutes = 0x30;
	sAlarm.AlarmTime.Seconds = 0x0;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_HOURS;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/** Enable the Alarm B
	 */

	sAlarm.AlarmTime.Minutes = 0x31;
	sAlarm.Alarm = RTC_ALARM_B;

	/* USER CODE BEGIN RTC_Init 2 */
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_HOURS;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 7999;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 14999;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
// Callbacks
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	sprintf((char*) msg, "Alarm A callback entered\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen((char const*) msg), 1000);
	sprintf((char*) msg, "alarmA flag: %d\talarmB flag: %d\r\n\n",
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF),
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF));
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);
	// clear the alarm flag
	__HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
	while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF) != RESET)
		__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);
	__HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();

	sprintf((char*) msg, "alarmA flag after clear: %d\talarmB flag: %d\r\n\n",
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF),
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF));
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);
	alarmAOccurred = 1;

	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	  HAL_SuspendTick(); // systick generates interrupts which may wake the processor
	  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
}

void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc) {

	sprintf((char*) msg, "Alarm B callback entered\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen((char const*) msg), 1000);

	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen((char const*) msg), 1000);
	sprintf((char*) msg,
			"before clear attempt: alarmA flag: %d\talarmB flag: %d\r\n\n",
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF),
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF));
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);

	// clear the alarm flag
	__HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
	while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF) != RESET)
		__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRBF);
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
	__HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();

	sprintf((char*) msg,
			"after clear attempt: alarmA flag: %d\talarmB flag: %d\r\n\n",
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF),
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF));
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);
	alarmAOccurred = 1;

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_SuspendTick(); // systick generates interrupts which may wake the processor

}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim16) {
//		sprintf((char*) msg, "TIM16 callback entered\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen((char const*) msg),
//				1000);
//
//
//		// print current RTC time for debugging
//		HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
//		HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
//		sprintf((char*) msg, "main, current: GetTime/Date: %.2d:%.2d:%.2d\r\n",
//				stimestructureget.Hours, stimestructureget.Minutes,
//				stimestructureget.Seconds);
//		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);
//
////		// print status of alarm flags for debugging
//		sprintf((char*) msg, "alarmA flag: %d\talarmB flag: %d\r\n\n",
//				__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF),
//				__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRBF));
//		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);
//	}
//}

/*
 * polls the alarm interrupt flags and calls the appropriate callback functions.
 */
void pollAlarmInterruptFlag(void) {
	if (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != RESET)
		HAL_RTC_AlarmAEventCallback(&hrtc);
	if (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRBF) != RESET)
		HAL_RTCEx_AlarmBEventCallback(&hrtc);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartFakeSensors */
/**
 * @brief  Function implementing the fakeSensors thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartFakeSensors */
void StartFakeSensors(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	float accels[10] = { 0.0, 0.23, 120.0, 420.0, 69.42, 10.3, 20.5, 12.12,
			42.42, 0.0 };
	uint32_t friends[10] = { 0, 1, 2, 3, 4, 5, 4, 3, 2, 1 };
	/* Infinite loop */
	uint32_t i = 0;
	char *buffer = (char*) malloc(100);
	memset(buffer, 0, 100);
	for (;;) {

		acceleration = accels[i % 10];
		numberOfFriends = friends[i % 10];
		sprintf(buffer, "IN Acceleration: %f\r\n", acceleration);
		myprintf(buffer);
		sprintf(buffer, "IN Number of friends: %lu\r\n", numberOfFriends);
		myprintf(buffer);
		++i;
		osDelay(2000);

	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartFakeTelemetry */
/**
 * @brief Function implementing the fakeTelemetry thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartFakeTelemetry */
void StartFakeTelemetry(void *argument) {
	/* USER CODE BEGIN StartFakeTelemetry */
	/* Infinite loop */
	char *buffer1 = (char*) malloc(100);
	memset(buffer1, 0, 100);
	for (;;) {

		sprintf(buffer1, "OUT Acceleration: %f\r\n", acceleration);
		myprintf(buffer1);
		sprintf(buffer1, "OUT Number of friends: %li\r\n", numberOfFriends);
		myprintf(buffer1);
		osDelay(5000);

	}
	/* USER CODE END StartFakeTelemetry */
}

/* USER CODE BEGIN Header_StartFakeEjection */
/**
 * @brief Function implementing the fakeEjection thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartFakeEjection */
void StartFakeEjection(void *argument) {
	/* USER CODE BEGIN StartFakeEjection */
	/* Infinite loop */
	char *buffer = (char*) malloc(100);
	memset(buffer, 0, 100);
	for (;;) {

		if (numberOfFriends == 5) {
			sprintf(buffer, "EJECT or smthg\r\n");
			myprintf(buffer);
		}
		osDelay(1000);

	}
	/* USER CODE END StartFakeEjection */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM16) {
		pollAlarmInterruptFlag();
		char *buffer = (char*) malloc(100);
		memset(buffer, 0, 100);
		sprintf(buffer, "UwU\r\n");
		myprintf(buffer);
//		HAL_Delay(10000);
		sprintf(buffer, "wUw\r\n");
		myprintf(buffer);
	}
	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
