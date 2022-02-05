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
#include "fatfs.h"
#include "usb_device.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "i2c_sensor_functions.h"
#include "sd_card.h"
#include "semphr.h"
#include "gps.h"
#include "MAX31855.h"
#include "sx126x.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
stmdev_ctx_t dev_ctx_lsm;
stmdev_ctx_t dev_ctx_lps;
float acceleration[] = {0, 0, 0};
float angular_rate[]= {0, 0, 0};
float pressure = 0;
float temperature = 0;
double latitude;
double longitude;
float time;
int second = 0;
int minute = 0;

uint8_t tx_buffer[150];

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations
char filename[13];
uint8_t writeBuf[1000];
UINT bytesWrote;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart8;
UART_HandleTypeDef huart6;

/* Definitions for printSensors */
osThreadId_t printSensorsHandle;
const osThreadAttr_t printSensors_attributes = {
  .name = "printSensors",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pollSensors */
osThreadId_t pollSensorsHandle;
const osThreadAttr_t pollSensors_attributes = {
  .name = "pollSensors",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for saveData */
osThreadId_t saveDataHandle;
const osThreadAttr_t saveData_attributes = {
  .name = "saveData",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ejection */
osThreadId_t ejectionHandle;
const osThreadAttr_t ejection_attributes = {
  .name = "ejection",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for transmitData */
osThreadId_t transmitDataHandle;
const osThreadAttr_t transmitData_attributes = {
  .name = "transmitData",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mutex */
osSemaphoreId_t mutexHandle;
const osSemaphoreAttr_t mutex_attributes = {
  .name = "mutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI5_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART8_Init(void);
static void MX_SPI2_Init(void);
void printSensorsFunc(void *argument);
void pollSensorsFunc(void *argument);
void saveDataFunc(void *argument);
void ejectionFunc(void *argument);
void transmitDataFunc(void *argument);

/* USER CODE BEGIN PFP */
// LSM6DSR functions
extern stmdev_ctx_t lsm6dsr_init(void);
extern void get_acceleration(stmdev_ctx_t dev_ctx, float *acceleration_mg);
extern void get_angvelocity(stmdev_ctx_t dev_ctx, float *angular_rate_mdps);
// LPS22HH functions
extern stmdev_ctx_t lps22hh_init(void);
extern void get_pressure(stmdev_ctx_t dev_ctx,  float *pressure);
extern void get_temperature(stmdev_ctx_t dev_ctx,  float *temperature);

void GPS_Poll(double*, double*, float*);
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
  MX_ADC1_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_SPI5_Init();
  MX_FATFS_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  MX_UART8_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // reset LEDs
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);

  // reset recovery pyro pins
  HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, RESET);
  HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
  HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);

  // reset prop pyro pins
  HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, RESET);
  HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
  HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);

  // reset 12 V buck converter enable pin (disable converter)
  HAL_GPIO_WritePin(PM_12V_EN_GPIO_Port, PM_12V_EN_Pin, RESET);
  HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET);

  // reset payload EN signal
  HAL_GPIO_WritePin(Payload_EN_GPIO_Port, Payload_EN_Pin, RESET);

  // set CS pin for thermocouple chip high
  //	HAL_GPIO_WritePin(TH_CS_1_GPIO_Port, TH_CS_1_Pin, SET);

  // set power off for VR
  HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, RESET);
  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);

  dev_ctx_lsm = lsm6dsl_init();
  dev_ctx_lps = lps22hh_init();

  //SRAD Radio init
  set_hspi(hspi2);
  set_NSS_pin(SX_NSS_GPIO_Port, SX_NSS_Pin);
  set_BUSY_pin(SX_BUSY_GPIO_Port, SX_BUSY_Pin);
  set_NRESET_pin(SX_RST_GPIO_Port, SX_RST_Pin);
  set_DIO1_pin(SX_DIO_GPIO_Port, SX_DIO_Pin);
  Tx_setup();

  sd_init_dynamic_filename("FC", "", filename);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of mutex */
  mutexHandle = osSemaphoreNew(1, 1, &mutex_attributes);

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
  /* creation of printSensors */
  printSensorsHandle = osThreadNew(printSensorsFunc, NULL, &printSensors_attributes);

  /* creation of pollSensors */
  pollSensorsHandle = osThreadNew(pollSensorsFunc, NULL, &pollSensors_attributes);

  /* creation of saveData */
  saveDataHandle = osThreadNew(saveDataFunc, NULL, &saveData_attributes);

  /* creation of ejection */
  ejectionHandle = osThreadNew(ejectionFunc, NULL, &ejection_attributes);

  /* creation of transmitData */
  transmitDataHandle = osThreadNew(transmitDataFunc, NULL, &transmitData_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 9600;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PM_12V_EN_Pin|Vent_Valve_EN_Pin|Payload_EN_Pin|TH_CS_1_Pin
                          |TH_CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SD_CS_Pin|Prop_Gate_2_Pin|Prop_Gate_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin|SX_AMP_Pin
                          |VR_CTRL_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Prop_Pyro_Arming_Pin|SX_RST_Pin|SX_RF_SW_Pin|Rcov_Gate_Main_Pin
                          |Rcov_Gate_Drogue_Pin|Rcov_Arm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SX_NSS_GPIO_Port, SX_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SX_FILTER_GPIO_Port, SX_FILTER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PM_12V_EN_Pin Vent_Valve_EN_Pin Payload_EN_Pin TH_CS_1_Pin
                           TH_CS_2_Pin */
  GPIO_InitStruct.Pin = PM_12V_EN_Pin|Vent_Valve_EN_Pin|Payload_EN_Pin|TH_CS_1_Pin
                          |TH_CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin SX_AMP_Pin
                           VR_CTRL_PWR_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|SX_AMP_Pin
                          |VR_CTRL_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Vent_Valve_FB_Pin */
  GPIO_InitStruct.Pin = Vent_Valve_FB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Vent_Valve_FB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Prop_Cont_2_Pin */
  GPIO_InitStruct.Pin = Prop_Cont_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Prop_Cont_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Prop_Gate_2_Pin Prop_Gate_1_Pin */
  GPIO_InitStruct.Pin = Prop_Gate_2_Pin|Prop_Gate_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Prop_Cont_1_Pin SX_BUSY_Pin SX_DIO_Pin Rcov_Cont_Main_Pin
                           Rcov_Cont_Drogue_Pin */
  GPIO_InitStruct.Pin = Prop_Cont_1_Pin|SX_BUSY_Pin|SX_DIO_Pin|Rcov_Cont_Main_Pin
                          |Rcov_Cont_Drogue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : Prop_Pyro_Arming_Pin SX_RST_Pin SX_RF_SW_Pin Rcov_Gate_Main_Pin
                           Rcov_Gate_Drogue_Pin Rcov_Arm_Pin */
  GPIO_InitStruct.Pin = Prop_Pyro_Arming_Pin|SX_RST_Pin|SX_RF_SW_Pin|Rcov_Gate_Main_Pin
                          |Rcov_Gate_Drogue_Pin|Rcov_Arm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SX_NSS_Pin */
  GPIO_InitStruct.Pin = SX_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SX_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SX_FILTER_Pin */
  GPIO_InitStruct.Pin = SX_FILTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SX_FILTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VR_CTRL_REC_Pin */
  GPIO_InitStruct.Pin = VR_CTRL_REC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VR_CTRL_REC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_printSensorsFunc */
/**
  * @brief  Function implementing the printSensors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_printSensorsFunc */
void printSensorsFunc(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	//myprintf("ACCEL: %f, %f, %f\r\n", acceleration[0], acceleration[1], acceleration[2]);
	//myprintf("GPS: %lf, %lf, %f\r\n", latitude, longitude, time);
	//xSemaphoreTake(mutexHandle,500);
	//GPS_Poll(&latitude, &longitude, &time);
	//myprintf("GPS: %lf, %lf, %f\r\n", latitude, longitude, time);
	//myprintf("Pressure: %f, Temperature %f\r\n", pressure, temperature);
	//myprintf("Acceleration: %f,%f,%f\r\n", acceleration[0], acceleration[1], acceleration[2]);
	//myprintf("IN angular_rate: %f,%f,%f\r\n", angular_rate[0], angular_rate[1], angular_rate[2]);
	//xSemaphoreGive(mutexHandle);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_pollSensorsFunc */
/**
* @brief Function implementing the pollSensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pollSensorsFunc */
void pollSensorsFunc(void *argument)
{
  /* USER CODE BEGIN pollSensorsFunc */
  /* Infinite loop */
  for(;;)
  {
	get_pressure(dev_ctx_lps, &pressure);
	get_temperature(dev_ctx_lps,  &temperature);
	get_acceleration(dev_ctx_lsm, acceleration);
	get_angvelocity(dev_ctx_lsm, angular_rate);
	//xSemaphoreTake(mutexHandle,500);
	//GPS_Poll(&latitude, &longitude, &time);
	//xSemaphoreGive(mutexHandle);
    osDelay(100);
  }
  /* USER CODE END pollSensorsFunc */
}

/* USER CODE BEGIN Header_saveDataFunc */
/**
* @brief Function implementing the saveData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_saveDataFunc */
void saveDataFunc(void *argument)
{
  /* USER CODE BEGIN saveDataFunc */
  /* Infinite loop */

  for(;;)
  {
	//write data
	sd_open_file(&filename);
	sprintf((char*)writeBuf, "Data: %f, %f, %d, %d\r\n", pressure, temperature, minute, second);
	sd_write(&fil, writeBuf);
	f_close(&fil);
    osDelay(50);
  }
  /* USER CODE END saveDataFunc */
}

/* USER CODE BEGIN Header_ejectionFunc */
/**
* @brief Function implementing the ejection thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ejectionFunc */
void ejectionFunc(void *argument)
{
  /* USER CODE BEGIN ejectionFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END ejectionFunc */
}

/* USER CODE BEGIN Header_transmitDataFunc */
/**
* @brief Function implementing the transmitData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmitDataFunc */
void transmitDataFunc(void *argument)
{
  /* USER CODE BEGIN transmitDataFunc */
  /* Infinite loop */
  for(;;)
  {
	//S,ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,PRESSURE,LAT,LONG,MIN,SEC,SUBSEC,STATE,CONT,E
	second++;
	if(second == 60){
		second = 0;
		minute++;
	}
	sprintf((char*)tx_buffer, "S,%f,%f,%f,%f,%f,%f,%f,%lf,%lf,%d,%d,%d,%d,%d,E",acceleration[0],acceleration[1],acceleration[2],angular_rate[0],angular_rate[1],angular_rate[2],pressure,latitude,longitude,minute,second,0,0,0);
	TxProtocol(tx_buffer,150);
    osDelay(1);
  }
  /* USER CODE END transmitDataFunc */
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
