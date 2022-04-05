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


//TODO uncomment watchdog dog and iridium code


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <MRT_RTOS.h>
#include <IridiumSBD_Static_API.h>
#include <MRT_Helpers.h>
#include <i2c_sensors.h>
#include <gps.h>
#include <sx126x.h>
#include <MAX31855.h>

//#include <MRT_setup.h> included in main.h

#include <math.h>
//#include <usbd_cdc_if.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//TODO DEFINES ARE IN main.h
#define DIAGNOSTICS false

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for Memory0 */
osThreadId_t Memory0Handle;
const osThreadAttr_t Memory0_attributes = {
  .name = "Memory0",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for Ejection1 */
osThreadId_t Ejection1Handle;
const osThreadAttr_t Ejection1_attributes = {
  .name = "Ejection1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Telemetry2 */
osThreadId_t Telemetry2Handle;
const osThreadAttr_t Telemetry2_attributes = {
  .name = "Telemetry2",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Sensors3 */
osThreadId_t Sensors3Handle;
const osThreadAttr_t Sensors3_attributes = {
  .name = "Sensors3",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Printing */
osThreadId_t PrintingHandle;
const osThreadAttr_t Printing_attributes = {
  .name = "Printing",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for WatchDog */
osThreadId_t WatchDogHandle;
const osThreadAttr_t WatchDog_attributes = {
  .name = "WatchDog",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */


//**************************************************//
//GENERAL

//Jasper's variables
volatile uint8_t start_ejection = 0;
volatile uint8_t timer_actuated_vent_valve = 0;

//Eject data
float altitude_m = 0;


//**************************************************//
//MEMORY
// sd card
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations
char filename[13];
uint8_t writeBuf[1000];


//**************************************************//
//TELEMETRY
#if XTEND_
char xtend_tx_buffer[XTEND_BUFFER_SIZE]; // XTend Transmit
uint8_t xtend_rx_buffer[XTEND_BUFFER_SIZE]; // XTend Receive
#elif SRADIO_
uint8_t sradio_rx_buffer[64] = {0}; // SRADio Reception buffer
char sradio_tx_buffer[SRADIO_BUFFER_SIZE]; // SRADio Transmit
#endif

//Telemetry variables
float ACCx;
float ACCy;
float ACCz;
float GYROx;
float GYROy;
float GYROz;
float PRESSURE;
float LATITUDE;
float LONGITUDE;
float MIN;
float SEC;
float SUBSEC;
float STATE;
uint8_t CONT;

float TANK_PRESSURE;
float THERMO_TEMPERATURE;
uint8_t VALVE_STATUS;


//**************************************************//
//SENSORS
//Propulsion
float transducer_pressure;

// GPS data
float time;
static uint8_t gps_fix_lat = 0;
static uint8_t gps_fix_long = 0; // beep when we get fix
char gps_data[GPS_DATA_BUF_DIM];

//I2C devices
stmdev_ctx_t lsm_ctx;
stmdev_ctx_t lps_ctx;


//**************************************************//
//WATCH DOG
osThreadId_t threadID[NUMBER_OF_THREADS]; //Thread list accessed by Watch Dog thread



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
void StartMemory0(void *argument);
void StartEjection1(void *argument);
void StartTelemetry2(void *argument);
void StartSensors3(void *argument);
void StartPrinting(void *argument);
void StartWatchDog(void *argument);

/* USER CODE BEGIN PFP */

//TODO
// XTend transmit function
static void XTend_Transmit(char *Msg);

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
  MX_I2C1_Init();
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
  //MX_IWDG_Init(); TODO remove
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */




  //**************************************************//
  //GENERAL

  /*
   * Reinitialize all peripherals
   */

  // reset LEDs
  HAL_GPIO_WritePin(OUT_LED1_GPIO_Port, OUT_LED1_Pin, RESET);
  HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, RESET);
  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);

  // reset recovery pyro pins
  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, SET); //PG14 ARMING RCOV
  HAL_GPIO_WritePin(OUT_EJ_Drogue_Gate_GPIO_Port, OUT_EJ_Drogue_Gate_Pin, RESET); //PG12 DROGUE GATE
  HAL_GPIO_WritePin(OUT_EJ_Main_Gate_GPIO_Port, OUT_EJ_Main_Gate_Pin, RESET); //PG11 MAIN GATE

  // reset prop pyro pins
  HAL_GPIO_WritePin(OUT_PyroValve_Arming_GPIO_Port, OUT_PyroValve_Arming_Pin, SET); //PG1 ARMING_PROP
  HAL_GPIO_WritePin(OUT_PyroValve_Gate_1_GPIO_Port, OUT_PyroValve_Gate_1_Pin, RESET); //PF15 PROP GATE 1
  HAL_GPIO_WritePin(OUT_PyroValve_Gate_2_GPIO_Port,OUT_PyroValve_Gate_2_Pin, RESET); //PF14 PROP GATE 2

  // reset 12 V buck converter enable pin (disable converter)
  HAL_GPIO_WritePin(EN_12V_Buck_GPIO_Port, EN_12V_Buck_Pin, RESET); //PE2 Buck converter enable

  // TODO Couldn't find the pin of the vent gate enable
  //HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET); //This was in the previous code
  //HAL_GPIO_WritePin(OUT_Prop_ActuatedVent_Gate_GPIO_Port, OUT_Prop_ActuatedVent_Gate_Pin, RESET); //PE7 (MAY NOT BE THE RIGHT ONE)


  // reset payload EN signal
  HAL_GPIO_WritePin(PAYLOAD_I2C_EN_GPIO_Port, PAYLOAD_I2C_EN_Pin, RESET); //PE9 Payload I2C enable

  // set CS pin for thermocouple chip high
  //	HAL_GPIO_WritePin(TH_CS_1_GPIO_Port, TH_CS_1_Pin, SET);

  // set power off for VR
  HAL_GPIO_WritePin(OUT_VR_PWR_GPIO_Port, OUT_VR_PWR_Pin, RESET); //PG9
  HAL_GPIO_WritePin(OUT_VR_REC_GPIO_Port, OUT_VR_REC_Pin, RESET); //PD7

  // FLASH set CS, WP and IO3 pins high
  HAL_GPIO_WritePin(OUT_FLASH_CS_GPIO_Port, OUT_FLASH_CS_Pin, SET);
  HAL_GPIO_WritePin(OUT_FLASH_WP_GPIO_Port, OUT_FLASH_WP_Pin, SET);
  HAL_GPIO_WritePin(OUT_FLASH_IO3_GPIO_Port, OUT_FLASH_IO3_Pin, SET);

  //TODO
  /*
   * In general:
   *-Activate freeRTOS
   *-Change SysTic to any other timer (done in .ioc)
   *-Include the path to all includes folders of the drivers (for C and C++ linkers)
   */

  HAL_UART_Transmit(&DEBUG_UART,"\r\n\r\nStarting FC\r\n\r\n",19,HAL_MAX_DELAY);

  /*
   * For external FLASH memory
   *-Put before RTOS setup because you need the external flash in its setup
   */
    MRT_SetupRTOS(&hrtc, DEBUG_UART, SLEEP_TIME); //Put here so we can pass the uart value to the setup
	MRT_externalFlashSetup(&DEBUG_UART);


	  /*
	   * Watch dog
	   * -Remove the MX_IWDG_Init() that is auto-generated and add it just before the osKernelStart
	   * -Need to be put after RTOS setup
	   */
	#if IWDG_ACTIVE
	MX_IWDG_Init();
	#endif

  //RTC
  MRT_setRTC(prev_hours,prev_min,prev_sec);
  HAL_Delay(2000); //To make sure that when you set the Alarm it doesn't go off automatically
  #if ALARM_A_ACTIVE
    if (wu_flag == 0){
    	MRT_setAlarmA(PRE_WHEN_SLEEP_TIME_HOURS, PRE_WHEN_SLEEP_TIME_MIN, PRE_WHEN_SLEEP_TIME_SEC);
    }
    else{
    	MRT_setAlarmA(POST_WHEN_SLEEP_TIME_HOURS, POST_WHEN_SLEEP_TIME_MIN, POST_WHEN_SLEEP_TIME_SEC);
    }
  #endif





	  //**************************************************//
	  //MEMORY THREAD
#if MEMORY_THREAD
	  #if IWDG_ACTIVE
	    HAL_IWDG_Refresh(&hiwdg);
	  #endif

		//SD card
		sd_init_dynamic_filename("FC", "", filename);
#endif






    //**************************************************//
    //SENSORS THREAD
#if SENSORS_THREAD
	#if CHECK_I2C
      checkForI2CDevices(huart8,hi2c1);
      checkForI2CDevices(huart8,hi2c2);
      checkForI2CDevices(huart8,hi2c3);
	#endif

	/*
     * hi2c3:
	 * -Barometer: 0x5C
	 * -6 DOF IMU (LSM6DSR): 0x6A
	 * -LPS22HH: 0x5C
	 */

	  #if IWDG_ACTIVE
		HAL_IWDG_Refresh(&hiwdg);
	  #endif
	  lsm_ctx = MRT_LSM6DSR_Setup(&LSM_I2C, &DEBUG_UART);

	  #if IWDG_ACTIVE
	    HAL_IWDG_Refresh(&hiwdg);
	  #endif
	  lps_ctx = MRT_LPS22HH_Setup(&LPS_I2C, &DEBUG_UART);

	  GPS_init(&GPS_UART, &DEBUG_UART);
#endif






	  //**************************************************//
	  //TELEMETRY_THREAD
#if TELEMETRY_THREAD
	  #if IWDG_ACTIVE
		HAL_IWDG_Refresh(&hiwdg);
   	  #endif

	  #if XTEND_
	   HAL_GPIO_WritePin(XTend_CTS_Pin, GPIO_PIN_10, GPIO_PIN_RESET); //TODO is it necessary?
	  #elif SRADIO_
	  set_hspi(SRADIO_SPI);
	  // SPI2_SX_CS_GPIO_Port TODO ???
	  set_NSS_pin(SPI2_SX_CS_GPIO_Port, SPI2_SX_CS_Pin);
	  set_BUSY_pin(SX_BUSY_GPIO_Port, SX_BUSY_Pin);
	  set_NRESET_pin(SX_RST_GPIO_Port, SX_RST_Pin);
	  set_DIO1_pin(SX_DIO_GPIO_Port, SX_DIO_Pin);
	  Tx_setup();
	  #endif

	  #if IWDG_ACTIVE
		HAL_IWDG_Refresh(&hiwdg);
	  #endif

	  #if IRIDIUM_
	  HAL_GPIO_WritePin(Iridium_RST_GPIO_Port, Iridium_RST_Pin, SET);
	  uint8_t lol = MRT_Static_Iridium_Setup(DEBUG_UART, IRIDIUM_TIMEOUT, IRIDIUM_I2C); //TODO remove lol?
	  #endif
#endif


//TODO DISABLE EXTERNAL BUTTON INTERRUPT ONCE ROCKET IS ARMED (or find other way to completely reset the board)


#if FORCED_APOGEE
	  apogee_flag = 2; //Flag set to 'forced'. The value of 2 makes it such that it doesn't affect the external flash
#endif

#if FORCED_EJECTION_STAGE
	  //TODO
#endif


	  //**************************************************//

	  //Poll propulsion until launch command sent

  	  memset(xtend_rx_buffer, 0, XTEND_BUFFER_SIZE); //clear the buffer

	  while(strcmp(xtend_rx_buffer, "launch") != 0 && wu_flag == 0 && apogee_flag == 0){ //TODO need to change flag conditions
		  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, SET);

		  HAL_IWDG_Refresh(&hiwdg);

		  //Poll propulsion sensors

		  //Thermocouple
		  Max31855_Read_Temp();

		  //Pressure tank
		  transducer_pressure = MRT_prop_poll_pressure_transducer(&hadc1);


		  //Get propulsion data TODO
		  TANK_PRESSURE = transducer_pressure;
		  THERMO_TEMPERATURE = THERMO_TEMP;
		  VALVE_STATUS = 0;

		  //Send propulsion data
		  #if XTEND_ //Xtend send
	  		memset(xtend_tx_buffer, 0, XTEND_BUFFER_SIZE);
	  		sprintf(xtend_tx_buffer,"P,%.2f,%.2f, %i,E",TANK_PRESSURE,THERMO_TEMPERATURE,VALVE_STATUS);
	  		XTend_Transmit(xtend_tx_buffer);

		  	//Check for launch command
		  	memset(xtend_rx_buffer, 0, XTEND_BUFFER_SIZE);
		  	HAL_UART_Receive(&XTEND_UART, xtend_rx_buffer, sizeof(char) * 6, 0x500); //TODO timeout is about 1.2 sec (should be less than 5 sec)

		  #elif SRADIO_ //SRadio send
	    	memset(sradio_tx_buffer, 0, SRADIO_BUFFER_SIZE);
	    	sprintf(sradio_tx_buffer,"P,%.2f,%.2f, %i,E",TANK_PRESSURE,THERMO_TEMPERATURE,VALVE_STATUS);
	    	TxProtocol(sradio_tx_buffer, strlen(sradio_tx_buffer));

	    	//Check for launch command
	    	//TODO
		  #endif

	  	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);


	  	  //Reset IWDG timer
	  	  HAL_IWDG_Refresh(&hiwdg);

	      HAL_Delay(1000/PRE_APOGEE_SEND_FREQ);
	  }


	  //Send acknowledgement
	#if XTEND_ //Xtend send
		memset(xtend_tx_buffer, 0, XTEND_BUFFER_SIZE);
		sprintf(xtend_tx_buffer,"LAUNCH COMMAND RECEIVED");
		XTend_Transmit(xtend_tx_buffer);
	#elif SRADIO_ //SRadio send
		memset(sradio_tx_buffer, 0, SRADIO_BUFFER_SIZE);
		sprintf(sradio_tx_buffer,"LAUNCH COMMAND RECEIVED");
		TxProtocol(sradio_tx_buffer, strlen(sradio_tx_buffer));
	#endif




//TODO I2C SENSORS SOMETIMES DON'T WANT TO WORK ANYMORE -> NEED TO RESET THE POWER

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
  /* creation of Memory0 */
  Memory0Handle = osThreadNew(StartMemory0, NULL, &Memory0_attributes);

  /* creation of Ejection1 */
  Ejection1Handle = osThreadNew(StartEjection1, NULL, &Ejection1_attributes);

  /* creation of Telemetry2 */
  Telemetry2Handle = osThreadNew(StartTelemetry2, NULL, &Telemetry2_attributes);

  /* creation of Sensors3 */
  Sensors3Handle = osThreadNew(StartSensors3, NULL, &Sensors3_attributes);

  /* creation of Printing */
  PrintingHandle = osThreadNew(StartPrinting, NULL, &Printing_attributes);

  /* creation of WatchDog */
  WatchDogHandle = osThreadNew(StartWatchDog, NULL, &WatchDog_attributes);

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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 2499;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  sAlarm.AlarmTime.Minutes = 0x1;
  sAlarm.AlarmTime.Seconds = 0x0;
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
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  huart8.Init.BaudRate = 115200;
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
  huart3.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOE, EN_12V_Buck_Pin|OUT_Prop_ActuatedVent_Gate_Pin|SPI4_CS_Thermocouple_Pin|Iridium_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SD_CS_Pin|OUT_PyroValve_Gate_2_Pin|OUT_PyroValve_Gate_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUT_LED1_Pin|OUT_LED2_Pin|OUT_LED3_Pin|SX_AMPLIFIER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_LEDF_GPIO_Port, OUT_LEDF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, OUT_PyroValve_Arming_Pin|SX_RST_Pin|SX_RF_SW_Pin|OUT_VR_PWR_Pin
                          |OUT_EJ_Main_Gate_Pin|OUT_EJ_Drogue_Gate_Pin|OUT_EJ_Arming_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_SX_CS_GPIO_Port, SPI2_SX_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, XTend_CTS_Pin|XTend_RTS_Pin|XTend_SLEEP_Pin|XTend_RX_LED_Pin
                          |XTend_TX_PWR_Pin|OUT_FLASH_IO3_Pin|OUT_FLASH_WP_Pin|OUT_FLASH_CS_Pin
                          |OUT_VR_REC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_12V_Buck_Pin OUT_Prop_ActuatedVent_Gate_Pin SPI4_CS_Thermocouple_Pin Iridium_RST_Pin */
  GPIO_InitStruct.Pin = EN_12V_Buck_Pin|OUT_Prop_ActuatedVent_Gate_Pin|SPI4_CS_Thermocouple_Pin|Iridium_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin OUT_PyroValve_Gate_2_Pin OUT_PyroValve_Gate_1_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|OUT_PyroValve_Gate_2_Pin|OUT_PyroValve_Gate_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_Button_Pin */
  GPIO_InitStruct.Pin = IN_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_LED1_Pin OUT_LED2_Pin OUT_LED3_Pin SX_AMPLIFIER_Pin */
  GPIO_InitStruct.Pin = OUT_LED1_Pin|OUT_LED2_Pin|OUT_LED3_Pin|SX_AMPLIFIER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_LEDF_Pin */
  GPIO_InitStruct.Pin = OUT_LEDF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_LEDF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_Prop_PyroTurboValve_LimitSwitch_Pin IN_SD_CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = IN_Prop_PyroTurboValve_LimitSwitch_Pin|IN_SD_CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_Prop_ActuatedVent_Feedback_Pin */
  GPIO_InitStruct.Pin = IN_Prop_ActuatedVent_Feedback_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_Prop_ActuatedVent_Feedback_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_PyroValve_Cont_2_Pin */
  GPIO_InitStruct.Pin = IN_PyroValve_Cont_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_PyroValve_Cont_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_PyroValve_Cont_1_Pin SX_BUSY_Pin SX_DIO_Pin IN_EJ_Main_Cont_Pin
                           IN_EJ_Drogue_Cont_Pin */
  GPIO_InitStruct.Pin = IN_PyroValve_Cont_1_Pin|SX_BUSY_Pin|SX_DIO_Pin|IN_EJ_Main_Cont_Pin
                          |IN_EJ_Drogue_Cont_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_PyroValve_Arming_Pin SX_RST_Pin SX_RF_SW_Pin OUT_VR_PWR_Pin
                           OUT_EJ_Main_Gate_Pin OUT_EJ_Drogue_Gate_Pin OUT_EJ_Arming_Pin */
  GPIO_InitStruct.Pin = OUT_PyroValve_Arming_Pin|SX_RST_Pin|SX_RF_SW_Pin|OUT_VR_PWR_Pin
                          |OUT_EJ_Main_Gate_Pin|OUT_EJ_Drogue_Gate_Pin|OUT_EJ_Arming_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PAYLOAD_I2C_EN_Pin IN_XTend_Continuity_Pin */
  GPIO_InitStruct.Pin = PAYLOAD_I2C_EN_Pin|IN_XTend_Continuity_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_SX_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_SX_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_SX_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XTend_CTS_Pin XTend_RTS_Pin XTend_SLEEP_Pin XTend_RX_LED_Pin
                           XTend_TX_PWR_Pin OUT_FLASH_IO3_Pin OUT_FLASH_WP_Pin OUT_FLASH_CS_Pin
                           OUT_VR_REC_Pin */
  GPIO_InitStruct.Pin = XTend_CTS_Pin|XTend_RTS_Pin|XTend_SLEEP_Pin|XTend_RX_LED_Pin
                          |XTend_TX_PWR_Pin|OUT_FLASH_IO3_Pin|OUT_FLASH_WP_Pin|OUT_FLASH_CS_Pin
                          |OUT_VR_REC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SX_BANDPASS_FILTER_Pin */
  GPIO_InitStruct.Pin = SX_BANDPASS_FILTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SX_BANDPASS_FILTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_LPS22HH_DRDY_Pin EXTI_ISM330DCL_INT2_Pin EXTI_LSM6DSR_INT1_Pin */
  GPIO_InitStruct.Pin = EXTI_LPS22HH_DRDY_Pin|EXTI_ISM330DCL_INT2_Pin|EXTI_LSM6DSR_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

//TODO Maybe put this in another file to include

/**
 * @brief   Function to transmit message to XTend
 * @param  Msg : char array (range 1-800)
 */
static void XTend_Transmit(char* Msg){
	HAL_UART_Transmit(&XTEND_UART, Msg, strlen(Msg), HAL_Delay);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMemory0 */
/**
  * @brief  Function implementing the Memory0 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMemory0 */
void StartMemory0(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

	//Add thread id to the list
	threadID[0]=osThreadGetId();

	#if !MEMORY_THREAD
    osThreadExit();
	#endif

	uint8_t counter = 0;


	  /* Infinite loop */
	  for(;;)
	  {
		  //Write data to sd and flash
		  if(counter==1) sd_open_file(&filename);
		  sprintf((char*)writeBuf, "Data: %f, %f, %f, %f\r\n", PRESSURE, MIN, SEC, SUBSEC);
		  sd_write(&fil, writeBuf);
		  if (counter == 50) {
			  f_close(&fil);
			  counter = 0;
		  }
		  counter++;

		  osDelay(1000/DATA_FREQ);
	  }

	  //In case it leaves the infinite loop
	  HAL_UART_Transmit(&DEBUG_UART,"Something went wrong with thread 0\r\n",36,HAL_MAX_DELAY);
	  osThreadExit();

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartEjection1 */
/**
* @brief Function implementing the Ejection1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEjection1 */
void StartEjection1(void *argument)
{
  /* USER CODE BEGIN StartEjection1 */

	//Add thread id to the list
	threadID[1]=osThreadGetId();

	#if !EJECTION_THREAD
	osThreadExit();
	#endif


	//TODO add flag for when we are done with the thread
	if (altitude_m < GROUND_LEVEL)  osThreadExit();
	if (wu_flag) osThreadExit(); //WHEN WAKING UP

	char buffer[TX_BUF_DIM];

	  /* Infinite loop */
	  for(;;)
	  {

		  if (MIN_APOGEE <= altitude_m && MAX_APOGEE < altitude_m){

			  HAL_UART_Transmit(&DEBUG_UART, "Eject Drogue\r\n", 15, HAL_MAX_DELAY);

			  while(!HAL_GPIO_ReadPin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin)){
				  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, SET); //PG14 ARMING RCOV
			  }
			  while(!HAL_GPIO_ReadPin(OUT_EJ_Drogue_Gate_GPIO_Port, OUT_EJ_Drogue_Gate_Pin)){
				  HAL_GPIO_WritePin(OUT_EJ_Drogue_Gate_GPIO_Port, OUT_EJ_Drogue_Gate_Pin, SET); //PG12 DROGUE GATE
			  }
			  while(HAL_GPIO_ReadPin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin)){
				  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, RESET); //PG14 ARMING RCOV
			  }

			  for(;;){

				  //We reached main deployment altitude
				  if (altitude_m>DEPLOY_ALT_MIN && altitude_m<DEPLOY_ALT_MAX){

					  HAL_UART_Transmit(&DEBUG_UART, "Eject Main\r\n", 13, HAL_MAX_DELAY);

					  while(!HAL_GPIO_ReadPin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin)){
						  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, SET); //PG14 ARMING RCOV
					  }
					  while(!HAL_GPIO_ReadPin(OUT_EJ_Main_Gate_GPIO_Port, OUT_EJ_Main_Gate_Pin)){
						  HAL_GPIO_WritePin(OUT_EJ_Main_Gate_GPIO_Port, OUT_EJ_Main_Gate_Pin, SET); //PG11 MAIN GATE
					  }
					  while(HAL_GPIO_ReadPin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin)){
						  HAL_GPIO_WritePin(OUT_EJ_Arming_GPIO_Port, OUT_EJ_Arming_Pin, RESET); //PG14 ARMING RCOV
					  }

					  for(;;){

						  if (altitude_m < GROUND_LEVEL)  osThreadExit();

						  osDelay(100);
					  }
				  }

				  osDelay(100);
			  }
		  }

	    osDelay(100);
	  }

	  //In case it leaves the infinite loop
	  HAL_UART_Transmit(&DEBUG_UART,"Something went wrong with thread 1\r\n",36,HAL_MAX_DELAY);
	  osThreadExit();

  /* USER CODE END StartEjection1 */
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

	#if !TELEMETRY_THREAD
	osThreadExit();
	#endif

	uint8_t counter = 0;
	uint8_t iridium_counter = 0;

	osDelay(1000);

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, SET);

	  if(apogee_flag == 0){ //Only send prop data pre-apogee
		  //Get propulsion data TODO
		  TANK_PRESSURE = transducer_pressure;
		  THERMO_TEMPERATURE = THERMO_TEMP;
		  VALVE_STATUS = 0;

		  //Send propulsion data
		  #if XTEND_ //Xtend send
			memset(xtend_tx_buffer, 0, XTEND_BUFFER_SIZE);
			sprintf(xtend_tx_buffer,"P,%.2f,%.2f, %i,E",TANK_PRESSURE,THERMO_TEMPERATURE,VALVE_STATUS);
			XTend_Transmit(xtend_tx_buffer);
		  #elif SRADIO_ //SRadio send
			memset(sradio_tx_buffer, 0, SRADIO_BUFFER_SIZE);
			sprintf(sradio_tx_buffer,"P,%.2f,%.2f, %i,E",TANK_PRESSURE,THERMO_TEMPERATURE,VALVE_STATUS);
			TxProtocol(sradio_tx_buffer, strlen(sradio_tx_buffer));
		  #endif
	  }


	  if (counter == SENSORS_SEND_FREQ_DIVIDER){
		  counter = 0;

		  //Get sensors data
		  //TODO Need to verify these six to make sure they are in the right order
	  	  ACCx = acceleration_mg[0];
	  	  ACCy = acceleration_mg[1];
	  	  ACCz = acceleration_mg[2];
	  	  GYROx = angular_rate_mdps[0];
	  	  GYROy = angular_rate_mdps[1];
	  	  GYROz = angular_rate_mdps[2];
	  	  PRESSURE = pressure_hPa;
	  	  //LATITUDE Poll already updated
	  	  //LONGITUDE already updated

		  //TODO Need to make this 't' variable from the Iridium or convert the seconds from the GPS
		  /*
		  HOUR = t.tm_hour;
		  MIN = t.tm_min;
		  SEC = t.tm_sec;
		  */

		  //From the GPS time value
		  MIN = ((uint8_t) time % 3600) / 60.0; sprintf(&MIN, "%.0f",MIN);
		  SEC = (uint8_t) time % 60; sprintf(&SEC,"%.0f",SEC);
		  SUBSEC = time / 3600.0; sprintf(&SUBSEC,"%.0f",SUBSEC);
	  	  STATE = 0; //TODO not the right value
	  	  CONT = MRT_getContinuity();

	  	  //Send sensors data
		  #if XTEND_ //Xtend send
			memset(xtend_tx_buffer, 0, XTEND_BUFFER_SIZE);
			sprintf(xtend_tx_buffer,"S,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.7f,%.7f,%.1f,%.1f,%.1f,%.2f,%i,E",
										ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,PRESSURE,LATITUDE,LONGITUDE,MIN,SEC,SUBSEC,STATE,CONT);
			XTend_Transmit(xtend_tx_buffer);
		  #elif SRADIO_ //SRadio send
			memset(sradio_tx_buffer, 0, SRADIO_BUFFER_SIZE);
			sprintf(sradio_tx_buffer,"S,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.7f,%.7f,%.1f,%.1f,%.1f,%.2f,%i,E",
										ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,PRESSURE,LATITUDE,LONGITUDE,MIN,SEC,SUBSEC,STATE,CONT);
			TxProtocol(sradio_tx_buffer, strlen(sradio_tx_buffer));
		  #endif


		  if(apogee_flag && iridium_counter == IRIDIUM_SEND_FREQ_DIVIDER){
			  iridium_counter = 0;
			  #if IRIDIUM_ //Iridium send
			  MRT_Static_Iridium_getTime(); //TODO doesn't cost anything
			  //MRT_Static_Iridium_sendMessage(msg); TODO IT COSTS CREDITS WATCH OUT
			  #endif
		  }
		  iridium_counter++;
	  }
	  counter++;


	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);

	  if (apogee_flag){
		  osDelay(1000/POST_APOGEE_SEND_FREQ);
	  }
	  else{
		  osDelay(1000/PRE_APOGEE_SEND_FREQ);
	  }
  }

  //In case it leaves the infinite loop
  HAL_UART_Transmit(&DEBUG_UART,"Something went wrong with thread 2\r\n",36,HAL_MAX_DELAY);
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

	//Add thread id to the list
	threadID[3]=osThreadGetId();

	#if !SENSORS_THREAD
	osThreadExit();
	#endif

	uint8_t counter = 0;

  for(;;)
  {

	  HAL_GPIO_WritePin(OUT_LED1_GPIO_Port, OUT_LED1_Pin, SET);

	  if (counter == SENSORS_POLL_FREQ_DIVIDER){
		  counter=0;

		  //GPS
		  GPS_Poll(&LATITUDE, &LONGITUDE, &time);

	  	  //LSM6DSR
	  	  MRT_LSM6DSR_getAcceleration(lsm_ctx,acceleration_mg);
	  	  MRT_LSM6DSR_getAngularRate(lsm_ctx,angular_rate_mdps);
		  MRT_LSM6DSR_getTemperature(lsm_ctx,&lsm_temperature_degC);

		  //LPS22HH
		  MRT_LPS22HH_getTemperature(lps_ctx,&lps_temperature_degC);
		  MRT_LPS22HH_getPressure(lps_ctx,&pressure_hPa);
		  altitude_m = MRT_getAltitude(pressure_hPa); //Update altitude
	  }
	  counter++;


	  if(apogee_flag == 0){
		  //Poll propulsion sensors

		  //Thermocouple
		  Max31855_Read_Temp();

		  //Pressure tank
		  transducer_pressure = MRT_prop_poll_pressure_transducer(&hadc1);
	  }

	  HAL_GPIO_WritePin(OUT_LED1_GPIO_Port, OUT_LED1_Pin, RESET);

	  if (apogee_flag){
		  osDelay(1000/POST_APOGEE_POLL_FREQ);
	  }
	  else{
		  osDelay(1000/PRE_APOGEE_POLL_FREQ);
	  }
}

  //In case it leaves the infinite loop
  HAL_UART_Transmit(&DEBUG_UART,"Something went wrong with thread 3\r\n",36,HAL_MAX_DELAY);

  osThreadExit();

  /* USER CODE END StartSensors3 */
}

/* USER CODE BEGIN Header_StartPrinting */
/**
* @brief Function implementing the Printing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrinting */
void StartPrinting(void *argument)
{
  /* USER CODE BEGIN StartPrinting */

	#if !PRINTING_THREAD
	osThreadExit();
	#endif

	char buffer[TX_BUF_DIM];

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, SET);

	  //GPS
  	  /*
  	   * TODO HOW DO WE RESET THE TIME
  	   */
	  memset(gps_data, 0, GPS_DATA_BUF_DIM);
	  sprintf(gps_data,"Alt: %.2f   Long: %.2f   Time: %.0f\r\n",LATITUDE, LONGITUDE, time);
	  HAL_UART_Transmit(&DEBUG_UART,gps_data,strlen(gps_data),HAL_MAX_DELAY);

  	  //LSM6DSR
  	  memset(buffer, 0, TX_BUF_DIM);
  	  sprintf(buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
  	  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);

  	  /*
  	   * TODO NEEDS FILTERING BUT WORKS (maybe acceleration needs filtering too)
  	   */
  	  memset(buffer, 0, TX_BUF_DIM);
  	  sprintf(buffer,"Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
  	  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);

	  memset(buffer, 0, TX_BUF_DIM);
	  sprintf(buffer, "Temperature [degC]:%6.2f\r\n", lsm_temperature_degC);
	  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);


	  //LPS22HH
  	  memset(buffer, 0, TX_BUF_DIM);
  	  sprintf(buffer,"Pressure [hPa]:%6.2f\r\n",pressure_hPa);
  	  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);

	  memset(buffer, 0, TX_BUF_DIM);
	  sprintf(buffer, "Temperature [degC]:%6.2f\r\n", lps_temperature_degC);
	  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);


	  //Thermocouple
	  memset(buffer, 0, TX_BUF_DIM);
	  sprintf(buffer, "Thermocouple temperature [degC]: %6.2f\r\n", THERMO_TEMP);
	  HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);


	  //Iridium
	  #if IRIDIUM_
	  MRT_Static_Iridium_getTime(); //TODO Can get stuck for some time (SHOULD CHANGE TIMEOUT)
	  #endif

	  HAL_GPIO_WritePin(OUT_LED3_GPIO_Port, OUT_LED3_Pin, RESET);

	  if (apogee_flag){
		  osDelay(1000/POST_APOGEE_SEND_FREQ);
	  }
	  else{
		  osDelay(1000/PRE_APOGEE_SEND_FREQ);
	  }
}

  //In case it leaves the infinite loop
  HAL_UART_Transmit(&DEBUG_UART,"Something went wrong with thread p\r\n",36,HAL_MAX_DELAY);
  osThreadExit();
  /* USER CODE END StartPrinting */
}

/* USER CODE BEGIN Header_StartWatchDog */
/**
* @brief Function implementing the WatchDog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWatchDog */
void StartWatchDog(void *argument)
{
  /* USER CODE BEGIN StartWatchDog */

	#if !WATCHDOG_THREAD
	osThreadExit();
	#endif

	char buffer[TX_BUF_DIM];

	osThreadState_t thread_state;

  /* Infinite loop */
  for(;;)
  {
	 HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, SET);

	#if IWDG_ACTIVE
	HAL_IWDG_Refresh(&hiwdg);
	#endif

	 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	 prev_hours = sTime.Hours;
	 prev_min = sTime.Minutes;
	 prev_sec = sTime.Seconds;

	 memset(buffer, 0, TX_BUF_DIM);
	 sprintf(buffer, "Time: %i:%i:%i	Date: \r\n %f\r\n", prev_hours,prev_min,prev_sec, altitude_m);
	 HAL_UART_Transmit(&DEBUG_UART, buffer, strlen(buffer), HAL_MAX_DELAY);


	  /*
	   * TODO Watch OUT: when writing to the external flash, don't have an interrupt that
	   * does it at the same time or it's a hardfault crash
	   *
	   * Moved the other code where we write to external flash to this thread (when going to sleep)
	   */
	  //Save the time
	  MRT_saveRTCTime();

	  //Check if it's sleep time
	  if (flagA==1){
		//Update iwdg_flag
		iwdg_flag = 1;
		flash_flags_buffer[IWDG_FLAG_OFFSET] = iwdg_flag;
		W25qxx_EraseSector(1);
		W25qxx_WriteSector(flash_flags_buffer, 1, FLAGS_OFFSET, NB_OF_FLAGS);

		//Reset to deactivate IWDG
		NVIC_SystemReset();
	  }


	  //Check each thread state
	  for (int i=0; i < NUMBER_OF_THREADS;i++){
		  thread_state = osThreadGetState(threadID[i]);

		  if (thread_state == osThreadInactive ||
		      thread_state == osThreadBlocked  ||
		      thread_state == osThreadTerminated){
			  uint8_t ejection_stage = 5; //TODO invented a random variable with a random value
			  if (i==1 && ejection_stage < 5){
				 osThreadResume(threadID[i]);
			  }
			  else if (i!=1){
				 osThreadResume(threadID[i]);
			  }
		  }

		  else if (thread_state == osThreadError){
			  NVIC_SystemReset();
		  }

		  /*
		  else if (thread_state == osThreadReady){
		  }
		  else if (thread_state == osThreadRunning){
		  }
		  else if (thread_state == osThreadReserved){ TODO not sure what is this state
		  }
		  */
	  }

	  HAL_GPIO_WritePin(OUT_LED2_GPIO_Port, OUT_LED2_Pin, RESET);

	  osDelay(1000/WD_FREQ);
  }
  /* USER CODE END StartWatchDog */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
