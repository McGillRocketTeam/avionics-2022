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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"

// sensors
#include "i2c_sensor_functions.h"
#include "gps.h"
#include "MAX31855.h" // thermocouple

// data storage
#include "sd_card.h"
#include "w25qxx.h" // external flash

// sradio
#include "sx126x.h"
#include "sx126x_regs.h"

// others
//#include <IridiumSBD_Static_API.h>
#include "video_recorder.h"
#include "ejection.h"
#include "MRT_RTOS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define DEBUG

// radios
#define USING_XTEND // comment out to use SRADio

// buzzer durations
#define BUZZ_SUCCESS_DURATION	5		// ms
#define BUZZ_SUCCESS_REPEATS	1

#define BUZZ_FAILURE_DURATION	500 	// ms
#define BUZZ_FAILURE_REPEATS	1

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

volatile uint8_t button_pressed = 0;
volatile uint8_t continuity = 0;
volatile uint8_t state = 0;

// i2c sensors
stmdev_ctx_t dev_ctx_lsm;
stmdev_ctx_t dev_ctx_lps;
volatile float acceleration_mg[] = {0, 0, 0};
volatile float angular_rate_mdps[]= {0, 0, 0};
volatile float pressure_hPa = 0;
volatile float temperature_degC = 0;
volatile float local_pressure = 0.0;

// gps data
volatile double latitude;
volatile double longitude;
volatile float time;
static uint8_t gps_fix_lat = 0;
static uint8_t gps_fix_long = 0; // beep when we get fix
extern char *rx_buffer_it;

// tank temperature (thermocouple)
volatile float tank_temperature = 0.0f;
volatile float tank_pressure = 0.0f;
volatile uint8_t valve_state = 0;

// rtc
RTC_TimeTypeDef stimeget = {0};
RTC_DateTypeDef sdateget = {0};

// sd card
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations
static uint8_t msg_buffer[200];
static uint8_t msg_buffer_av[200];
static uint8_t msg_buffer_pr[50];
static char filename[13]; // filename will be of form fc000000.txt which is 13 chars in the array (with null termination)
//const char sd_file_header[] = "S,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,PRESSURE,LAT,LONG,MIN,SEC,SUBSEC,STATE,CONT,E\r\n"; // printed to top of SD card file
const char sd_file_header[] = "pressure,altitude_raw,altitude_filtered,flight_state\r\n"; // printed to top of SD card file

volatile uint8_t curr_task = 0;

// external flash
extern w25qxx_t w25qxx;
uint8_t flash_write_buffer[256]; // page size is 256 bytes for w25qxx
uint8_t flash_read_buffer[256];
static volatile uint32_t flash_write_address = 0;

// ejection
float SmoothData = 0.0;
float alt_ground = 0.0;
float alt_filtered = 0.0;
uint8_t flight_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
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
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

// helpers
void tone(uint32_t duration, uint32_t repeats);

// flash and sd
int save_flash_to_sd(void);

uint8_t get_continuity();
float prop_poll_pressure_transducer(void);

float getAltitude(void);
void getAltitudeDataLog(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// radio transmission wrapper
// TODO: add reception
#ifdef USING_XTEND
void radio_tx(uint8_t *msg_buffer, uint16_t size) {
	HAL_UART_Transmit(&huart3, msg_buffer, size, HAL_MAX_DELAY);

	#ifdef DEBUG
	HAL_UART_Transmit(&huart8, msg_buffer, size, HAL_MAX_DELAY);
	#endif
}
#else // SRADio
void radio_tx(uint8_t *msg_buffer, uint16_t size) {
//	TxProtocol(msg_buffer, strlen(msg_buffer));
	TxProtocol(msg_buffer, size);

	#ifdef DEBUG
	HAL_UART_Transmit(&huart8, msg_buffer, size, HAL_MAX_DELAY);
	#endif
}
#endif

// helper functions for buzzing
void tone(uint32_t duration, uint32_t repeats) {
	for (uint32_t i = 0; i < repeats; i++) {
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_Delay(duration);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		if (repeats > 1)
			HAL_Delay(duration);
	}
}
void buzz_success() { tone(BUZZ_SUCCESS_DURATION, BUZZ_SUCCESS_REPEATS); };
void buzz_failure() { tone(BUZZ_FAILURE_DURATION, BUZZ_FAILURE_REPEATS); };

#ifdef DEBUG
void debug_tx_uart(uint8_t *msg_buffer) {
	HAL_UART_Transmit(&huart8, msg_buffer, strlen((char *)msg_buffer), HAL_MAX_DELAY);
}
#endif
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
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  MX_UART8_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // reset LEDs
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
  HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, RESET);

  // reset recovery pyro pins
  HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET);
  HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
  HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);

  // reset prop pyro pins
  HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, SET);
  HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
  HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);

  // reset 12 V buck converter enable pin (disable converter)
  HAL_GPIO_WritePin(PM_12V_EN_GPIO_Port, PM_12V_EN_Pin, RESET);
  HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET);

  // reset payload EN signal
  HAL_GPIO_WritePin(Payload_EN_GPIO_Port, Payload_EN_Pin, RESET);

  // set CS pin for thermocouple chip high (SPI idle CS is high)
  HAL_GPIO_WritePin(TH_CS_GPIO_Port, TH_CS_Pin, SET);

  // set power off for VR
  HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, RESET);
  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);

  // FLASH set CS, WP and IO3 pins high
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);
  HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, SET);
  HAL_GPIO_WritePin(FLASH_IO3_GPIO_Port, FLASH_IO3_Pin, SET);


#ifndef USING_XTEND
  set_hspi(hspi2);
  set_NSS_pin(SX_NSS_GPIO_Port, SX_NSS_Pin);
  set_BUSY_pin(SX_BUSY_GPIO_Port, SX_BUSY_Pin);
  set_NRESET_pin(SX_RST_GPIO_Port, SX_RST_Pin);
  set_DIO1_pin(SX_DIO_GPIO_Port, SX_DIO_Pin);
  Tx_setup();
#endif

#ifdef USING_RTC
  MRT_SetupRTOS(&hrtc, DEBUG_UART, SLEEP_TIME);
  MRT_setRTC(prev_hours,prev_min,prev_sec);
  HAL_Delay(2000); //To make sure that when you set the Alarm it doesn't go off automatically

#endif
#if ALARM_A_ACTIVE
  if (wu_flag == 0){
  	MRT_setAlarmA(PRE_WHEN_SLEEP_TIME_HOURS, PRE_WHEN_SLEEP_TIME_MIN, PRE_WHEN_SLEEP_TIME_SEC);
  }
  else{
  	MRT_setAlarmA(POST_WHEN_SLEEP_TIME_HOURS, POST_WHEN_SLEEP_TIME_MIN, POST_WHEN_SLEEP_TIME_SEC);
  }
#endif
  // init i2c sensors and data storage
  dev_ctx_lsm = lsm6dsl_init();
  buzz_success();
  HAL_Delay(500);

  dev_ctx_lps = lps22hh_init();
  buzz_success();
  HAL_Delay(500);

  // get local_pressure
  local_pressure = 1022.0;  // https://montreal.weatherstats.ca/charts/pressure_sea-hourly.html
  for (uint8_t i = 0; i < 100; i++) {
	  alt_ground += getAltitude();
  }

  alt_ground /= 100.0;

  // init FLASH
//  if (!W25qxx_Init()) Error_Handler();
  buzz_success();

  // init sd card with dynamic filename
  fres = sd_init_dynamic_filename("FC", sd_file_header, filename);
  if (fres != FR_OK) {
  		Error_Handler();
  }

//  int save_flash = save_flash_to_sd(); // check if flash empty and write to sd card if not
//  if (save_flash) {
//	  buzz_failure();
//  }

//  VR_Power_On();

  // init Iridium
//  MRT_Static_Iridium_Setup(huart3);
//  MRT_Static_Iridium_getIMEI();

  // send message with Iridium
//  MRT_Static_Iridium_sendMessage("message");
//  MRT_Static_Iridium_Shutdown();

  // init is done, can start timer 4 in interrupt mode for telemetry
  HAL_TIM_Base_Start_IT(&htim4);


//  sprintf(msg_buffer_av, "hello world!\n");

//  sprintf((char *)msg_buffer_av, "S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.7f,%03.7f,%02d,%02d,%lu,%d,%d,E\r\n",
//  						acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
//  						angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
//  						pressure_hPa, latitude, longitude,
//  						stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds,
//  						continuity, state);
//
//  sprintf((char *)msg_buffer_pr, "P,%03.2f,%03.2f,%d,%02d,%02d,%lu,E\r\n",
//  						tank_pressure, tank_temperature, valve_state,
//  						stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) // ejection code, only goes through loop once.
  {


	  /*// ejection
	  // initialize values
	  getAltitudeDataLog();

	  // wait for launch
	  while (alt_filtered < 50) {
		  getAltitudeDataLog();
		  HAL_Delay(5);
	  }

	  HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, SET);
	  flight_state = 1;

	  // save launched message to sd card
	  sprintf(msg_buffer_av, "launched, altitude_filtered = %.7f\r\n", alt_filtered);
	  fres = sd_open_file(filename);
	  sd_write(&fil, msg_buffer_av);
	  f_close(&fil);

	  // wait for apogee
	  uint8_t numNVals = 0;
	  float alt_prev = alt_filtered; // previous altitude in next while loop
	  float fittedSlope = 0;

	  while (1) {
		  getAltitudeDataLog();

		  if (alt_filtered > alt_prev) {
			  alt_prev = alt_filtered;
		  }
		  else {
			  fittedSlope = LSLinRegression();

			  if (fittedSlope < 0) {
				  numNVals += 1;
				  if (numNVals > NUM_DESCENDING_SAMPLES) {
					  break;
				  }
			  }
			  else {
				  numNVals = 0;
		  }

		  HAL_Delay(5);
	  }
	  flight_state = 2;
	  HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, SET); // apogee reached, drogue deployed

	  // save launched message to sd card
	  sprintf(msg_buffer_av, "apogee, altitude_filtered = %.7f\r\n", alt_filtered);
	  fres = sd_open_file(filename);
	  sd_write(&fil, msg_buffer_av);
	  f_close(&fil);

	  // wait for main
	  while (alt_filtered > MAIN_DEPLOYMENT) {
		  getAltitudeDataLog();
		  HAL_Delay(5);
	  }
	  HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
	  flight_state = 3;

	  // save launched message to sd card
	  sprintf(msg_buffer_av, "main chute, altitude_filtered = %.7f\r\n", alt_filtered);
	  fres = sd_open_file(filename);
	  sd_write(&fil, msg_buffer_av);
	  f_close(&fil);

	  // wait for landing
	  uint8_t count = 0;
	  while (count < LANDING_SAMPLES) {
		  getAltitudeDataLog();
		  if(alt_filtered - alt_previous[NUM_MEAS_REG-1] < LANDING_THRESHOLD)
			  count++;
		  else
			  count = 0;

		  HAL_Delay(5);
	  }
	  HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
	  flight_state = 4;

	  // save launched message to sd card
	  sprintf(msg_buffer_av, "landed, altitude_filtered = %.7f\r\n", alt_filtered);
	  fres = sd_open_file(filename);
	  sd_write(&fil, msg_buffer_av);
	  f_close(&fil);


	  while (1); // terminate
	  // -------------------------------------- //
	   */

//	  #ifdef DEBUG
//	  debug_tx_uart(msg_buffer); // data is in global variables
//	  	  HAL_Delay(100);
//	  #endif

//	   start/stop video
//	  VR_Start_Rec();
//	  HAL_Delay(1000000);
//	  VR_Stop_Rec();
//	  buzz_success();

	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
	  buzz_success();

	  // av message
	  sprintf((char *)msg_buffer_av, "S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.7f,%03.7f,%02d,%02d,%lu,%d,%d,E\r\n",
						acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
						angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
						pressure_hPa, latitude, longitude,
						stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds,
						continuity, state);
		HAL_UART_Transmit(&huart3, msg_buffer_av, strlen((char *)msg_buffer_av), HAL_MAX_DELAY);
//		HAL_Delay(1);


		// prop message
		sprintf((char *)msg_buffer_pr, "P,%03.2f,%03.2f,%d,%02d,%02d,%lu,E\r\n",
						tank_pressure, tank_temperature, valve_state,
						stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds);
		HAL_UART_Transmit(&huart3, msg_buffer_pr, strlen((char *)msg_buffer_pr), HAL_MAX_DELAY);
//		HAL_Delay(1); // 35 minimum

		// transmit via radio
//		radio_tx(msg_buffer, strlen(msg_buffer));
//		TxProtocol(msg_buffer, strlen(msg_buffer));

//		HAL_UART_Transmit(&huart3, msg_buffer_av, strlen((char *)msg_buffer_av), HAL_MAX_DELAY);
//		HAL_Delay(20);
//		memset(msg_buffer, 0, 100);

		// prop message
//		sprintf((char *)msg_buffer, "P,%03.2f,%03.2f,%d,%02d,%02d,%lu,E\r\n",
//						tank_pressure, tank_temperature, valve_state,
//						stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds);

		// transmit via radio (TODO: can be modified to send at different rate)
//		radio_tx(msg_buffer);
//		TxProtocol(msg_buffer, strlen(msg_buffer));
//		HAL_UART_Transmit(&huart3, msg_buffer_pr, strlen((char *)msg_buffer), HAL_MAX_DELAY);

		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		HAL_Delay(1);

//		memset(msg_buffer, 0, 100);

//		HAL_Delay(200);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sDate.Month = RTC_MONTH_FEBRUARY;
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
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400-1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOE, PM_12V_EN_Pin|Vent_Valve_EN_Pin|TH_CS_Pin|Iridium_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SD_CS_Pin|Prop_Gate_2_Pin|Prop_Gate_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin|SX_AMPLIFIER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Prop_Pyro_Arming_Pin|SX_RST_Pin|SX_BUSY_Pin|SX_DIO_Pin
                          |SX_RF_SW_Pin|VR_CTRL_PWR_Pin|Rcov_Gate_Main_Pin|Rcov_Gate_Drogue_Pin
                          |Rcov_Arm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SX_NSS_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, XTend_CTS_Pin|XTend_RTS_Pin|XTend_SLEEP_Pin|XTend_RX_LED_Pin
                          |XTend_TX_PWR_Pin|FLASH_IO3_Pin|FLASH_WP_Pin|FLASH_CS_Pin
                          |VR_CTRL_REC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PM_12V_EN_Pin Vent_Valve_EN_Pin TH_CS_Pin Iridium_RST_Pin */
  GPIO_InitStruct.Pin = PM_12V_EN_Pin|Vent_Valve_EN_Pin|TH_CS_Pin|Iridium_RST_Pin;
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

  /*Configure GPIO pin : IN_Button_Pin */
  GPIO_InitStruct.Pin = IN_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin SX_AMPLIFIER_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|SX_AMPLIFIER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDF_Pin */
  GPIO_InitStruct.Pin = LEDF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDF_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : Prop_Cont_1_Pin Rcov_Cont_Main_Pin Rcov_Cont_Drogue_Pin */
  GPIO_InitStruct.Pin = Prop_Cont_1_Pin|Rcov_Cont_Main_Pin|Rcov_Cont_Drogue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : Prop_Pyro_Arming_Pin SX_RST_Pin SX_BUSY_Pin SX_DIO_Pin
                           SX_RF_SW_Pin VR_CTRL_PWR_Pin Rcov_Gate_Main_Pin Rcov_Gate_Drogue_Pin
                           Rcov_Arm_Pin */
  GPIO_InitStruct.Pin = Prop_Pyro_Arming_Pin|SX_RST_Pin|SX_BUSY_Pin|SX_DIO_Pin
                          |SX_RF_SW_Pin|VR_CTRL_PWR_Pin|Rcov_Gate_Main_Pin|Rcov_Gate_Drogue_Pin
                          |Rcov_Arm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : Payload_EN_Pin IN_XTend_Continuity_Pin */
  GPIO_InitStruct.Pin = Payload_EN_Pin|IN_XTend_Continuity_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SX_NSS_Pin */
  GPIO_InitStruct.Pin = SX_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SX_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XTend_CTS_Pin XTend_RTS_Pin XTend_SLEEP_Pin XTend_RX_LED_Pin
                           XTend_TX_PWR_Pin FLASH_IO3_Pin FLASH_WP_Pin FLASH_CS_Pin
                           VR_CTRL_REC_Pin */
  GPIO_InitStruct.Pin = XTend_CTS_Pin|XTend_RTS_Pin|XTend_SLEEP_Pin|XTend_RX_LED_Pin
                          |XTend_TX_PWR_Pin|FLASH_IO3_Pin|FLASH_WP_Pin|FLASH_CS_Pin
                          |VR_CTRL_REC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SX_BANDPASS_FILTER_Pin */
  GPIO_InitStruct.Pin = SX_BANDPASS_FILTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SX_BANDPASS_FILTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_LPS22HH_DRDY_Pin EXTI_ISM330DCL_INT2_Pin EXTI_ISM330DLC_INT1_Pin */
  GPIO_InitStruct.Pin = EXTI_LPS22HH_DRDY_Pin|EXTI_ISM330DCL_INT2_Pin|EXTI_ISM330DLC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */


// just for vacuum chamber testing of the baro sensor
void getAltitudeDataLog() {
	// get value
	float raw_altitude = getAltitude();
	alt_filtered = runAltitudeMeasurements(HAL_GetTick(), raw_altitude);
	alt_filtered = raw_altitude;

	// save to sd
	sprintf(msg_buffer_av, "%.7f,%.7f,%.7f,%d\r\n", pressure_hPa, raw_altitude, alt_filtered, flight_state);
	fres = sd_open_file(filename);
	sd_write(&fil, msg_buffer_av);
	f_close(&fil);

#ifdef DEBUG
	HAL_UART_Transmit(&huart8, msg_buffer_av, strlen(msg_buffer_av), HAL_MAX_DELAY);
#endif
}

float getAltitude() {
	get_pressure(dev_ctx_lps, &pressure_hPa);
	uint32_t altitude = 145442.1609 * (1.0 - pow(pressure_hPa/local_pressure, 0.190266436));
	return altitude;
}

// timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {

		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		switch (curr_task)
		{
			case 1:
				// lsm6dsl data
				get_acceleration(dev_ctx_lsm, acceleration_mg);
				get_angvelocity(dev_ctx_lsm, angular_rate_mdps);
				break;

			case 2:
				// lps22hh data
				get_pressure(dev_ctx_lps, &pressure_hPa);
				get_temperature(dev_ctx_lps, &temperature_degC);
				break;

			case 3:
				// rtc data
				HAL_RTC_GetTime(&hrtc, &stimeget, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sdateget, RTC_FORMAT_BIN); // have to call GetDate for the time to be correct
				break;

			case 4:
				// continuity on pyro channels (one hot encoded)
				continuity = get_continuity();
				break;

			case 5:
				// propulsion data
				tank_temperature = Max31855_Read_Temp();
				tank_pressure = prop_poll_pressure_transducer();
				valve_state = HAL_GPIO_ReadPin(IN_Prop_ActuatedVent_Feedback_GPIO_Port, IN_Prop_ActuatedVent_Feedback_Pin);
				break;

			case 6:
				// gps
//				GPS_Poll(&latitude, &longitude, &time);

				break;

			case 7:
				// avionics message
//				sprintf((char *)msg_buffer_av, "S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.7f,%03.7f,%02d,%02d,%lu,%d,%d,E\r\n",
//								acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
//								angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
//								pressure_hPa, latitude, longitude,
//								stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds,
//								continuity, state);
//				HAL_UART_Transmit(&huart3, msg_buffer_av, strlen((char *)msg_buffer_av), HAL_MAX_DELAY);
//				memset(msg_buffer_av, 0, 200);
				break;

			case 8:
				// prop message
//				sprintf((char *)msg_buffer_pr, "P,%03.2f,%03.2f,%d,%02d,%02d,%lu,E\r\n",
//								tank_pressure, tank_temperature, valve_state,
//								stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds);
//				HAL_UART_Transmit(&huart3, msg_buffer_pr, strlen((char *)msg_buffer_pr), HAL_MAX_DELAY);
//				memset(msg_buffer_pr, 0, 50);
				break;

			case 9:
				// save to sd
				fres = sd_open_file(filename);
				sd_write(&fil, msg_buffer_av);
				sd_write(&fil, msg_buffer_pr);
				f_close(&fil);

				break;

//			case 10:
////				 radio send avionics
//				HAL_UART_Transmit(&huart3, msg_buffer_av, strlen((char *)msg_buffer_av), HAL_MAX_DELAY);
//				break;
//
//			case 11:
////				 radio send avionics
//				HAL_UART_Transmit(&huart3, msg_buffer_pr, strlen((char *)msg_buffer_pr), HAL_MAX_DELAY);
//				break;

			default:
				curr_task = 0;
		}
		curr_task++;

		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);

		// TODO: save to FLASH
		// calculate page address and offset based on number of bytes already written
//		uint32_t page_address = (int) (flash_write_address / w25qxx.BlockSize);
//		uint32_t page_offset = (int) (flash_write_address % w25qxx.BlockSize);

//		W25qxx_WriteBlock(msg_buffer, page_address, page_offset, strlen((const char *)msg_buffer));
//		flash_write_address += strlen((const char *)msg_buffer);

	}
}

//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart->Instance == USART6) {
//
//		// mcu has received data from GPS into buffer, time to parse buffer
//		GPS_ParseBuffer_IT(&latitude, &longitude, &time);
//		HAL_UART_Transmit(&huart8, rx_buffer_it, 150, HAL_MAX_DELAY);
//
//	}
//}

/**
 * reads data out of external FLASH and saves to SD card.
 * erases the used flash after finished.
 *
 * assumes f_mount has already been run.
 * this function does not close the file system.
 * opens a file "datalog.txt" and closes it when finished.
 */
int save_flash_to_sd(void) {
	// FLASH variables
	uint32_t page_num = 0;
	uint16_t page_bytes = w25qxx.PageSize; // 256 bytes saved per page
	uint8_t readBuf[page_bytes];

	// write to file
	fres = f_open(&fil, "flashlog.txt", FA_WRITE | FA_OPEN_ALWAYS);
	if (fres != FR_OK) {
		myprintf("f_open error (%i)\r\n", fres);
		return -1;
	}

	// set pointer to end of file
	f_lseek(&fil, f_size(&fil));

	// print string to indicate new log session
	sprintf((char *)msg_buffer, "\n--- new logging session! ---\r\n");
	sd_write(&fil, msg_buffer);

	for (page_num = 0; page_num < w25qxx.PageCount; page_num++) {

		if (!W25qxx_IsEmptyPage(page_num, 0, page_bytes)) {

			// page not empty, read page out of flash
			W25qxx_ReadPage(readBuf, page_num, 0, page_bytes);

			// save to SD
			int8_t status = sd_write(&fil, readBuf);
			if (status <= 0) {
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET); // failed
			}
		}
		else break; // page empty, no need to continue
	}

	// close file
	f_close(&fil);

	if (page_num == 0) { // nothing saved
		tone(50, 5);
	}
	else {
		// wait for button press to erase
		tone(250, 3);

		// clear the blocks with data
		uint32_t blocks_to_clear = W25qxx_PageToBlock(page_num);
		for (uint32_t block = 0; block <= blocks_to_clear; block++) {
			W25qxx_EraseBlock(block);
		}

		tone(50, 3);
	}

	return 0;

}


uint8_t get_continuity() {

	// read pins
	GPIO_PinState drogue = HAL_GPIO_ReadPin(Rcov_Cont_Drogue_GPIO_Port, Rcov_Cont_Drogue_Pin);
	GPIO_PinState main = HAL_GPIO_ReadPin(Rcov_Cont_Main_GPIO_Port, Rcov_Cont_Main_Pin);
	GPIO_PinState prop_1 = HAL_GPIO_ReadPin(Prop_Cont_1_GPIO_Port, Prop_Cont_1_Pin);
	GPIO_PinState prop_2 = HAL_GPIO_ReadPin(Prop_Cont_2_GPIO_Port, Prop_Cont_2_Pin);

	// assign one-hot encoded result (apparently you can multiply enums?)
	uint8_t continuity = (drogue) + (main * 2) + (prop_1 * 4) + (prop_2 * 8);

	return continuity;

}

float prop_poll_pressure_transducer(void) {

	// reading adc
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	uint32_t pressure_sensor_raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	float voltage = (float) (pressure_sensor_raw / 4095.0); // assuming 12 bits

	// convert using transfer function
	// TODO

	return voltage;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	buzz_failure();
	__BKPT();
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
