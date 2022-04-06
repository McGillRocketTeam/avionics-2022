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
#include "adc.h"
#include "dma.h"
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
#include "radio_commands.h"
#include "MRT_setup.h"
#include "MRT_RTOS.h"
#include "helpers.h"
#include "state_restoration.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define DEBUG_MODE

// radios
#define USING_XTEND 	// comment out to use SRADio
//#define TIMING_ITM 		// comment out

#ifdef TIMING_ITM
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
#define TIMING_ITM_LOOPS	500
#endif

#define PROP_TANK_PRESSURE_ADC_BUF_LEN	15	// samples
#define USING_RTC

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint8_t button_pressed = 0;
volatile uint8_t continuity = 0;

// i2c sensors
stmdev_ctx_t dev_ctx_lsm;
stmdev_ctx_t dev_ctx_lps;
float acceleration_mg[] = {0, 0, 0};
float angular_rate_mdps[]= {0, 0, 0};
float pressure_hPa = 0;
float temperature_degC = 0;

// gps data
double latitude;
double longitude;
float time;
volatile char gps_rx_buf[GPS_RX_DMA_BUF_LEN+1]; // +1 for manual insertion of '\0'. make sure to only request GPS_RX_DMA_BUF_LEN in DMA call
volatile uint8_t gps_dma_ready = 0;

// propulsion
volatile float tank_temperature = 0.0f;
uint8_t valve_state = 0;
volatile float tank_pressure = 0.0f;
volatile uint16_t tank_pressure_buf[PROP_TANK_PRESSURE_ADC_BUF_LEN]; // circular buffer for averaging (low pass filter)
volatile uint8_t tank_pressure_buf_idx = 0;

// rtc
RTC_TimeTypeDef stimeget = {0};
RTC_DateTypeDef sdateget = {0};
volatile uint8_t sleepmode = 0;

// sd card
FATFS FatFs; 	// Fatfs handle
FIL fil; 		// File handle
FRESULT fres; 	// Result after operations
static uint8_t msg_buffer_av[200];
static uint8_t msg_buffer_pr[50];
volatile uint8_t msg[1000];
static char filename[13]; // filename will be of form fc000000.txt which is 13 chars in the array (with null termination)
const char sd_file_header[] = "S,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,PRESSURE,LAT,LONG,MIN,SEC,SUBSEC,STATE,CONT,E\r\n"; // printed to top of SD card file

// external flash
extern w25qxx_t w25qxx;
volatile uint32_t flash_write_address = 0;

// state of flight (for changing radio transmission rate)
volatile uint8_t state = FLIGHT_STATE_PAD;
volatile uint8_t num_radio_transmissions = 0;
volatile uint8_t state_arm_rcov = 0;
volatile uint8_t state_arm_prop = 0;

// tracking altitude for state of flight changing
float alt_ground = 0;
float alt_current = 0;
float alt_prev = 0;
float alt_diff = 0;
float alt_apogee = 0;
float fitted_slope = 0;
uint8_t num_descending_samples = 0; // for crude apogee detection

// bidirectional xtend communication
volatile char xtend_rx_buf[10];
volatile uint8_t xtend_rx_dma_ready = 0;
volatile uint8_t xtend_tx_completed = 1; // to allow first tx

volatile uint8_t xtend_tx_start_av = 0;	 // distinguishing tx of av and pr in callback with timer
volatile uint8_t xtend_tx_start_pr = 0;

const char xtend_ack_msg[] = "xtend_ack\r\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// flash
int flash_write(char *msg_buffer);

uint8_t get_continuity(void);
void prop_poll_pressure_transducer(void);
float convert_prop_tank_pressure(void);
float getAltitude(void);

uint8_t xtend_parse_dma_command(void);
void check_flight_state(volatile uint8_t *state);
void xtend_transmit_telemetry(volatile uint8_t *state);
void update_radio_timer_params(volatile uint8_t *state);

void telemetry_format_avionics(void);
void telemetry_format_propulsion(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#ifdef DEBUG_MODE
void debug_tx_uart(char *msg) {
	HAL_UART_Transmit(&huart8, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}
//#endif

// radio transmission wrapper
#ifdef USING_XTEND
// reception using UART DMA
void radio_tx(uint8_t *msg_buffer, uint16_t size) {
//	HAL_UART_Transmit(&huart3, msg_buffer, size, HAL_MAX_DELAY);
	HAL_UART_Transmit_DMA(&huart3, msg_buffer, size);

	#ifdef DEBUG_MODE
	debug_tx_uart(msg_buffer);
	#endif
}
#else // SRADio
void radio_tx(uint8_t *msg_buffer, uint16_t size) {
	TxProtocol(msg_buffer, size);

	#ifdef DEBUG_MODE
	debug_tx_uart(msg_buffer);
	#endif
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

	// just for debugging adc resolution with averaging
	for (uint8_t i = 0; i < PROP_TANK_PRESSURE_ADC_BUF_LEN; i++) {
		tank_pressure_buf[i] = 1940;
	}

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_DBGMCU_FREEZE_IWDG();	// turn off IWDG and RTC during debugging
  __HAL_FREEZE_RTC_DBGMCU();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
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
  MX_TIM3_Init();
  MX_TIM8_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // *** IMPORTANT: DMA Init function must be called before peripheral init! *** //

  if (!(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))) {

	  // turn on LED near vent hole to show that FC is on
	  HAL_GPIO_WritePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin, SET);

	  // FLASH set CS, WP and IO3 pins high
	  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);
	  HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, SET);
	  HAL_GPIO_WritePin(FLASH_IO3_GPIO_Port, FLASH_IO3_Pin, SET);

	  // set other SPI CS pins high
	  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, SET);
	  HAL_GPIO_WritePin(TH_CS_GPIO_Port, TH_CS_Pin, SET);

	  // reset LEDs
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
	  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
	  HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, RESET);

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

	  // set power off for VR
	  HAL_GPIO_WritePin(VR_CTRL_PWR_GPIO_Port, VR_CTRL_PWR_Pin, RESET);
	  HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);

	#ifndef USING_XTEND
	  set_hspi(hspi2);
	  set_NSS_pin(SX_NSS_GPIO_Port, SX_NSS_Pin);
	  set_BUSY_pin(SX_BUSY_GPIO_Port, SX_BUSY_Pin);
	  set_NRESET_pin(SX_RST_GPIO_Port, SX_RST_Pin);
	  set_DIO1_pin(SX_DIO_GPIO_Port, SX_DIO_Pin);
	  Tx_setup();
	#endif

	  init_backup_regs();
  } // if IWDG reset flag high
  else {
	  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
	  restore_fc_states();
	  sprintf(msg, "state = %d, ap = %d, ar = %d, alt_ground = %f",
			  state, state_arm_prop, state_arm_rcov, alt_ground);
	  debug_tx_uart(msg);

	  // for debug
	  for (uint8_t i = 0; i < 3; i++) {
		  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
		  HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, SET);

		  HAL_Delay(100);

		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		  HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, RESET);
	  }
  }

  // init i2c sensors and data storage
  dev_ctx_lsm = lsm6dsl_init();
  dev_ctx_lps = lps22hh_init();

  // init FLASH
//  if (!W25qxx_Init()) Error_Handler();
//  buzz_success();

  // check if SD card is inserted
  if (HAL_GPIO_ReadPin(IN_SD_CARD_DETECT_GPIO_Port, IN_SD_CARD_DETECT_Pin) == GPIO_PIN_RESET) {
	  // init sd card with dynamic filename
	  fres = sd_init_dynamic_filename("FC", sd_file_header, filename);
	  if (fres != FR_OK) {
			Error_Handler();
	  }
  }
  else {
	  Error_Handler();
  }

  // check if flash empty and write to sd card if not
//  int save_flash = save_flash_to_sd();
//  if (save_flash) {
//	  buzz_failure();
//  }

  if (!(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))) {
	  // got to this point, successful init
	  buzz_startup_success();
  }

  // get ground altitude
  for (uint8_t i = 0; i < 100; i++) {
	  alt_ground += getAltitude();
  }
  alt_ground /= 100.0;
  set_backup_state(FC_STATE_ALT_GROUND, (uint32_t) alt_ground);
  alt_current = alt_ground;

  // initial DMA requests:
//  HAL_UART_Receive_DMA(&huart6, gps_rx_buf, GPS_RX_DMA_BUF_LEN); // GPS
  memset(xtend_rx_buf, 0, 10);
//  HAL_UART_Receive_DMA(&huart3, (uint8_t *)xtend_rx_buf, XTEND_RX_DMA_CMD_LEN); // XTend

  // initialize avionics and propulsion xtend buffers with *something* so DMA can happen without zero length error
  telemetry_format_avionics();
  telemetry_format_propulsion();

  // start timers:
//  HAL_TIM_Base_Start_IT(&htim3);	// drives XTend DMA
  HAL_TIM_Base_Start_IT(&htim8);	// drives ADC

  // start watchdog
  MX_IWDG_Init();

  // clear RCC reset flags
  __HAL_RCC_CLEAR_RESET_FLAGS();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#ifndef TIMING_ITM
  while (1)
#else
  ITM_Port32(31) = 1;
  for (uint32_t i = 2; i < 2+TIMING_ITM_LOOPS; i++)
#endif
  {
//	    buzz_success();
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		// -----  GATHER TELEMETRY ----- //
		get_acceleration(dev_ctx_lsm, acceleration_mg);
		get_angvelocity(dev_ctx_lsm, angular_rate_mdps);
		alt_current = runAltitudeMeasurements(HAL_GetTick(), getAltitude());

		HAL_RTC_GetTime(&hrtc, &stimeget, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sdateget, RTC_FORMAT_BIN); // have to call GetDate for the time to be correct

		continuity = get_continuity();
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg); // refresh watchdog

		// gps
		if (gps_dma_ready) {
			gps_dma_ready = 0;
			GPS_ParseBuffer(&latitude, &longitude, &time);

			// start new DMA request
			HAL_UART_Receive_DMA(&huart6, gps_rx_buf, GPS_RX_DMA_BUF_LEN);
		}

		// propulsion data (not needed after apogee)
		if (state < FLIGHT_STATE_PRE_MAIN) {
			tank_temperature = Max31855_Read_Temp();
			valve_state = HAL_GPIO_ReadPin(IN_Prop_ActuatedVent_Feedback_GPIO_Port, IN_Prop_ActuatedVent_Feedback_Pin);

			#ifdef TIMING_ITM
				ITM_Port32(31) = 300; // start of pr sprintf
			#endif

			telemetry_format_propulsion();

			#ifdef TIMING_ITM
				ITM_Port32(31) = 301; // end of pr sprintf
			#endif
		}

		// -----  FORMATTING TELEMETRY ----- //

		__HAL_IWDG_RELOAD_COUNTER(&hiwdg); // refresh watchdog

		// avionics message
		#ifdef TIMING_ITM
			ITM_Port32(31) = 400; // start of av sprintf
		#endif

		telemetry_format_avionics();

		#ifdef TIMING_ITM
			ITM_Port32(31) = 401; // end of av sprintf
		#endif

		// save to sd and flash
		fres = sd_open_file(filename);
		sd_write(&fil, msg_buffer_av);
		if (state < FLIGHT_STATE_PRE_MAIN) {
			sd_write(&fil, msg_buffer_pr);
		}
		f_close(&fil);

		#ifdef TIMING_ITM
			ITM_Port32(31) = 500; // end of SD
		#endif

//		flash_write((char *)msg_buffer_av);
//		flash_write((char *)msg_buffer_pr);

//		if (loopcount == 20) {
//			while (1);
//		}
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg); // refresh watchdog

	  	#ifdef DEBUG_MODE
//			debug_tx_uart(msg_buffer_av);
//			debug_tx_uart(msg_buffer_pr);
			sprintf(msg, "states of fc: s=%d, ap=%d, ar=%d, HH:MM:SS = %02d:%02d:%02d\r\n",
				   state, state_arm_prop, state_arm_rcov,
				   stimeget.Hours, stimeget.Minutes, stimeget.Seconds);
			debug_tx_uart(msg);
		#endif

		// check which state of flight we are in
		check_flight_state(&state);

		// reload watchdog counter
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);

		if (sleepmode) {
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			HAL_PWR_EnableSleepOnExit();
			HAL_SuspendTick();
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	   }


//		loopcount++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	#ifdef TIMING_ITM
	  ITM_Port32(31) = i;
	#endif
  }

#ifdef TIMING_ITM
  HAL_TIM_Base_Stop(&htim3);
  __BKPT();
  while (1); // prevent hardfault if main() exits
#endif
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

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

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == IN_Button_Pin) {
		button_pressed = 1;
		state += 1;
		if (state == 2) {
			state = 0;
		}
		set_backup_state(FC_STATE_FLIGHT, (uint32_t) state);

		state_arm_prop++;
		state_arm_rcov++;

		set_backup_state(FC_STATE_ARM_PROP, (uint32_t) state_arm_prop);
		set_backup_state(FC_STATE_ARM_RCOV, (uint32_t) state_arm_rcov);

		sprintf(msg, "state = %d, ap = %d, ar = %d, alt_ground = %f",
					  state, state_arm_prop, state_arm_rcov, alt_ground);
		debug_tx_uart(msg);

		while (1); // trigger watchdog during testing

		__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
	}
	else if (GPIO_Pin == EXTI_SWIER_SWIER4) { // software interrupt to change timer settings
		update_radio_timer_params(&state);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart6) { // gps
		// insert null termination and indicate buffer is ready to parse
		// (total buffer length is GPS_RX_DMA_BUF_LEN + 1)
		gps_rx_buf[GPS_RX_DMA_BUF_LEN] = '\0';
		gps_dma_ready = 1;
	}
	else if (huart == &huart3) { // xtend radio
		// go check what the command is
		radio_command cmd = xtend_parse_dma_command();
		execute_parsed_command(cmd); // ASSUMING VR IS CONTROLLED BY ATTINY AND VR COMMANDS ARE NON BLOCKING

		// send acknowledge
		HAL_UART_DMAStop(&huart3);
		xtend_tx_start_av = 0;	// set flags to zero to allow new tx
		xtend_tx_start_pr = 0;
		sprintf(msg_buffer_av, "%s_%d", (int)cmd);
		radio_tx(msg_buffer_av, strlen(msg_buffer_av));

		// prep for next command to be sent
		memset(xtend_rx_buf, 0, 10);
		HAL_UART_Receive_DMA(&huart3, xtend_rx_buf, XTEND_RX_DMA_CMD_LEN);
		xtend_rx_dma_ready = 0;

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart3) {
//		#ifdef TIMING_ITM
//			ITM_Port32(31) = 200;
//		#endif
		// don't care who started it, transmit is complete
		xtend_tx_start_av = 0;
		xtend_tx_start_pr = 0;

		num_radio_transmissions++;
		if (num_radio_transmissions == 10) {
			num_radio_transmissions = 0;
		}
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

	HAL_RTCEx_DeactivateWakeUpTimer(hrtc);

	sprintf((char*) msg, "Alarm A callback entered\r\n");
	debug_tx_uart(msg);

	sprintf((char*) msg, "alarmA flag: %d\talarmB flag: %d\r\n\n",
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF),
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF));
	debug_tx_uart(msg);

	// clear the alarm flag
	__HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
	while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF) != RESET) {
		__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);
		__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRBF);
		__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
	}
	__HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

	// start the timer (resets the count too)
	HAL_RTCEx_SetWakeUpTimer_IT(hrtc, 2000-1, RTC_WAKEUPCLOCK_RTCCLK_DIV16); // start the timer

	sprintf((char*) msg, "alarmA flag after clear: %d\talarmB flag: %d\r\n\n",
				__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF),
				__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF));
	debug_tx_uart(msg);

	sleepmode = 1;
}

void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc) {
	// wake up by alarm B, re-init clocks and resume tick
	SystemClock_Config();
	HAL_ResumeTick();

	HAL_PWR_DisableSleepOnExit();
	HAL_RTCEx_DeactivateWakeUpTimer(hrtc);

	sprintf((char *)msg, "Alarm B callback entered\r\n");
	debug_tx_uart(msg);

	sprintf((char*) msg, "before clear attempt: alarmA flag: %d\talarmB flag: %d\r\n\n",
			__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF), __HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF));
	debug_tx_uart(msg);

	sprintf((char*) msg, "after clear attempt: alarmA flag: %d\talarmB flag: %d\r\n\n", __HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF), __HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBF));
    debug_tx_uart(msg);

    sleepmode = 0;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
	if (sleepmode && __HAL_PWR_GET_FLAG(PWR_FLAG_WU)) {
		SystemClock_Config(); // woke from stop mode, re-init clocks
		HAL_ResumeTick();
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg); // refresh watchdog

		sprintf(msg, "wakeup timer callback\r\n");
		debug_tx_uart(msg);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
//		#ifdef TIMING_ITM
//			ITM_Port32(31) = 100;
//		#endif

		HAL_GPIO_TogglePin(LEDF_GPIO_Port, LEDF_Pin);
		xtend_transmit_telemetry(&state);
	}
	else if (htim == &htim8) {

#ifdef TIMING_ITM
		ITM_Port32(31) = 20;
#endif
		prop_poll_pressure_transducer();

#ifdef TIMING_ITM
		ITM_Port32(31) = 21;
#endif
#ifdef DEBUG_MODE
		tank_pressure = convert_prop_tank_pressure(); // for debug, later move to telemetry_format_prop()
#endif

#ifdef TIMING_ITM
		ITM_Port32(31) = 22;
#endif
	}
}

float getAltitude(void) {
	get_pressure(dev_ctx_lps, &pressure_hPa);
	uint32_t altitude = 145442.1609 * (1.0 - pow(pressure_hPa/LOCAL_PRESSURE_HPA, 0.190266436));
	return altitude;
}

int flash_write(char *msg_buffer) {
	// calculate page address and offset based on number of bytes already written
	uint32_t block_address = (uint32_t) (flash_write_address / w25qxx.BlockSize);
	uint32_t block_offset = (uint32_t) (flash_write_address % w25qxx.BlockSize);

	W25qxx_WriteBlock(msg_buffer, block_address, block_offset, strlen((const char *)msg_buffer));
	flash_write_address += strlen((const char *)msg_buffer);
	return flash_write_address;
}

uint8_t get_continuity(void) {
	// read pins
	GPIO_PinState drogue = HAL_GPIO_ReadPin(Rcov_Cont_Drogue_GPIO_Port, Rcov_Cont_Drogue_Pin);
	GPIO_PinState main = HAL_GPIO_ReadPin(Rcov_Cont_Main_GPIO_Port, Rcov_Cont_Main_Pin);
	GPIO_PinState prop_1 = HAL_GPIO_ReadPin(Prop_Cont_1_GPIO_Port, Prop_Cont_1_Pin);
	GPIO_PinState prop_2 = HAL_GPIO_ReadPin(Prop_Cont_2_GPIO_Port, Prop_Cont_2_Pin);

	uint8_t continuity = (drogue) + (main * 2) + (prop_1 * 4) + (prop_2 * 8);
	return continuity;
}

// polling ADC for pressure transducer voltage
void prop_poll_pressure_transducer(void) {
	// reading adc
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	uint32_t pressure_sensor_raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// store to circular buffer
	tank_pressure_buf[tank_pressure_buf_idx++] = pressure_sensor_raw; // convert to float later
	if (tank_pressure_buf_idx == PROP_TANK_PRESSURE_ADC_BUF_LEN)
		tank_pressure_buf_idx = 0;
}

// supposed to be used for ADC DMA callback to average
// the values in the buffer and convert to pressure.
// could consider sending a raw voltage because transfer functions
// can get screwed up in code but raw voltage won't?
float convert_prop_tank_pressure(void) {
	// average values in the buffer
	uint32_t avg = 0;
	for (uint16_t i = 0; i < PROP_TANK_PRESSURE_ADC_BUF_LEN; i++) {
		avg += tank_pressure_buf[i];
	}

	float pressure = ((float) avg) / PROP_TANK_PRESSURE_ADC_BUF_LEN / 4095.0 * 3.3; // 12 bit ADC
	return pressure;
}

// logic to change states of flight
void check_flight_state(volatile uint8_t *state) {
	switch (*state) {
	case FLIGHT_STATE_PAD: // launch pad, waiting. prioritize prop data

		// check current state
		if (alt_current > LAUNCH_ALT_CHANGE_THRESHOLD) { // launched
			*state = FLIGHT_STATE_PRE_APOGEE;

			fres = sd_open_file(filename);
			sd_write(&fil, (uint8_t *)"launched\r\n");
			f_close(&fil);

			#ifdef DEBUG_MODE
				HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, SET);
			#endif

			// generate software interrupt to change TIM3 update rate
			__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
		}

		break;

	case FLIGHT_STATE_PRE_APOGEE: // pre-apogee, waiting for ejection and drogue deployment

		fitted_slope = LSLinRegression();
		if (fitted_slope < 0) {
			num_descending_samples += 1;

			if (num_descending_samples > APOGEE_NUM_DESCENDING_SAMPLES) {
				// *** EJECTION AND DROGUE DEPLOYMENT *** //
				HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET); // can't hurt right? in case arming failed on the pad
				HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
				HAL_Delay(DROGUE_DELAY);
				HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
				// *** ------------------------------ *** //

				*state = FLIGHT_STATE_PRE_MAIN; // passed apogee
				num_descending_samples = 0;

				fres = sd_open_file(filename);
				sd_write(&fil, (uint8_t *)"apogee\r\n");
				f_close(&fil);

				#ifdef DEBUG_MODE
					HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, SET);
				#endif

				// generate software interrupt to change TIM3 update rate
				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		}
		else {
			num_descending_samples = 0;
		}

		break;

	case FLIGHT_STATE_PRE_MAIN: // post-apogee, waiting for main parachute deployment

		// check current state
		if (alt_current < MAIN_DEPLOY_ALTITUDE) {
			num_descending_samples++;

			if (num_descending_samples > MAIN_NUM_DESCENDING_SAMPLES) {
				// *** DEPLOYING MAIN PARACHUTE *** //
				HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET); // can't hurt right? in case arming failed on the pad
				HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
				HAL_Delay(MAIN_DELAY);
				HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);
				// *** ------------------------ *** //

				*state = FLIGHT_STATE_PRE_LANDED;
				alt_prev = alt_current; // in next stage we need to know the previous altitude
				num_descending_samples = 0;

				fres = sd_open_file(filename);
				sd_write(&fil, (uint8_t *)"main deployed\r\n");
				f_close(&fil);

				#ifdef DEBUG_MODE
					HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
				#endif

				// generate software interrupt to change TIM3 update rate
				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		} else {
			num_descending_samples = 0;
		}

		break;

	case FLIGHT_STATE_PRE_LANDED:
		// post main deploy, want to transmit data fast to maximize possibility of getting good GPS coordinates

		// check current state
		alt_diff = alt_current - alt_prev;
		if (alt_diff < 0) {
			alt_diff *= -1; // absolute value
		}

		if (alt_diff < LANDING_ALT_CHANGE_THRESHOLD) {
			num_descending_samples++;

			if (num_descending_samples > LANDING_NUM_DESCENDING_SAMPLES) {
				*state = FLIGHT_STATE_LANDED;
				num_descending_samples = 0;

				fres = sd_open_file(filename);
				sd_write(&fil, (uint8_t *)"landed\r\n");
				f_close(&fil);

				#ifdef DEBUG_MODE
					HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
				#endif

				// generate software interrupt to change TIM3 update rate
				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		} else {
			num_descending_samples = 0;
		}

		alt_prev = alt_current;
		break;

	case FLIGHT_STATE_LANDED: // landed
//		__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);

		// stop video recorder
		VR_Stop_Rec();
		set_backup_state(FC_STATE_VR_RECORDING, 0);
		HAL_Delay(1000);
		VR_Power_Off();
		set_backup_state(FC_STATE_VR_POWER, 0);

		#ifdef DEBUG_MODE
			while (1) {
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
				HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, SET);
			}
		#endif

		break;

	default:

		break;
	}
}

// sends an avionics or propulsion string depending on the situation
void xtend_transmit_telemetry(volatile uint8_t *state) {
	switch (*state) {
	case FLIGHT_STATE_PAD:

		// send av
		if (num_radio_transmissions % 2 == 0 && xtend_tx_start_pr == 0 && xtend_tx_start_av == 0) {
			radio_tx(msg_buffer_av, strlen((char *)msg_buffer_av));
			xtend_tx_start_av = 1;
		}

		// send prop
		else if (xtend_tx_start_av == 0 && xtend_tx_start_pr == 0) {
			radio_tx(msg_buffer_pr, strlen((char *)msg_buffer_pr));
			xtend_tx_start_pr = 1;
		}
		break;

	case FLIGHT_STATE_PRE_APOGEE:
		// transmit avionics and prop at equal priority
		if (xtend_tx_start_pr == 0 && xtend_tx_start_av == 0 && num_radio_transmissions % 2 == 0) {
			radio_tx(msg_buffer_av, strlen((char *)msg_buffer_av));
		}
		else if (xtend_tx_start_pr == 0 && xtend_tx_start_av == 0 && num_radio_transmissions % 2 == 1) {
			radio_tx(msg_buffer_pr, strlen((char *)msg_buffer_pr));
		}
		break;

	default:
		if (xtend_tx_start_av == 0) {
			radio_tx(msg_buffer_av, strlen((char *)msg_buffer_av));
		}
		break;
	}
}

// updates settings for TIM3 depending on the state of the flight.
// TIM3 controls the rate of XTend radio transmission
void update_radio_timer_params(volatile uint8_t *state) {
	switch (*state) {
	// assuming clock freq = 90 MHz, PSC = 9000-1, use ARR to get desired counter freq
	// 		10 Hz -> ARR = 1000
	// 		 5 Hz -> ARR = 2000
	// 		 2 Hz -> ARR = 5000
	// 		 1 Hz -> ARR = 10000

	case FLIGHT_STATE_PAD:
		TIM3->ARR = 1000-1;
		TIM3->EGR |= TIM_EGR_UG;
//		debug_tx_uart("10hz\r\n");
		break;

	case FLIGHT_STATE_PRE_APOGEE:
		TIM3->ARR = 5000-1;
		TIM3->EGR |= TIM_EGR_UG;
//		debug_tx_uart("02hz\r\n");
		break;

	case FLIGHT_STATE_PRE_MAIN:
		TIM3->ARR = 2000-1;
		TIM3->EGR |= TIM_EGR_UG;
//		debug_tx_uart("05hz\r\n");
		break;

	case FLIGHT_STATE_PRE_LANDED:
		TIM3->ARR = 1000-1;
		TIM3->EGR |= TIM_EGR_UG;
//		debug_tx_uart("10hz\r\n");
		break;

	case FLIGHT_STATE_LANDED:
		TIM3->ARR = 20000-1;
		TIM3->EGR |= TIM_EGR_UG;
//		debug_tx_uart("01hz\r\n");
		break;

	default:
		TIM3->ARR = 1000-1;
		TIM3->EGR |= TIM_EGR_UG;
//		debug_tx_uart("10hz\r\n");
		state = 0;
		break;
	}
}

// formats avionics telemetry string using sprintf
void telemetry_format_avionics(void) {
	sprintf((char*) msg_buffer_av,
			"S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.7f,%03.7f,%02d,%02d,%lu,%d,%d,E\r\n",
			acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
			angular_rate_mdps[0], angular_rate_mdps[1],
			angular_rate_mdps[2], pressure_hPa, latitude, longitude,
			stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds,
			continuity, state);
}

// formats propulsion telemetry string using sprintf
void telemetry_format_propulsion(void) {
	tank_pressure = convert_prop_tank_pressure(); // convert buffered readings to voltage
	sprintf((char*) msg_buffer_pr, "P,%03.2f,%03.2f,%d,%02d,%02d,%lu,E\r\n",
			tank_pressure, tank_temperature, valve_state, stimeget.Minutes,
			stimeget.Seconds, stimeget.SubSeconds);
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
	HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, GPIO_PIN_SET);
	while (1) {
		buzz_failure();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
