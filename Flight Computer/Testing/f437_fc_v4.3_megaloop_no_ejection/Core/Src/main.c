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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define DEBUG

// radios
#define USING_XTEND // comment out to use SRADio

// buzzer durations
#define BUZZ_SUCCESS_DURATION	50		// ms
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
float local_pressure = 1028.0;

// gps data
double latitude;
double longitude;
float time;
volatile char gps_rx_buf[GPS_RX_DMA_BUF_LEN+1]; // +1 for manual insertion of '\0'. make sure to only request GPS_RX_DMA_BUF_LEN in DMA call
volatile char gps_parsed[100];
volatile uint8_t gps_dma_ready = 0;

// tank temperature (thermocouple)
float tank_temperature = 0.0f;
float tank_pressure = 0.0f;
uint8_t valve_state = 0;

// rtc
RTC_TimeTypeDef stimeget = {0};
RTC_DateTypeDef sdateget = {0};

// sd card
FATFS FatFs; 	// Fatfs handle
FIL fil; 		// File handle
FRESULT fres; 	// Result after operations
static uint8_t msg_buffer_av[200];
static uint8_t msg_buffer_pr[50];
static char filename[13]; // filename will be of form fc000000.txt which is 13 chars in the array (with null termination)
const char sd_file_header[] = "S,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,PRESSURE,LAT,LONG,MIN,SEC,SUBSEC,STATE,CONT,E\r\n"; // printed to top of SD card file

// external flash
extern w25qxx_t w25qxx;
uint8_t flash_write_buffer[256]; // page size is 256 bytes for w25qxx
uint8_t flash_read_buffer[256];
static uint32_t flash_write_address = 0;

// state of flight (for changing radio transmission rate)
volatile uint8_t state = FLIGHT_STATE_PAD;
uint8_t num_radio_transmissions = 0;

// tracking altitude for state of flight changing
float alt_ground = 0;
float alt_current = 0;
float alt_prev = 0;
float alt_diff = 0;
float alt_apogee = 0;
uint8_t num_descending_samples = 0; // for crude apogee detection

// bidirectional xtend communication
volatile char xtend_rx_buf[10];
volatile uint8_t xtend_rx_dma_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// helpers
void tone(uint32_t duration, uint32_t repeats);

// flash
int flash_write(char *msg_buffer);

uint8_t get_continuity(void);
float prop_poll_pressure_transducer(void);
float getAltitude(void);

uint8_t xtend_parse_dma_command(void);

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
  /* USER CODE BEGIN 2 */

  // *** IMPORTANT: DMA Init function must be called before peripheral init! *** //

  // FLASH set CS, WP and IO3 pins high
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);
  HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, SET);
  HAL_GPIO_WritePin(FLASH_IO3_GPIO_Port, FLASH_IO3_Pin, SET);

  // set CS for SD card high
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, SET);

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

  // set CS pin for thermocouple chip high (SPI idle CS is high)
  HAL_GPIO_WritePin(TH_CS_GPIO_Port, TH_CS_Pin, SET);

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

  // init i2c sensors and data storage
  dev_ctx_lsm = lsm6dsl_init();
  buzz_success();
  HAL_Delay(500);

  dev_ctx_lps = lps22hh_init();
  buzz_success();
  HAL_Delay(500);

  // init FLASH
  if (!W25qxx_Init()) Error_Handler();
  buzz_success();

  // init sd card with dynamic filename
  fres = sd_init_dynamic_filename("FC", sd_file_header, filename);
  if (fres != FR_OK) {
  		Error_Handler();
  }

  // check if flash empty and write to sd card if not
  int save_flash = save_flash_to_sd();
  if (save_flash) {
	  buzz_failure();
  }

//  VR_Power_On();
//  VR_Start_Rec();

  // init Iridium
//  MRT_Static_Iridium_Setup(huart3);
//  MRT_Static_Iridium_getIMEI();

  // send message with Iridium
//  MRT_Static_Iridium_sendMessage("message");
//  MRT_Static_Iridium_Shutdown();

  // start/stop video
//	  VR_Start_Rec();
//	  HAL_Delay(1000000);
//	  VR_Stop_Rec();
//	  buzz_success();

  // get ground altitude
  for (uint8_t i = 0; i < 100; i++) {
	  alt_ground += getAltitude();
  }
  alt_ground /= 100.0;
  alt_current = alt_ground;

  // initial DMA request for GPS
  HAL_UART_Receive_DMA(&huart6, gps_rx_buf, GPS_RX_DMA_BUF_LEN);

  // initial DMA request for XTend
  memset(xtend_rx_buf, 0, 10);
  HAL_UART_Receive_DMA(&huart3, xtend_rx_buf, XTEND_RX_DMA_CMD_LEN);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    uint32_t start_tick = HAL_GetTick();
	    uint32_t end_tick = 0; 	// polled later
	    uint32_t loop_duration; // end_tick - start_tick

//	    buzz_success();
	    HAL_Delay(10);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);

		// check for launch command -- do not do this in the callback because...reasons?
		if (xtend_rx_dma_ready) {
			// go check what the command is
			radio_command cmd = xtend_parse_dma_command();

			// prep for next command to be sent
			memset(xtend_rx_buf, 0, 10);
			HAL_UART_Receive_DMA(&huart3, xtend_rx_buf, XTEND_RX_DMA_CMD_LEN);
			xtend_rx_dma_ready = 0;

			switch (cmd) {
			case LAUNCH:
				rocket_launch();
				HAL_UART_Transmit(&huart8, "launch\r\n", 8, HAL_MAX_DELAY);
				break;

			case ARM_PROP:
				arming_propulsion();
				HAL_UART_Transmit(&huart8, "arm pr\r\n", 8, HAL_MAX_DELAY);
				break;

			case ARM_RCOV:
				arming_recovery();
				HAL_UART_Transmit(&huart8, "arm rc\r\n", 8, HAL_MAX_DELAY);
				break;

			case VR_POWER_ON:
				VR_Power_On();
				break;

			case VR_REC_START:
				VR_Start_Rec();
				break;

			case VR_REC_STOP:
				VR_Stop_Rec();
				break;

			case VR_POWER_OFF:
				VR_Power_Off();
				break;

			default:
				break;
			}
		}

		// -----  GATHER AVIONICS TELEMETRY ----- //
		// lsm6dsl data
		get_acceleration(dev_ctx_lsm, acceleration_mg);
		get_angvelocity(dev_ctx_lsm, angular_rate_mdps);

		// lps22hh data
		alt_current = getAltitude(); // calls get_pressure();
//		get_temperature(dev_ctx_lps, &temperature_degC);

		// rtc data
		HAL_RTC_GetTime(&hrtc, &stimeget, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sdateget, RTC_FORMAT_BIN); // have to call GetDate for the time to be correct

		// continuity on pyro channels
		continuity = get_continuity();

		// gps
//		if (gps_dma_ready) {
//			gps_dma_ready = 0;
//
//			char *gps_parsed = GPS_ParseBuffer(&latitude, &longitude, &time);
//			fres = sd_open_file(filename);
//			sd_write(&fil, "\nNew GPS\n");
//			f_close(&fil);
//
//			HAL_UART_Transmit(&huart8, gps_parsed, strlen(gps_parsed), HAL_MAX_DELAY);
//
//			// start new DMA request
//			HAL_UART_Receive_DMA(&huart6, gps_rx_buf, GPS_RX_DMA_BUF_LEN);
//		}

//		GPS_Poll(&latitude, &longitude, &time);

		if (latitude != 0) {
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		} else {
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		}

		if (longitude != 0) {
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
		} else {
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		}

		// propulsion data (not needed after apogee)
		if (state < FLIGHT_STATE_PRE_MAIN) {
			tank_temperature = Max31855_Read_Temp();
			tank_pressure = prop_poll_pressure_transducer();
			valve_state = HAL_GPIO_ReadPin(IN_Prop_ActuatedVent_Feedback_GPIO_Port, IN_Prop_ActuatedVent_Feedback_Pin);

			sprintf((char*) msg_buffer_pr, "P,%03.2f,%03.2f,%d,%02d,%02d,%lu,E\r\n",
					tank_pressure, tank_temperature, valve_state, stimeget.Minutes,
					stimeget.Seconds, stimeget.SubSeconds);
		}

		// -----  FORMATTING TELEMETRY ----- //

		// avionics message
		sprintf((char*) msg_buffer_av,
				"S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.7f,%03.7f,%02d,%02d,%lu,%d,%d,E\r\n",
				acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
				angular_rate_mdps[0], angular_rate_mdps[1],
				angular_rate_mdps[2], pressure_hPa, latitude, longitude,
				stimeget.Minutes, stimeget.Seconds, stimeget.SubSeconds,
				continuity, state);

		// save to sd and flash
		fres = sd_open_file(filename);
		sd_write(&fil, msg_buffer_av);
		if (state < FLIGHT_STATE_PRE_MAIN) {
			sd_write(&fil, msg_buffer_pr);
		}
		f_close(&fil);

//		flash_write((char *)msg_buffer_av);
//		flash_write((char *)msg_buffer_pr);

	  	#ifdef DEBUG
//			debug_tx_uart(msg_buffer_av);
//			debug_tx_uart(msg_buffer_pr);
		#endif

		// radio transmission
		switch (state) {
		case FLIGHT_STATE_PAD: // launch pad, waiting. prioritize prop data

			// check current state
			if (alt_current - alt_ground > LAUNCH_ALT_CHANGE_THRESHOLD) { // launched
				state = FLIGHT_STATE_PRE_APOGEE;

				fres = sd_open_file(filename);
				sd_write(&fil, (uint8_t *)"launched\r\n");
				f_close(&fil);
			}

			// send prop
			radio_tx(msg_buffer_pr, strlen((char *)msg_buffer_pr));

			if (num_radio_transmissions % 1 == 0) { // av at 2 Hz
				// send av
				radio_tx(msg_buffer_av, strlen((char *)msg_buffer_av));
			}

			num_radio_transmissions++;
			if (num_radio_transmissions == 10) {
				num_radio_transmissions = 0;
			}

			// loop timing calculation
			end_tick = HAL_GetTick();
			loop_duration = end_tick - start_tick;
			if (loop_duration < LOOP_DURATION_PAD) { // ticks in ms, hopefully loop runs at 10 Hz
				HAL_Delay(LOOP_DURATION_PAD - loop_duration);
			} // else just go straight back to top of loop

			break;

		case FLIGHT_STATE_PRE_APOGEE: // pre-apogee

			// check current state
			if (alt_current > alt_apogee) {
				alt_apogee = alt_current;
				num_descending_samples = 0;
			} else {
				num_descending_samples++;

				if (num_descending_samples > APOGEE_NUM_DESCENDING_SAMPLES) {
					state = FLIGHT_STATE_PRE_MAIN; // passed apogee
					num_descending_samples = 0;

					fres = sd_open_file(filename);
					sd_write(&fil, (uint8_t *)"apogee\r\n");
					f_close(&fil);

				}
			}

			// transmit avionics and prop at equal priority
			radio_tx(msg_buffer_av, strlen((char *)msg_buffer_av));
			radio_tx(msg_buffer_pr, strlen((char *)msg_buffer_pr));

			// loop timing calculation
			end_tick = HAL_GetTick();
			loop_duration = end_tick - start_tick;
			if (loop_duration < LOOP_DURATION_PRE_APOGEE) {
				HAL_Delay(LOOP_DURATION_PRE_APOGEE - loop_duration);
			} // else just go straight back to top of loop

			break;

		case FLIGHT_STATE_PRE_MAIN: // post-apogee

			// check current state
			if (alt_current < MAIN_DEPLOY_ALTITUDE) {
				num_descending_samples++;

				if (num_descending_samples > MAIN_NUM_DESCENDING_SAMPLES) {
					state = FLIGHT_STATE_PRE_LANDED;
					alt_prev = alt_current; // in next stage we need to know the previous altitude
					num_descending_samples = 0;

					fres = sd_open_file(filename);
					sd_write(&fil, (uint8_t *)"main deployed\r\n");
					f_close(&fil);
				}
			} else {
				num_descending_samples = 0;
			}

			// transmit avionics only
			radio_tx(msg_buffer_av, strlen((char *)msg_buffer_av));

			// loop timing calculation
			loop_duration = end_tick - start_tick;
			if (loop_duration < LOOP_DURATION_PRE_MAIN) {
				HAL_Delay(LOOP_DURATION_PRE_MAIN - loop_duration);
			} // else just go straight back to top of loop

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
					state = FLIGHT_STATE_LANDED;
					num_descending_samples = 0;

					fres = sd_open_file(filename);
					sd_write(&fil, (uint8_t *)"landed\r\n");
					f_close(&fil);
				}
			} else {
				num_descending_samples = 0;
			}

			// transmit avionics only
			radio_tx(msg_buffer_av, strlen((char *)msg_buffer_av));

			// loop timing calculation
			loop_duration = end_tick - start_tick;
			if (loop_duration < LOOP_DURATION_PRE_LANDED) {
				HAL_Delay(LOOP_DURATION_PRE_LANDED - loop_duration);
			} // else just go straight back to top of loop

			alt_prev = alt_current;
			break;

		case FLIGHT_STATE_LANDED: // landed
			// reduce transmission rate to save power. no need to check state anymore

			// transmit avionics only
			radio_tx(msg_buffer_av, strlen((char *)msg_buffer_av));

			// loop timing calculation
			loop_duration = end_tick - start_tick;
			if (loop_duration < LOOP_DURATION_LANDED) {
				HAL_Delay(LOOP_DURATION_LANDED - loop_duration);
			} // else just go straight back to top of loop

			break;

		default:
			VR_Stop_Rec();
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
			HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, SET);

			while (1); // terminate

			break;
		}

		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
		state++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart6) {

		// received data from GPS into buffer.
		// insert null termination and parse buffer (total buffer length is GPS_RX_DMA_BUF_LEN + 1)
		gps_rx_buf[GPS_RX_DMA_BUF_LEN] = '\0';
		gps_dma_ready = 1;
	}
	else if (huart == &huart3) {
		xtend_rx_dma_ready = 1;

		// the launch command is time critical, let's not wait until the next loop
		if (xtend_parse_dma_command() == LAUNCH) {
			rocket_launch();
		}
	}
}

float getAltitude(void) {
	get_pressure(dev_ctx_lps, &pressure_hPa);
	uint32_t altitude = 145442.1609 * (1.0 - pow(pressure_hPa/local_pressure, 0.190266436));
	return altitude;
}

int flash_write(char *msg_buffer) {
	// calculate page address and offset based on number of bytes already written
	uint32_t block_address = (uint32_t) (flash_write_address / w25qxx.BlockSize);
	uint32_t block_offset = (uint32_t) (flash_write_address % w25qxx.BlockSize);

	W25qxx_WriteBlock(msg_buffer, block_address, block_offset, strlen((const char *)msg_buffer));
	flash_write_address += strlen((const char *)msg_buffer);
}

uint8_t get_continuity(void) {
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
	HAL_GPIO_WritePin(LEDF_GPIO_Port, LEDF_Pin, GPIO_PIN_SET);
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
