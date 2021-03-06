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

#include "i2c_sensor_functions.h"
#include "sd_card.h"
#include "gps.h"

// xtend radio
#include "xtend.h"

// others
#include "video_recorder.h"
#include "ejection.h"
#include "radio_commands.h"
#include "helpers.h"
#include "state_restoration.h"
#include "telemetry.h"
#include "startup.h"
#include "int_callbacks.h"
#include <iwdg.h>

#ifdef NEW_ADC
#include <MRT_ADS1015.h>
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//#define DEBUG_MODE

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// flags
volatile uint8_t button_pressed = 0;
volatile uint8_t gps_dma_ready = 0;
volatile uint8_t xtend_time_to_transmit_flag = 0;
volatile uint8_t xtend_is_transmitting = 0;
volatile uint8_t save_sd_fsync = 0;
volatile uint8_t run_main_loop = 0;

extern volatile uint8_t vr_is_stop;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//TODO no print function
void no_print(char* text){
	return;
}

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
  MX_IWDG_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */

  // *** IMPORTANT: DMA Init function must be called before peripheral init! *** //

  init_gpio_startup(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST));

  if (!(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))) {
	  init_ejection_states();
	  init_backup_regs();
  }
  else {
	  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
	  restore_fc_states();

	  sprintf((char *)msg, "state = %d, ap = %d, ar = %d, alt_ground = %f",
			  state_flight, state_arm_prop, state_arm_rcov, alt_ground);
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

		  HAL_Delay(100);
	  }
  }

  #ifndef USING_XTEND
	  set_hspi(hspi2);
	  set_NSS_pin(SX_NSS_GPIO_Port, SX_NSS_Pin);
	  set_BUSY_pin(SX_BUSY_GPIO_Port, SX_BUSY_Pin);
	  set_NRESET_pin(SX_RST_GPIO_Port, SX_RST_Pin);
	  set_DIO1_pin(SX_DIO_GPIO_Port, SX_DIO_Pin);
	  Tx_setup();
  #endif

  dev_ctx_lsm = lsm6dsl_init();
  dev_ctx_lps = lps22hh_init();

  //TODO new ADC
  #ifdef NEW_ADC
  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
  hadc1015 = MRT_ADC1015_Init(ADC_I2C,no_print);
  #endif

  sd_init_dynamic_filename("TE", sd_file_header, filename);
  GPS_StartDMA();
  xtend_init();

  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
  telemetry_init();

  if (!(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))) {
	  buzz_startup_success();	// got to this point, successful init

	  // get ground altitude
	  for (uint8_t i = 0; i < 100; i++) { // ALT_MEAS_AVGING defined in main.h
		  alt_ground += getAltitude();
	  }
	  alt_ground /= ((float) (100));
	  set_backup_state(FC_STATE_ALT_GROUND, (uint32_t) alt_ground);
	  alt_current = alt_ground;
  }

  sprintf((char *)msg_buffer_av, "alt_ground=%.3f\r\n\n", alt_ground);
  fres = sd_open_file(filename);
  sd_write(&fil, msg_buffer_av);
  if (state_flight < EJ_STATE_DROGUE_DESCENT) {
	  sd_write(&fil, msg_buffer_pr);
  }
  memset(msg_buffer_av, 0, 200);

  // start timers:
  HAL_TIM_Base_Start_IT(&XTEND_TIM_H);	// drives XTend transmitting
  //HAL_TIM_Base_Start_IT(&htim8);		// drives ADC TODO remove
  HAL_TIM_Base_Start_IT(&htim12);		// drives main while loop

  __HAL_RCC_CLEAR_RESET_FLAGS();		// clear RCC reset flags

  HAL_GPIO_WritePin(DEBUG_GPIO_GPIO_Port, DEBUG_GPIO_Pin, RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		if (run_main_loop) {

			run_main_loop = 0;
			__HAL_IWDG_RELOAD_COUNTER(&hiwdg); // refresh watchdog

			read_update_telemetry_av();

			if (state_flight < EJ_STATE_DROGUE_DESCENT) { // propulsion data (not needed after apogee)
				read_update_telemetry_pr();
				telemetry_format_propulsion();
			}

			telemetry_format_avionics();

			sd_write(&fil, msg_buffer_av);
			if (state_flight < EJ_STATE_DROGUE_DESCENT) {
				sd_write(&fil, msg_buffer_pr);
			}

			if (save_sd_fsync) {
				save_sd_fsync = 0;
				f_sync(&fil);
			}

		#ifdef DEBUG_MODE
//			debug_tx_uart(msg_buffer_av);
//			debug_tx_uart(msg_buffer_pr);
			sprintf(msg, "states of fc: s=%d, ap=%d, ar=%d, HH:MM:SS = %02d:%02d:%02d\r\n",
				   state_flight, state_arm_prop, state_arm_rcov,
				   stimeget.Hours, stimeget.Minutes, stimeget.Seconds);
			debug_tx_uart(msg);
		#endif

			if (xtend_time_to_transmit_flag) {
				xtend_time_to_transmit_flag = 0;
				xtend_transmit_telemetry(state_flight);
			}

			check_ejection_state(); // check which state of flight we are in

			__HAL_IWDG_RELOAD_COUNTER(&hiwdg);

			if (sleepmode == 1) { // stop mode after landing

				__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//				HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 2000-1, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
				HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 20000-1, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
				HAL_PWR_EnableSleepOnExit();
				HAL_SuspendTick();
				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			}
			else if (sleepmode == 2) { // sleep mode on pad
				__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//				HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 2000-1, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
				HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 20000-1, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

				HAL_TIM_Base_Stop_IT(&XTEND_TIM_H);
				HAL_TIM_Base_Stop_IT(&htim8);
				HAL_TIM_Base_Stop_IT(&htim12);

				HAL_PWR_EnableSleepOnExit();
				HAL_SuspendTick();
				HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			}

//			if (state_flight == EJ_STATE_LANDED) {
//				for (uint8_t i = 0; i < 5; i++) { // 5 seconds
//					__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
//					HAL_Delay(1000); // slow it down
//				}
//			}

//			if (vr_is_stop) {
//				vr_is_stop = 0;
//
//				HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, SET);
//				HAL_Delay(1000);
//				HAL_GPIO_WritePin(VR_CTRL_REC_GPIO_Port, VR_CTRL_REC_Pin, RESET);
//			}
		}

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
}

/* USER CODE BEGIN 4 */

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
