/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void); // for callbacks.c to see
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PM_12V_EN_Pin GPIO_PIN_2
#define PM_12V_EN_GPIO_Port GPIOE
#define SD_CS_Pin GPIO_PIN_10
#define SD_CS_GPIO_Port GPIOF
#define IN_Button_Pin GPIO_PIN_0
#define IN_Button_GPIO_Port GPIOC
#define IN_Button_EXTI_IRQn EXTI0_IRQn
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_2
#define Buzzer_GPIO_Port GPIOA
#define LEDF_Pin GPIO_PIN_3
#define LEDF_GPIO_Port GPIOA
#define ADC1_IN6_PropulsionPressureTransducer_Pin GPIO_PIN_6
#define ADC1_IN6_PropulsionPressureTransducer_GPIO_Port GPIOA
#define Prop_RunValve_LimitSwitch_Pin GPIO_PIN_5
#define Prop_RunValve_LimitSwitch_GPIO_Port GPIOC
#define IN_Prop_ActuatedVent_Feedback_Pin GPIO_PIN_1
#define IN_Prop_ActuatedVent_Feedback_GPIO_Port GPIOB
#define Prop_Cont_2_Pin GPIO_PIN_13
#define Prop_Cont_2_GPIO_Port GPIOF
#define Prop_Gate_2_Pin GPIO_PIN_14
#define Prop_Gate_2_GPIO_Port GPIOF
#define Prop_Gate_1_Pin GPIO_PIN_15
#define Prop_Gate_1_GPIO_Port GPIOF
#define Prop_Cont_1_Pin GPIO_PIN_0
#define Prop_Cont_1_GPIO_Port GPIOG
#define Prop_Pyro_Arming_Pin GPIO_PIN_1
#define Prop_Pyro_Arming_GPIO_Port GPIOG
#define Vent_Valve_EN_Pin GPIO_PIN_7
#define Vent_Valve_EN_GPIO_Port GPIOE
#define Payload_EN_Pin GPIO_PIN_9
#define Payload_EN_GPIO_Port GPIOE
#define TH_CS_Pin GPIO_PIN_10
#define TH_CS_GPIO_Port GPIOE
#define IN_XTend_Continuity_Pin GPIO_PIN_11
#define IN_XTend_Continuity_GPIO_Port GPIOE
#define Iridium_RST_Pin GPIO_PIN_15
#define Iridium_RST_GPIO_Port GPIOE
#define SX_NSS_Pin GPIO_PIN_12
#define SX_NSS_GPIO_Port GPIOB
#define USART3_TX_XTend_Pin GPIO_PIN_8
#define USART3_TX_XTend_GPIO_Port GPIOD
#define USART3_RX_XTend_Pin GPIO_PIN_9
#define USART3_RX_XTend_GPIO_Port GPIOD
#define XTend_CTS_Pin GPIO_PIN_10
#define XTend_CTS_GPIO_Port GPIOD
#define XTend_RTS_Pin GPIO_PIN_11
#define XTend_RTS_GPIO_Port GPIOD
#define XTend_SLEEP_Pin GPIO_PIN_12
#define XTend_SLEEP_GPIO_Port GPIOD
#define XTend_RX_LED_Pin GPIO_PIN_13
#define XTend_RX_LED_GPIO_Port GPIOD
#define XTend_TX_PWR_Pin GPIO_PIN_14
#define XTend_TX_PWR_GPIO_Port GPIOD
#define SX_BANDPASS_FILTER_Pin GPIO_PIN_15
#define SX_BANDPASS_FILTER_GPIO_Port GPIOD
#define SX_RST_Pin GPIO_PIN_2
#define SX_RST_GPIO_Port GPIOG
#define SX_BUSY_Pin GPIO_PIN_3
#define SX_BUSY_GPIO_Port GPIOG
#define SX_DIO_Pin GPIO_PIN_4
#define SX_DIO_GPIO_Port GPIOG
#define SX_RF_SW_Pin GPIO_PIN_5
#define SX_RF_SW_GPIO_Port GPIOG
#define EXTI_LPS22HH_DRDY_Pin GPIO_PIN_6
#define EXTI_LPS22HH_DRDY_GPIO_Port GPIOG
#define EXTI_ISM330DCL_INT2_Pin GPIO_PIN_7
#define EXTI_ISM330DCL_INT2_GPIO_Port GPIOG
#define EXTI_ISM330DLC_INT1_Pin GPIO_PIN_8
#define EXTI_ISM330DLC_INT1_GPIO_Port GPIOG
#define USART6_TX_GPS_Pin GPIO_PIN_6
#define USART6_TX_GPS_GPIO_Port GPIOC
#define USART6_RX_GPS_Pin GPIO_PIN_7
#define USART6_RX_GPS_GPIO_Port GPIOC
#define SX_AMPLIFIER_Pin GPIO_PIN_8
#define SX_AMPLIFIER_GPIO_Port GPIOC
#define IN_SD_CARD_DETECT_Pin GPIO_PIN_12
#define IN_SD_CARD_DETECT_GPIO_Port GPIOC
#define FLASH_IO3_Pin GPIO_PIN_4
#define FLASH_IO3_GPIO_Port GPIOD
#define FLASH_WP_Pin GPIO_PIN_5
#define FLASH_WP_GPIO_Port GPIOD
#define FLASH_CS_Pin GPIO_PIN_6
#define FLASH_CS_GPIO_Port GPIOD
#define VR_CTRL_REC_Pin GPIO_PIN_7
#define VR_CTRL_REC_GPIO_Port GPIOD
#define VR_CTRL_PWR_Pin GPIO_PIN_9
#define VR_CTRL_PWR_GPIO_Port GPIOG
#define Rcov_Cont_Main_Pin GPIO_PIN_10
#define Rcov_Cont_Main_GPIO_Port GPIOG
#define Rcov_Gate_Main_Pin GPIO_PIN_11
#define Rcov_Gate_Main_GPIO_Port GPIOG
#define Rcov_Gate_Drogue_Pin GPIO_PIN_12
#define Rcov_Gate_Drogue_GPIO_Port GPIOG
#define Rcov_Cont_Drogue_Pin GPIO_PIN_13
#define Rcov_Cont_Drogue_GPIO_Port GPIOG
#define Rcov_Arm_Pin GPIO_PIN_14
#define Rcov_Arm_GPIO_Port GPIOG
#define DEBUG_GPIO_Pin GPIO_PIN_8
#define DEBUG_GPIO_GPIO_Port GPIOB
#define POWER_ON_EXT_LED_Pin GPIO_PIN_9
#define POWER_ON_EXT_LED_GPIO_Port GPIOB
#define UART8_RX_Debug_Pin GPIO_PIN_0
#define UART8_RX_Debug_GPIO_Port GPIOE
#define UART8_TX_Debug_Pin GPIO_PIN_1
#define UART8_TX_Debug_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi5
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart6;

// -- 2020 FC parameters -- //
#define		DROGUE_DELAY			500			// ms (Time that drogue is HIGH)
#define		MAIN_DELAY				500			// ms (Time that main is HIGH)

// Configurations
#define		NUM_MEAS_REG			50			// Number of historic measurements for linear regression
#define		ALT_MEAS_AVGING			500
#define		NUM_DESCENDING_SAMPLES	10			// Number of descending slope values for apogee detection to pass
// -- end 2020 parameters -- //

// -- 2021 megaloop parameters -- //
#define LAUNCH_ALT_CHANGE_THRESHOLD		75		// ft, change in altitude needed to change to "launched" state
#define LAUNCH_NUM_DESCENDING_SAMPLES	20		// number of samples needed to set as launched
#define APOGEE_NUM_DESCENDING_SAMPLES	30
#define MAIN_NUM_DESCENDING_SAMPLES		20
#define LANDING_NUM_DESCENDING_SAMPLES	100		// number of samples needed to set as landing
#define MAIN_DEPLOY_ALTITUDE			1500	// ft
#define LANDING_ALT_CHANGE_THRESHOLD	5		// ft

#define XTEND_RX_DMA_CMD_LEN			2		// all commands have this length (in chars)

#define LOCAL_PRESSURE_HPA		1028	// hPa

#define USING_XTEND 	// comment out to use SRADio


//New ADC
#define NEW_ADC
#define ADC_I2C 2


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
