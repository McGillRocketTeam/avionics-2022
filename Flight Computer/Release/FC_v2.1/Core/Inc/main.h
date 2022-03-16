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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_12V_Buck_Pin GPIO_PIN_2
#define EN_12V_Buck_GPIO_Port GPIOE
#define SD_CS_Pin GPIO_PIN_10
#define SD_CS_GPIO_Port GPIOF
#define IN_Button_Pin GPIO_PIN_0
#define IN_Button_GPIO_Port GPIOC
#define IN_Button_EXTI_IRQn EXTI0_IRQn
#define OUT_LED1_Pin GPIO_PIN_1
#define OUT_LED1_GPIO_Port GPIOC
#define OUT_LED2_Pin GPIO_PIN_2
#define OUT_LED2_GPIO_Port GPIOC
#define OUT_LED3_Pin GPIO_PIN_3
#define OUT_LED3_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_2
#define Buzzer_GPIO_Port GPIOA
#define OUT_LEDF_Pin GPIO_PIN_3
#define OUT_LEDF_GPIO_Port GPIOA
#define ADC1_IN6_PropulsionPressureTransducer_Pin GPIO_PIN_6
#define ADC1_IN6_PropulsionPressureTransducer_GPIO_Port GPIOA
#define IN_Prop_PyroTurboValve_LimitSwitch_Pin GPIO_PIN_5
#define IN_Prop_PyroTurboValve_LimitSwitch_GPIO_Port GPIOC
#define IN_Prop_ActuatedVent_Feedback_Pin GPIO_PIN_1
#define IN_Prop_ActuatedVent_Feedback_GPIO_Port GPIOB
#define IN_PyroValve_Cont_2_Pin GPIO_PIN_13
#define IN_PyroValve_Cont_2_GPIO_Port GPIOF
#define OUT_PyroValve_Gate_2_Pin GPIO_PIN_14
#define OUT_PyroValve_Gate_2_GPIO_Port GPIOF
#define OUT_PyroValve_Gate_1_Pin GPIO_PIN_15
#define OUT_PyroValve_Gate_1_GPIO_Port GPIOF
#define IN_PyroValve_Cont_1_Pin GPIO_PIN_0
#define IN_PyroValve_Cont_1_GPIO_Port GPIOG
#define OUT_PyroValve_Arming_Pin GPIO_PIN_1
#define OUT_PyroValve_Arming_GPIO_Port GPIOG
#define OUT_Prop_ActuatedVent_Gate_Pin GPIO_PIN_7
#define OUT_Prop_ActuatedVent_Gate_GPIO_Port GPIOE
#define PAYLOAD_I2C_EN_Pin GPIO_PIN_9
#define PAYLOAD_I2C_EN_GPIO_Port GPIOE
#define SPI4_CS_Thermocouple_Pin GPIO_PIN_10
#define SPI4_CS_Thermocouple_GPIO_Port GPIOE
#define IN_XTend_Continuity_Pin GPIO_PIN_11
#define IN_XTend_Continuity_GPIO_Port GPIOE
#define Iridium_RST_Pin GPIO_PIN_15
#define Iridium_RST_GPIO_Port GPIOE
#define SPI2_SX_CS_Pin GPIO_PIN_12
#define SPI2_SX_CS_GPIO_Port GPIOB
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
#define EXTI_LSM6DSR_INT1_Pin GPIO_PIN_8
#define EXTI_LSM6DSR_INT1_GPIO_Port GPIOG
#define USART6_TX_GPS_Pin GPIO_PIN_6
#define USART6_TX_GPS_GPIO_Port GPIOC
#define USART6_RX_GPS_Pin GPIO_PIN_7
#define USART6_RX_GPS_GPIO_Port GPIOC
#define SX_AMPLIFIER_Pin GPIO_PIN_8
#define SX_AMPLIFIER_GPIO_Port GPIOC
#define IN_SD_CARD_DETECT_Pin GPIO_PIN_12
#define IN_SD_CARD_DETECT_GPIO_Port GPIOC
#define OUT_FLASH_IO3_Pin GPIO_PIN_4
#define OUT_FLASH_IO3_GPIO_Port GPIOD
#define OUT_FLASH_WP_Pin GPIO_PIN_5
#define OUT_FLASH_WP_GPIO_Port GPIOD
#define OUT_FLASH_CS_Pin GPIO_PIN_6
#define OUT_FLASH_CS_GPIO_Port GPIOD
#define OUT_VR_REC_Pin GPIO_PIN_7
#define OUT_VR_REC_GPIO_Port GPIOD
#define OUT_VR_PWR_Pin GPIO_PIN_9
#define OUT_VR_PWR_GPIO_Port GPIOG
#define IN_EJ_Main_Cont_Pin GPIO_PIN_10
#define IN_EJ_Main_Cont_GPIO_Port GPIOG
#define OUT_EJ_Main_Gate_Pin GPIO_PIN_11
#define OUT_EJ_Main_Gate_GPIO_Port GPIOG
#define OUT_EJ_Drogue_Gate_Pin GPIO_PIN_12
#define OUT_EJ_Drogue_Gate_GPIO_Port GPIOG
#define IN_EJ_Drogue_Cont_Pin GPIO_PIN_13
#define IN_EJ_Drogue_Cont_GPIO_Port GPIOG
#define OUT_EJ_Arming_Pin GPIO_PIN_14
#define OUT_EJ_Arming_GPIO_Port GPIOG
#define UART8_RX_Debug_Pin GPIO_PIN_0
#define UART8_RX_Debug_GPIO_Port GPIOE
#define UART8_TX_Debug_Pin GPIO_PIN_1
#define UART8_TX_Debug_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */


//TODO
UART_HandleTypeDef huart8;
#define DEBUG_UART huart8
#define SD_SPI_HANDLE hspi5

#define SEA_LEVEL_TEMPERATURE 25+273.15 //Sea level temperature kelvin
#define SEA_LEVEL_PRESSURE 1014 //Sea level pressure hPa
#define  BASE_HEIGHT 100 //In meters
#define M 0.0289644 //Molar mass of earth's air in kg/mol
#define go 9.80665 //Gravitational acceleration constant in m/s^2
#define R 8.31432 //Universal gas constant Nm / mol K


//RTOS
#define SLEEP_TIME 5 //In seconds
#define WHEN_SLEEP_TIME_SEC 15  //In seconds
#define WHEN_SLEEP_TIME_MIN 0  //In minutes
#define WHEN_SLEEP_TIME_HOURS 0  //In hours

//WatchDog Thread
#define WD_FREQ 1 //Times per second that you want to check threads


//Telemetry
#define DATA_FREQ 10 //Times per second that you want to save data
#define SEND_FREQ 20 //Times per second that you want to transmit data

//SRadio
#define SRADIO_SPI hspi2

//XTend
#define XTEND_UART huart3
#define XTEND_BUFFER_SIZE 256

//Iridium
//#define IRIDIUM_I2C I2C2 //TODO defined in the IridiumSBD.h


//Sensors
#define POLL_FREQ 20 //Times per second that you want to poll data

//GPS
#define GPS_UART huart6 //TODO cannot call it GPS_USART because already defined in gps.c

//LSM6DSR
#define LSM_I2C hi2c3

//LPS22HH
#define LPS_I2C hi2c3


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
