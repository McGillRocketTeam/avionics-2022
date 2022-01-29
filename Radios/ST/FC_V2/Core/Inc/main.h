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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f3xx_hal.h"

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
#define LED_Status_Pin GPIO_PIN_2
#define LED_Status_GPIO_Port GPIOC
#define Button_Pin GPIO_PIN_3
#define Button_GPIO_Port GPIOC
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_4
#define Buzzer_GPIO_Port GPIOA
#define Relay_Drogue_1_Pin GPIO_PIN_6
#define Relay_Drogue_1_GPIO_Port GPIOA
#define Relay_Drogue_2_Pin GPIO_PIN_7
#define Relay_Drogue_2_GPIO_Port GPIOA
#define Relay_Main_1_Pin GPIO_PIN_4
#define Relay_Main_1_GPIO_Port GPIOC
#define Relay_Main_2_Pin GPIO_PIN_5
#define Relay_Main_2_GPIO_Port GPIOC
#define Drogue_Continuity_1_Pin GPIO_PIN_0
#define Drogue_Continuity_1_GPIO_Port GPIOB
#define Drogue_Sense_1_Pin GPIO_PIN_1
#define Drogue_Sense_1_GPIO_Port GPIOB
#define Main_Sense_1_Pin GPIO_PIN_2
#define Main_Sense_1_GPIO_Port GPIOB
#define Drogue_Continuity_2_Pin GPIO_PIN_10
#define Drogue_Continuity_2_GPIO_Port GPIOB
#define Input_Sense_1_Pin GPIO_PIN_11
#define Input_Sense_1_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOC
#define Main_Continuity_1_Pin GPIO_PIN_9
#define Main_Continuity_1_GPIO_Port GPIOC
#define Main_Continuity_2_Pin GPIO_PIN_8
#define Main_Continuity_2_GPIO_Port GPIOA
#define NSS_1_Pin GPIO_PIN_15
#define NSS_1_GPIO_Port GPIOA
#define NRESET_1_Pin GPIO_PIN_2
#define NRESET_1_GPIO_Port GPIOD
#define BUSY_1_Pin GPIO_PIN_4
#define BUSY_1_GPIO_Port GPIOB
#define DIO1_1_Pin GPIO_PIN_5
#define DIO1_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
