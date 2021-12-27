/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define PM_12V_EN_Pin GPIO_PIN_2
#define PM_12V_EN_GPIO_Port GPIOE
#define SD_CS_Pin GPIO_PIN_10
#define SD_CS_GPIO_Port GPIOF
#define Button_Pin GPIO_PIN_0
#define Button_GPIO_Port GPIOC
#define Button_EXTI_IRQn EXTI0_IRQn
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOC
#define Vent_Valve_FB_Pin GPIO_PIN_1
#define Vent_Valve_FB_GPIO_Port GPIOB
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
#define TH_CS_1_Pin GPIO_PIN_10
#define TH_CS_1_GPIO_Port GPIOE
#define TH_CS_2_Pin GPIO_PIN_11
#define TH_CS_2_GPIO_Port GPIOE
#define VR_CTRL_REC_Pin GPIO_PIN_15
#define VR_CTRL_REC_GPIO_Port GPIOA
#define VR_CTRL_PWR_Pin GPIO_PIN_10
#define VR_CTRL_PWR_GPIO_Port GPIOC
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
/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi5
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c3;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
