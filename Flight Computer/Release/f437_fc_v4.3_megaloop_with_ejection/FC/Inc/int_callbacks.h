/*
 * int_callbacks.h
 *
 *  Created on: Apr. 8, 2022
 *      Author: jasper
 */

#ifndef INC_INT_CALLBACKS_H_
#define INC_INT_CALLBACKS_H_

#include "main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


#endif /* INC_INT_CALLBACKS_H_ */
