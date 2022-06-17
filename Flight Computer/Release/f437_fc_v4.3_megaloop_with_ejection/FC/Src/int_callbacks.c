/*
 * int_callbacks.c
 *
 * contains the interrupt callback functions.
 *
 * setup:
 * 	RTC:
 * 		- Alarm A and Alarm B
 * 		- Wakeup Timer
 *
 * 	Timers:
 * 		- TIM3: drives XTend transmissions via DMA
 * 		- TIM8: drives ADC polling calls
 *
 * 	GPIO:
 * 		- button interrupt
 * 		- software interrupt
 *
 * 	UART:
 * 		- UART3: XTend TX and RX
 * 		- UART6: GPS RX
 *
 *  Created on: Apr. 8, 2022
 *      Author: jasper
 */


#include "int_callbacks.h"
#include "main.h"
#include "xtend.h"
#include "gps.h"
#include "radio_commands.h"
#include "ejection.h"
#include "helpers.h"
#include "state_restoration.h"
#include "telemetry.h"
#include "sd_card.h"

#include "tim.h"
#include "iwdg.h"
#include "rtc.h"

#include <string.h>
#include <stdio.h>

extern volatile uint8_t button_pressed;
extern volatile uint8_t gps_dma_ready;
extern volatile uint8_t xtend_time_to_transmit_flag;
extern volatile uint8_t xtend_is_transmitting;
extern volatile uint8_t save_sd_fsync;
extern volatile uint8_t run_main_loop;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == IN_Button_Pin) {
		button_pressed = 1;

//		state_arm_prop++;
//		state_arm_rcov++;
//
//		set_backup_state(FC_STATE_ARM_PROP, (uint32_t) state_arm_prop);
//		set_backup_state(FC_STATE_ARM_RCOV, (uint32_t) state_arm_rcov);
//
//		sprintf((char *)msg, "state = %d, ap = %d, ar = %d, alt_ground = %f",
//					  state_flight, state_arm_prop, state_arm_rcov, alt_ground);
//		debug_tx_uart(msg);
//
//		while (1); // trigger watchdog during testing

//		__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
	}
	else if (GPIO_Pin == EXTI_SWIER_SWIER4) { // software interrupt to change timer settings
		update_radio_timer_params(state_flight);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &GPS_USART) { // gps
		// insert null termination and indicate buffer is ready to parse
		// (total buffer length is GPS_RX_DMA_BUF_LEN + 1)
		gps_rx_buf[GPS_RX_DMA_BUF_LEN] = '\0';
		gps_dma_ready = 1;
	}
	else if (huart == &XTEND_UART) { // xtend radio
		// go check what the command is
		radio_command cmd = parse_radio_command(xtend_rx_buf);

		// send acknowledge
		HAL_UART_DMAStop(&XTEND_UART);

		char ack_buf[20] = {0};
		sprintf(ack_buf, "\n\n%s_%d\r\n\n", xtend_ack_msg, (uint8_t)cmd);
		xtend_tx(ack_buf, strlen((const char *)ack_buf));

		// prep for next command to be sent
		memset(xtend_rx_buf, 0, 10);
		HAL_UART_Receive_DMA(&XTEND_UART, xtend_rx_buf, XTEND_RX_DMA_CMD_LEN);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &XTEND_UART) {
//		#ifdef TIMING_ ITM
//			ITM_Port32(31) = 200;
//		#endif

		xtend_is_transmitting = 0;
		num_radio_transmissions++;
		if (num_radio_transmissions == 10) {
			num_radio_transmissions = 0;
		}
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

	HAL_RTCEx_DeactivateWakeUpTimer(hrtc);
	xtend_sleep();

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
	xtend_wake();

	sprintf((char *)msg, "Alarm B callback entered\r\n");
	debug_tx_uart(msg);

    sleepmode = 0;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
	SystemClock_Config();
	HAL_ResumeTick(); // unnecessary? systemclock_config should restart the tick
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);

	if (sleepmode && __HAL_PWR_GET_FLAG(PWR_FLAG_WU) && state_flight == EJ_STATE_LANDED) {
		sprintf((char *)msg, "wakeup timer callback\r\n");
		debug_tx_uart(msg);
	}
	else {
//		xtend_wake();

		// poll av and telemetry, send it, then sleep
		read_update_telemetry_av();
		read_update_telemetry_pr();

		telemetry_format_avionics();
		telemetry_format_propulsion();

		xtend_tx(msg_buffer_av, strlen((char *)msg_buffer_av));
		xtend_tx(msg_buffer_pr, strlen((char *)msg_buffer_pr));

		sprintf((char *)msg, "wakeup timer poll sensors\r\n");
		debug_tx_uart(msg);

//		xtend_sleep();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &XTEND_TIM_H) {
//		#ifdef TIMING_ITM
//			ITM_Port32(31) = 100;
//		#endif

		HAL_GPIO_TogglePin(LEDF_GPIO_Port, LEDF_Pin);
		xtend_time_to_transmit_flag = 1;
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
	else if (htim == &htim12) {	// saving to sd card
		static uint8_t tim12_counts = 0;

		HAL_GPIO_TogglePin(POWER_ON_EXT_LED_GPIO_Port, POWER_ON_EXT_LED_Pin);
		tim12_counts += 1;
		run_main_loop = 1;

		if (tim12_counts == 50) {
			save_sd_fsync = 1;
			tim12_counts = 0;
		}
	}
}
