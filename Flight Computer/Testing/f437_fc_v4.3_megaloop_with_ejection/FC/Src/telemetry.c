/*
 * telemetry.c
 *
 *  Created on: Apr 6, 2022
 *      Author: jasper
 */

#include "telemetry.h"
#include "main.h"
#include "gps.h"
#include "rtc.h"
#include "ejection.h"
#include "adc.h"
#include "i2c_sensor_functions.h"
#include "gpio.h"
#include "MAX31855.h"

#include <stdio.h> // sprintf

extern volatile uint8_t gps_dma_ready; // flag
volatile float acceleration_mg[3];
volatile float angular_rate_mdps[3];
volatile float pressure_hPa;
volatile float temperature_degC;

volatile float tank_temperature;
uint8_t dump_valve_state;
uint8_t run_valve_state;
volatile float tank_pressure;
volatile uint16_t tank_pressure_buf[PROP_TANK_PRESSURE_ADC_BUF_LEN]; // circular buffer for averaging (low pass filter)
volatile uint8_t tank_pressure_buf_idx;

RTC_TimeTypeDef stimeget;
RTC_DateTypeDef sdateget;
volatile uint8_t sleepmode;

volatile uint8_t continuity;

volatile uint8_t msg_buffer_av[200];
volatile uint8_t msg_buffer_pr[50];
volatile uint8_t msg[1000];

void telemetry_init(void) {
	telemetry_format_avionics();
	telemetry_format_propulsion();
}

// formats avionics telemetry string using sprintf
void telemetry_format_avionics(void) {
	uint16_t sec = stimeget.Seconds;
	if (__HAL_RTC_SHIFT_GET_FLAG(&hrtc, RTC_FLAG_SHPF)) {
		sec += 1;
	}

	sprintf((char*) msg_buffer_av,
			"S,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.7f,%.7f,%02d,%02d,%lu,%d,%d,E\r\n",
			acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
			angular_rate_mdps[0], angular_rate_mdps[1],
			angular_rate_mdps[2], pressure_hPa, GPS.dec_latitude, GPS.dec_longitude,
			stimeget.Minutes, sec, stimeget.SubSeconds,
			state_flight, continuity);
}

// formats propulsion telemetry string using sprintf
void telemetry_format_propulsion(void) {
	uint16_t sec = stimeget.Seconds;
	if (__HAL_RTC_SHIFT_GET_FLAG(&hrtc, RTC_FLAG_SHPF)) {
		sec += 1;
	}

	tank_pressure = convert_prop_tank_pressure(); // convert buffered readings to voltage
	sprintf((char*) msg_buffer_pr, "P,%.4f,%.4f,%d,%d,%02d,%02d,%lu,E\r\n",
			tank_pressure, tank_temperature, dump_valve_state, run_valve_state, stimeget.Minutes,
			sec, stimeget.SubSeconds);
}

// reads all sensors and updates values in global variables
void read_update_telemetry_av(void) {
	get_acceleration(dev_ctx_lsm, acceleration_mg);
	get_angvelocity(dev_ctx_lsm, angular_rate_mdps);
	alt_current = runAltitudeMeasurements(HAL_GetTick(), getAltitude());

	HAL_RTC_GetTime(&hrtc, &stimeget, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdateget, RTC_FORMAT_BIN); // must call GetDate for time to be correct

	continuity = get_continuity();

	if (gps_dma_ready) {
		gps_dma_ready = 0;
		GPS_ParseBuffer();

		if (GPS.dec_latitude != 0) HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		if (GPS.dec_longitude != 0) HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

		// start new DMA request
		HAL_UART_Receive_DMA(&huart6, gps_rx_buf, GPS_RX_DMA_BUF_LEN);
	}
}

void read_update_telemetry_pr(void) {
	tank_temperature = Max31855_Read_Temp();
	dump_valve_state = get_actuated_valve_state();
	run_valve_state = get_run_valve_state();
}

// gets current pressure reading from barometer and converts to altitude
float getAltitude(void) {
	get_pressure(dev_ctx_lps, &pressure_hPa);
	uint32_t altitude = 145442.1609 * (1.0 - pow(pressure_hPa/LOCAL_PRESSURE_HPA, 0.190266436));
	return altitude;
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

// averages the values in the circular buffer (tank_pressure_buf)
// and returns the voltage in V, assuming 12 bit ADC readings.
float convert_prop_tank_pressure(void) {
	uint32_t avg = 0;
	for (uint16_t i = 0; i < PROP_TANK_PRESSURE_ADC_BUF_LEN; i++) {
		avg += tank_pressure_buf[i];
	}

	float pressure = ((float) avg) / PROP_TANK_PRESSURE_ADC_BUF_LEN / 4095.0 * 3.3; // 12 bit ADC
	return pressure;
}

// reads feedback pin for actuated valve (dump valve) for propulsion
uint8_t get_actuated_valve_state(void) {
	uint8_t vstate = (uint8_t) HAL_GPIO_ReadPin(IN_Prop_ActuatedVent_Feedback_GPIO_Port, IN_Prop_ActuatedVent_Feedback_Pin);
	return vstate;
}

// reads limit switch pin for run valve for propulsion
uint8_t get_run_valve_state(void) {
	return ((uint8_t) HAL_GPIO_ReadPin(IN_Prop_ActuatedVent_Feedback_GPIO_Port, IN_Prop_ActuatedVent_Feedback_Pin));
}
