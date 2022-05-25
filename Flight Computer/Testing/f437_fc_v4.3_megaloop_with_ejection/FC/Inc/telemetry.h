/*
 * telemetry.h
 *
 *  Created on: Apr 6, 2022
 *      Author: jasper
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>
#include "rtc.h"

#define PROP_TANK_PRESSURE_ADC_BUF_LEN	15	// samples
#define PROP_ADC_TIMER	tim8	// interrupt drives calls to ADC

extern volatile float acceleration_mg[3];
extern volatile float angular_rate_mdps[3];
extern volatile float pressure_hPa;
extern volatile float temperature_degC;

extern volatile float tank_temperature;
extern uint8_t dump_valve_state;
extern uint8_t run_valve_state;
extern volatile float tank_pressure;
extern volatile uint16_t tank_pressure_buf[PROP_TANK_PRESSURE_ADC_BUF_LEN]; // circular buffer for averaging (low pass filter)
extern volatile uint8_t tank_pressure_buf_idx;

extern RTC_TimeTypeDef stimeget;
extern RTC_DateTypeDef sdateget;
extern volatile uint8_t sleepmode;

extern volatile uint8_t continuity;

extern volatile uint8_t msg_buffer_av[200];
extern volatile uint8_t msg_buffer_pr[50];
extern volatile uint8_t msg[1000];

// functions
void telemetry_init(void);
void telemetry_format_avionics(void);
void telemetry_format_propulsion(void);
void read_update_telemetry_av(void);
void read_update_telemetry_pr(void);

float getAltitude(void);

void prop_poll_pressure_transducer(void);
float convert_prop_tank_pressure(void);
uint8_t get_actuated_valve_state(void);
uint8_t get_run_valve_state(void);

#endif /* INC_TELEMETRY_H_ */
