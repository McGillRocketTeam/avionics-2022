/*
 * radio_commands.h
 *
 *  Created on: Mar 1, 2022
 *      Author: jasper
 */

#ifndef INC_RADIO_COMMANDS_H_
#define INC_RADIO_COMMANDS_H_

#include <stdint.h>

typedef enum radio_command {
	LAUNCH = 1,
	ARM_PROP,
	ARM_RCOV,
	DISARM_PROP,
	DISARM_RCOV,
	DUMP_POWER_ON,
	DUMP_POWER_OFF,
	PAD_SLEEP_CMD,
	PAD_WAKE_CMD,
	VR_POWER_ON,
	VR_REC_START,
	VR_REC_STOP,
	VR_POWER_OFF,

#ifdef TEST_EJECTION_FIRING
	FIRE_RCOV_DROGUE,
	FIRE_RCOV_MAIN,
	FIRE_PROP_1,
	FIRE_PROP_2,
#endif

	COMMAND_INVALID,
} radio_command;

// defines for transmit rate
// assuming timer freq = 90 MHz, PSC = 9000-1, use ARR to get desired counter freq
// 		10 Hz -> ARR = 1000
// 		 5 Hz -> ARR = 2000
// 		 2 Hz -> ARR = 5000
// 		 1 Hz -> ARR = 10000
//	   0.5 Hz -> ARR = 20000
//	   0.2 Hz -> ARR = 50000
#define ARR_PAD				1000
#define ARR_BOOST_COAST		5000
#define ARR_DROGUE_DESCENT	2000
#define ARR_MAIN_DESCENT	1000
#define ARR_LANDED			50000

// functions
radio_command parse_radio_command(volatile char *buf);
void rocket_launch(void);
void arming_propulsion(void);
void arming_recovery(void);
void disarm_propulsion(void);
void disarm_recovery(void);
void dump_power_on(void);
void dump_power_off(void);
void pad_sleep_mode(void);
void pad_wake_mode(void);
void xtend_transmit_telemetry(volatile uint8_t state);
void update_radio_timer_params(volatile uint8_t state);

#endif /* INC_RADIO_COMMANDS_H_ */
