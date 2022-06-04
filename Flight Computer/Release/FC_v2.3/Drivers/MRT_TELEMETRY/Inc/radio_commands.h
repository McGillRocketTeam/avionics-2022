/*
 * radio_commands.h
 *
 *  Created on: Mar 1, 2022
 *      Author: jasper
 */

#ifndef INC_RADIO_COMMANDS_H_
#define INC_RADIO_COMMANDS_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum radio_command {
	LAUNCH = 1,
	ARM_PROP,
	ARM_RCOV,
	DISARM_PROP,
	DISARM_RCOV,
	DUMP_POWER_ON,
	DUMP_POWER_OFF,
	VR_POWER_ON,
	VR_REC_START,
	VR_REC_STOP,	// should not be used at comp
	VR_POWER_OFF,	// should not be used at comp
	PM_12_EN_ON,
	PM_12_EN_OFF

} radio_command;


// functions
radio_command radio_parse_command(char* rx_buf);
void execute_parsed_command(radio_command cmd);
void rocket_launch(void);
void arming_propulsion(void);
void arming_recovery(void);
void disarm_propulsion(void);
void disarm_recovery(void);
void dump_power_on(void);
void dump_power_off(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_RADIO_COMMANDS_H_ */
