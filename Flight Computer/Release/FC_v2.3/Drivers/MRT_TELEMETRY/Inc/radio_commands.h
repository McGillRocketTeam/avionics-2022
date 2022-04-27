/*
 * radio_commands.h
 *
 *  Created on: Mar 1, 2022
 *      Author: jasper
 */

#ifndef INC_RADIO_COMMANDS_H_
#define INC_RADIO_COMMANDS_H_

typedef enum radio_command {
	LAUNCH = 1,
	ARM_PROP,
	ARM_RCOV,
	DISARM_PROP,
	DISARM_RCOV,
	VR_POWER_ON,
	VR_REC_START,
	VR_REC_STOP,	// should not be used at comp
	VR_POWER_OFF,	// should not be used at comp

} radio_command;


// functions
radio_command radio_parse_command(char* rx_buf);
void execute_parsed_command(radio_command cmd);
void rocket_launch(void);
void arming_propulsion(void);
void arming_recovery(void);
void disarm_propulsion(void);
void disarm_recovery(void);

#endif /* INC_RADIO_COMMANDS_H_ */
