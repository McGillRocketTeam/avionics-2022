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
	ARM_PROP = 2,
	ARM_RCOV = 3,
	VR_POWER_ON = 4,
	VR_REC_START = 5,
	VR_REC_STOP = 6,	// should not be used at comp
	VR_POWER_OFF = 7,	// should not be used at comp
} radio_command;


// functions
radio_command xtend_parse_dma_command(void);
void rocket_launch(void);
void arming_propulsion(void);
void arming_recovery(void);


#endif /* INC_RADIO_COMMANDS_H_ */
