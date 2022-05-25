/*
 * state_restoration.h
 *
 *  Created on: Apr 5, 2022
 *      Author: jasper
 */

#ifndef INC_STATE_RESTORATION_H_
#define INC_STATE_RESTORATION_H_

typedef enum fc_state_backup {
	// for simplicity, put them all in separate backup registers
	// if we ever need more, we can pack the bits
	FC_STATE_FLIGHT = 0,
	FC_STATE_ARM_PROP,
	FC_STATE_ARM_RCOV,
	FC_STATE_VR_POWER,
	FC_STATE_VR_RECORDING,
	FC_STATE_ALT_GROUND,			// floats will be rounded to int
	FC_STATE_ALT_APOGEE,
	FC_STATE_ALT_PREV,
	FC_STATE_FLASH_WRITE_ADDRESS
} fc_state_backup;


// functions
void init_backup_regs(void);
void restore_fc_states(void);

uint32_t get_backup_state(fc_state_backup state);
void set_backup_state(fc_state_backup state, uint32_t value);

#endif /* INC_STATE_RESTORATION_H_ */
