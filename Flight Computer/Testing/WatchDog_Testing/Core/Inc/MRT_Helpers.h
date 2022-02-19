/*
 * MRT_Helpers.h
 *
 *  Created on: Jan 7, 2022
 *      Author: Jacoby
 */

#ifndef MRT_HELPERS_F4XX_MRT_HELPERS_H_
#define MRT_HELPERS_F4XX_MRT_HELPERS_H_
#endif /* MRT_HELPERS_F4XX_MRT_HELPERS_H_ */

#include <eeprom.h>

extern uint16_t VirtAddVarTab[NB_OF_VAR]; //Map of variables in internal flash


//Map every value to their address
extern uint8_t reset_flag; //if 0 -> start from beginning, if 1 -> random watchdog reset (if 2-> reset after wakeup??)
extern uint8_t wakeup_flag;


#define RESET_FLAG_ADDRESS VirtAddVarTab[0]
#define WAKEUP_FLAG_ADDRESS VirtAddVarTab[1]


void checkForI2CDevices(UART_HandleTypeDef uart, I2C_HandleTypeDef I2C);
void MRT_getFlags(void);

//void tone(uint32_t duration, uint32_t repeats, TIM_HandleTypeDef htim);
