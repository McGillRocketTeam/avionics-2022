/*
 * startup.h
 *
 *  Created on: Apr 6, 2022
 *      Author: jasper
 */

#ifndef INC_STARTUP_H_
#define INC_STARTUP_H_

#include <stdint.h>

void init_gpio_startup(uint8_t is_watchdog_restart);

#endif /* INC_STARTUP_H_ */
