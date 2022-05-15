/*
 * MRT_telemetry.h
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#ifndef MRT_TELEMETRY_INC_MRT_TELEMETRY_H_
#define MRT_TELEMETRY_INC_MRT_TELEMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <radio_commands.h>//Not needed here put nice to import along telemetry

void MRT_radio_Init(void);
void MRT_radio_tx(char* buffer);
void MRT_radio_rx(char* buffer, uint8_t size, uint16_t timeout);
void MRT_radio_send_ack(radio_command cmd);

void MRT_TELEMETRY_Init(void);
void MRT_TELEMETRY_Deinit(void);



#ifdef __cplusplus
}
#endif

#endif /* MRT_TELEMETRY_INC_MRT_TELEMETRY_H_ */
