/*
 * payload.h
 *
 *  Created on: Jun 10, 2022
 *      Author: thomas
 */

#ifndef MRT_PAYLOAD_H
#define MRT_PAYLOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#define PAYLOAD_COUNT 244 //Actual number /4

extern uint8_t payload_init_success;
extern uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];


void MRT_payloadInit(void);
uint8_t MRT_payloadPoll(void);



#ifdef __cplusplus
}
#endif

#endif
