/*
 * half_byte_encoder.h
 *
 *  Created on: May 29, 2022
 *      Author: Jacoby
 */

#ifndef MRT_TELEMETRY_INC_HALF_BYTE_ENCODER_H_
#define MRT_TELEMETRY_INC_HALF_BYTE_ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include<stdint.h>

typedef enum halfbyte_encode {
	HB_ENCODE_START = 0x0A,
	HB_ENCODE_PROP = 0x0B,
	HB_ENCODE_SEPAR = 0x0C,
	HB_ENCODE_DEC = 0x0D,
	HB_ENCODE_NEG = 0x0E,
	HB_ENCODE_END = 0x0F,
};

typedef enum halfbyte_decode {
	HB_DECODE_START = 'S',
	HB_DECODE_PROP = 'P',
	HB_DECODE_SEPAR = ',',
	HB_DECODE_DEC = '.',
	HB_DECODE_NEG = '-',
	HB_DECODE_END = 'E',
};

void hb_encode_string(uint8_t* buffer, uint8_t len_buffer, uint8_t* encoded_buffer, uint8_t* len_encoded_buffer);
void hb_decode_string(uint8_t* buffer, uint8_t len_buffer_limit, uint8_t* decoded_buffer, uint8_t* len_decoded_buffer);

#ifdef __cplusplus
}
#endif

#endif /* MRT_TELEMETRY_INC_HALF_BYTE_ENCODER_H_ */
