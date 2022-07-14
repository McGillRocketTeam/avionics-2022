#include "half_byte_encoder.h"

uint8_t hb_compress(uint8_t character) {
	if (character == HB_DECODE_START) {
		return(HB_ENCODE_START); //00001100
	}
	else if (character == HB_DECODE_END) {
		return(HB_ENCODE_END); //00001101
	}
	else if (character == HB_DECODE_SEPAR) {
		return(HB_ENCODE_SEPAR); //00001010
	}
	else if (character == HB_DECODE_PROP) {
		return(HB_ENCODE_PROP); //00001011
	}
	else if (character == HB_DECODE_DEC) {
		return(HB_ENCODE_DEC); //00001111
	}
	else if (character == HB_DECODE_NEG) {
		return(HB_ENCODE_NEG); //00001110
	}
	else {
		return character - '0';
	}
}

uint8_t hb_decompress(uint8_t character) {
	if (character == HB_ENCODE_START) {
		return(HB_DECODE_START); //00001100
	}
	else if (character == HB_ENCODE_END) {
		return(HB_DECODE_END); //00001101
	}
	else if (character == HB_ENCODE_SEPAR) {
		return(HB_DECODE_SEPAR); //00001010
	}
	else if (character == HB_ENCODE_PROP) {
		return(HB_DECODE_PROP); //00001011
	}
	else if (character == HB_ENCODE_DEC) {
		return(HB_DECODE_DEC); //00001111
	}
	else if (character == HB_ENCODE_NEG) {
		return(HB_DECODE_NEG); //00001110
	}
	else {
		return character + '0';
	}
}

void hb_encode_string(uint8_t* buffer, uint8_t len_buffer, uint8_t* encoded_buffer, uint8_t* len_encoded_buffer) {

	//dont encode acks and commands
	if(buffer[0] != HB_DECODE_START && buffer[0] != HB_DECODE_PROP) {
		memcpy(encoded_buffer, buffer, len_buffer);
		*len_encoded_buffer = len_buffer;
		return;
	}

	uint8_t index_buffer = 0; //index of the unencoded string
	*len_encoded_buffer = (len_buffer + 1) >> 1; // size of encoded message (len_buffer + 1) / 2
	uint8_t index_encoded = 0; //index of the encoded string

	for (index_buffer = 0; index_buffer < len_buffer; index_buffer += 2) {
		encoded_buffer[index_encoded] = hb_compress(buffer[index_buffer]) << 4 | hb_compress(buffer[index_buffer + 1]); //bitwise or
		index_encoded++;
	}

	//extra character in case of odd length, add two ending code ('e')
	if (index_buffer > len_buffer) {
		uint8_t lastCharIndex = index_buffer - 2;
		encoded_buffer[index_encoded - 1] = hb_compress(buffer[lastCharIndex]) << 4 | hb_compress(buffer[lastCharIndex]);
	}
}

void hb_decode_string(uint8_t* buffer, uint8_t len_buffer_limit, uint8_t* decoded_buffer, uint8_t* len_decoded_buffer) {

	uint8_t firstChar = buffer[0] >> 4;
	//dont decode acks and commands
	if(firstChar != HB_ENCODE_START && firstChar != HB_ENCODE_PROP) {
		memcpy(decoded_buffer, buffer, len_buffer_limit);
		*len_decoded_buffer = len_buffer_limit;
		return;
	}

	uint8_t index_buffer = 0; //index of the encoded string
	uint8_t index_decoded = 0;  //index of the decoded string

	for (index_buffer = 0; index_buffer < len_buffer_limit; index_buffer++) {
		decoded_buffer[index_decoded++] = hb_decompress(buffer[index_buffer] >> 4); //leftmost char (MSBits)
		decoded_buffer[index_decoded++] = hb_decompress(buffer[index_buffer] & 0xf); //rightmost char (LSBits)
		if(decoded_buffer[index_decoded-1] == HB_DECODE_END){
			break;
		}
	}

  *len_decoded_buffer = index_decoded; //size of decoded message

	//check for double e
	if (decoded_buffer[index_decoded - 2] == HB_DECODE_END) {
		decoded_buffer[index_decoded - 1] = '\0';
   *len_decoded_buffer = index_decoded - 1; //size of decoded message
	}
}
