

#ifdef __cplusplus
extern "C" {
#endif

#include<stdint.h>

#define	HB_ENCODE_START  0x0A
#define	HB_ENCODE_PROP 0x0B
#define	HB_ENCODE_SEPAR 0x0C
#define	HB_ENCODE_DEC 0x0D
#define	HB_ENCODE_NEG 0x0E
#define	HB_ENCODE_END 0x0F

#define	HB_DECODE_START 'S'
#define	HB_DECODE_PROP 'P'
#define	HB_DECODE_SEPAR ','
#define	HB_DECODE_DEC '.'
#define	HB_DECODE_NEG '-'
#define	HB_DECODE_END 'E'

void hb_encode_string(uint8_t* buffer, uint8_t len_buffer, uint8_t* encoded_buffer, uint8_t* len_encoded_buffer);
void hb_decode_string(uint8_t* buffer, uint8_t len_buffer_limit, uint8_t* decoded_buffer, uint8_t* len_decoded_buffer);

#ifdef __cplusplus
}
#endif
