/*
 * Even though we have an api, you still have to set the
 * project a c++ (don't worry about main being a .c file)
 */


#ifndef MRT_IRIDIUM_F4XX_INC_IRIDIUMSBD_STATIC_API_H_
#define MRT_IRIDIUM_F4XX_INC_IRIDIUMSBD_STATIC_API_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>

struct HIRIDIUM{

	//Data

	//Functions
	bool (*getIMEI)(void);
	bool (*getTime)(void);
	bool (*getNetworkAvailability)(void);
	int (*CSQ)(void);
	bool (*sendMessage)(char*);
};

extern struct HIRIDIUM hiridium;


//User functions
struct HIRIDIUM MRT_Iridium_Init(uint8_t timeout, uint8_t i2c_bus, void (*iridium_print)(char*));
bool MRT_Iridium_Deinit();
void MRT_Iridium_ErrorMessage(uint8_t error);
bool MRT_Iridium_getIMEI();
int MRT_Iridium_CSQ();
bool MRT_Iridium_NetworkAvailability();
bool MRT_Iridium_getTime();
bool MRT_Iridium_sendMessage(char* msg);
bool MRT_Iridium_sendReceive();



#ifdef __cplusplus
}
#endif


#endif /* MRT_IRIDIUM_F4XX_INC_IRIDIUMSBD_STATIC_API_H_ */
