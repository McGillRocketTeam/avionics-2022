/*
 * Even though we have an api, you still have to set the
 * project a c++ (don't worry about main being a .c file)
 */
#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>


#ifndef MRT_IRIDIUM_F4XX_INC_IRIDIUMSBD_STATIC_API_H_
#define MRT_IRIDIUM_F4XX_INC_IRIDIUMSBD_STATIC_API_H_

#ifdef __cplusplus
extern "C" {
#endif

//Helper functions
void MRT_Static_Iridium_Constructor();
void MRT_Static_Iridium_Destructor();

//User functions
uint8_t MRT_Static_Iridium_Setup(UART_HandleTypeDef huart, uint8_t timeout, uint8_t i2c_bus);
bool MRT_Static_Iridium_Shutdown(void);
void MRT_Static_Iridium_ErrorMessage(uint8_t error);
bool MRT_Static_Iridium_getIMEI(void);
int MRT_Static_Iridium_CSQ();
bool MRT_Static_Iridium_NetworkAvailability();
bool MRT_Static_Iridium_getTime(void);
bool MRT_Static_Iridium_sendMessage(char* msg);
bool MRT_Static_Iridium_sendReceive();


#ifdef __cplusplus
}
#endif


#endif /* MRT_IRIDIUM_F4XX_INC_IRIDIUMSBD_STATIC_API_H_ */
