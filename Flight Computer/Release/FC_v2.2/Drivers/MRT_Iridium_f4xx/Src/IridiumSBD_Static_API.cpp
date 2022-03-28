#include "IridiumSBD_Static_API.h"
#include "IridiumSBD.h"

#ifdef __cplusplus
extern "C" {
#endif

static IridiumSBD *E_T = NULL;


void MRT_Static_Iridium_Constructor(){
	if (E_T==NULL){
		E_T = new IridiumSBD();
	}
}

void MRT_Static_Iridium_Destructor(){
	if (E_T!=NULL){
		E_T = NULL;
	}
}


uint8_t MRT_Static_Iridium_Setup(UART_HandleTypeDef huart, uint8_t timeout, uint8_t i2c_bus){
	MRT_Static_Iridium_Constructor();
	return E_T->MRT_Iridium_setup(huart,timeout, i2c_bus);
}

bool MRT_Static_Iridium_Shutdown(void){
	bool b = E_T->MRT_Iridium_shutdown();
	MRT_Static_Iridium_Destructor();
	return b;
}

void MRT_Static_Iridium_ErrorMessage(uint8_t error){
	return E_T->MRT_Iridium_ErrorMessage(error);
}

bool MRT_Static_Iridium_getIMEI(void){
	return E_T->MRT_Iridium_getIMEI();
}


int MRT_Static_Iridium_CSQ(){
	return E_T->MRT_Iridium_CSQ();
}

bool MRT_Static_Iridium_NetworkAvailability(){
	return E_T->MRT_Iridium_NetworkAvailability();
}


bool MRT_Static_Iridium_getTime(void){
	return E_T->MRT_Iridium_getTime();
}

bool MRT_Static_Iridium_sendMessage(char* msg){
	return E_T->MRT_Iridium_sendMessage(msg);
}

bool MRT_Static_Iridium_sendReceive(){
	return E_T->MRT_Iridium_sendReceive();
}


#ifdef __cplusplus
}
#endif






