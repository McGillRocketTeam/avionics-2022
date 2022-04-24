#include <MRT_iridium.h>
#include "IridiumSBD.h"

#ifdef __cplusplus
extern "C" {
#endif

//C++ objects
static IridiumSBD *E_T = NULL;

//C structs
struct HIRIDIUM hiridium;


void MRT_Iridium_Constructor(){
	if (E_T==NULL){
		E_T = new IridiumSBD();
	}
}

void MRT_Iridium_Destructor(){
	if (E_T!=NULL){
		E_T = NULL;
	}
}


struct HIRIDIUM MRT_Iridium_Init(uint8_t timeout, uint8_t i2c_bus, void (*iridium_print)(char*)){
	MRT_Iridium_Constructor();
	E_T->MRT_Iridium_setup(timeout, i2c_bus, iridium_print);
	HIRIDIUM iridium_handler;
	iridium_handler.CSQ = &MRT_Iridium_CSQ;
	iridium_handler.getIMEI = &MRT_Iridium_getIMEI;
	iridium_handler.getNetworkAvailability = &MRT_Iridium_NetworkAvailability;
	iridium_handler.getTime = &MRT_Iridium_getTime;
	iridium_handler.sendMessage = &MRT_Iridium_sendMessage;
	return iridium_handler;
}

bool MRT_Iridium_Deinit(void){
	bool b = E_T->MRT_Iridium_shutdown();
	MRT_Iridium_Destructor();
	return b;
}

void MRT_Iridium_ErrorMessage(uint8_t error){
	return E_T->MRT_Iridium_ErrorMessage(error);
}

bool MRT_Iridium_getIMEI(void){
	return E_T->MRT_Iridium_getIMEI();
}


int MRT_Iridium_CSQ(){
	return E_T->MRT_Iridium_CSQ();
}

bool MRT_Iridium_NetworkAvailability(){
	return E_T->MRT_Iridium_NetworkAvailability();
}


bool MRT_Iridium_getTime(void){
	return E_T->MRT_Iridium_getTime();
}

bool MRT_Iridium_sendMessage(char* msg){
	return E_T->MRT_Iridium_sendMessage(msg);
}

bool MRT_Iridium_sendReceive(){
	return E_T->MRT_Iridium_sendReceive();
}


#ifdef __cplusplus
}
#endif






