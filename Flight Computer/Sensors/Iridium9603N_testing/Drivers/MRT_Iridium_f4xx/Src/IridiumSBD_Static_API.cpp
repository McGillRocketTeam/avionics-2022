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
	E_T->IMEI = (char*) malloc(16*sizeof(char));
}

void MRT_Static_Iridium_Destructor(){
	free(E_T->IMEI);
	if (E_T!=NULL){
		E_T = NULL;
	}
}


uint8_t MRT_Static_Iridium_Setup(UART_HandleTypeDef huart){
	MRT_Static_Iridium_Constructor();
	return E_T->MRT_Iridium_setup(huart);
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


int MRT_Static_Iridium_checkCSQ(bool b){
	return E_T->MRT_Iridium_checkCSQ(b);
}


bool MRT_Static_Iridium_getTime(void){
	return E_T->MRT_Iridium_getTime();
}

#ifdef __cplusplus
}
#endif






