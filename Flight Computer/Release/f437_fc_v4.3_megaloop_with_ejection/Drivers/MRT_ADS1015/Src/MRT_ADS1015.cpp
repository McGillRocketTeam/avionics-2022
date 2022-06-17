

#include "MRT_ADS1015.h"
#include "SparkFun_ADS1015_Arduino_Library.h"


//C++ objects
static ADS1015 *ADCS = NULL;

//C structs
struct HADC1015 hadc1015;



void MRT_ADC1015_Constructor(){
	if (ADCS==NULL){
		ADCS = new ADS1015();
	}
}

void MRT_Iridium_Destructor(){
	if (ADCS!=NULL){
		ADCS = NULL;
	}
}

struct HADC1015 MRT_ADC1015_Init(uint8_t i2c_bus, void (*adc1015_print)(char*)){
	MRT_ADC1015_Constructor();
	HADC1015 adc1015_handler;
	adc1015_handler.getDifferential = &MRT_ADC1015_getDifferential;
	return adc1015_handler;
}

uint16_t MRT_ADC1015_getDifferential(void){
	return ADCS->getDifferential();
}