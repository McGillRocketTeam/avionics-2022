#include "MRT_ADS1015.h"
#include "SparkFun_ADS1015_Arduino_Library.h"
#include <stdio.h>


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
	ADCS->MRT_adc1015_setup(i2c_bus, adc1015_print);
	HADC1015 adc1015_handler;
	adc1015_handler.getDifferential = &MRT_ADC1015_getDifferential;
	adc1015_handler.getSingleEnded = &MRT_ADC1015_getSingleEnded;
	return adc1015_handler;
}

uint16_t MRT_ADC1015_getDifferential(void){
	uint16_t temp = ADCS->getDifferential(ADS1015_CONFIG_MUX_DIFF_P0_N1);
    return temp;
}

uint16_t MRT_ADC1015_getSingleEnded(uint8_t port){
	uint16_t temp = ADCS->getSingleEnded(port);
	return temp;
}

