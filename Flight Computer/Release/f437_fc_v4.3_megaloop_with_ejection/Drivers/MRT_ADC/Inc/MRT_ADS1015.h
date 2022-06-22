#ifndef MRT_ADS1015_H_
#define MRT_ADS1015_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>


struct HADC1015 {

	//Data

	//Functions
	uint16_t (*getDifferential)(void);
	uint16_t (*getSingleEnded)(uint8_t port);
};

extern struct HADC1015 hadc1015;



struct HADC1015 MRT_ADC1015_Init(uint8_t i2c_bus, void (*adc1015_print)(char*));
uint16_t MRT_ADC1015_getDifferential(void);
uint16_t MRT_ADC1015_getSingleEnded(uint8_t port);



#ifdef __cplusplus
}
#endif

#endif
