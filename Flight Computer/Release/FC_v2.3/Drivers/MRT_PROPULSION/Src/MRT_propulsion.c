/*
 * MRT_propulsion.c
 *
 *  Created on: Apr 24, 2022
 *      Author: Jacoby
 */

#include <MRT_setup.h>
#include <MRT_helpers.h>
#include <MRT_propulsion.h>
#include <MRT_memory.h>
#include <MAX31855.h>

//Global variables
float thermocouple_temperature;
float transducer_voltage;
uint8_t valve_status;

char msg_buffer_pr[50];


//Private functions prototypes
float MRT_prop_poll_pressure_transducer(ADC_HandleTypeDef* hadc);


//**************************************************//
//PUBLIC FUNCTIONS

void MRT_pollPropulsion(void){
	MRT_getThermoTemp();
	MRT_getTransducerVoltage();
	MRT_getValveStatus();
}

void MRT_getThermoTemp(void){
	thermocouple_temperature = Max31855_Read_Temp();
}

void MRT_getTransducerVoltage(void){
	transducer_voltage = MRT_prop_poll_pressure_transducer(&TRANSDUCER_ADC);
}

void MRT_getValveStatus(void){
	valve_status = HAL_GPIO_ReadPin(IN_Prop_ActuatedVent_Feedback_GPIO_Port,IN_Prop_ActuatedVent_Feedback_Pin);
}

// formats propulsion telemetry string using sprintf
void MRT_formatPropulsion(void) {
	memset(msg_buffer_pr, 0, 50);
	//tank_pressure = convert_prop_tank_pressure(); // convert buffered readings to voltage TODO??
	sprintf(msg_buffer_pr, "P,%03.2f,%03.2f,%d,%02d,%02d,%lu,E\r\n",
			transducer_voltage, thermocouple_temperature, valve_status,
			prev_min, prev_sec, prev_subsec);
}

//**************************************************//
//PRIVATE FUNCTIONS

/*
 * Get the pressure transducer voltage (poll ADC)
 */
float MRT_prop_poll_pressure_transducer(ADC_HandleTypeDef* hadc) {
	// reading adc
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	uint32_t pressure_sensor_raw = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);

	float voltage = (float) (pressure_sensor_raw / 4095.0) * 3.3; // assuming 12 bits

	return voltage;
}

