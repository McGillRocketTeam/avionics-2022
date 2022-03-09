/*************************************************************************************
 Title	 :  MAXIM Integrated MAX31855 Library for STM32 Using HAL Libraries
 Author  :  Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>
 Software:  STM32CubeIDE
 Hardware:  Any STM32 device
*************************************************************************************/
#include"MAX31855.h"
SPI_HandleTypeDef hspi4;

// ------------------- Variables ----------------

uint8_t Error=0;                                      // Thermocouple Connection acknowledge Flag
uint32_t sign=0;									  // Sign bit
uint8_t DATARX[4];                                    // Raw Data from MAX6675
//uint8_t DATATX = {0xFF, 0xFF, 0xFF, 0xFF};                                    // Raw Data from MAX6675

// ------------------- Functions ----------------
void Max31855_Read_Temp(void) {
	int Temp = 0;                                        // Temperature Variable
	HAL_GPIO_WritePin(SSPORT, SSPIN, GPIO_PIN_RESET); // Low State for SPI Communication
	HAL_SPI_Receive(&hspi4, DATARX, 4, 1000);                // DATA Transfer
	HAL_GPIO_WritePin(SSPORT, SSPIN, GPIO_PIN_SET); // High State for SPI Communication


	uint32_t v = DATARX[3] | (DATARX[2] << 8) | (DATARX[1] << 16) | (DATARX[0] << 24);

	Error = v & 0x07;								  // Error Detection



		sign = (DATARX[0] & (0x80)) >> 7;					// Sign Bit calculation

		if (DATARX[3] & 0x07)								 // Returns Error Number
			THERMO_TEMP = (-1 * (DATARX[3] & 0x07));

		else if (sign == 1) {								// Negative Temperature
			Temp = (DATARX[0] << 6) | (DATARX[1] >> 2);
			Temp &= 0b01111111111111;
			Temp ^= 0b01111111111111;
			THERMO_TEMP = ((float) -Temp / 4);
		}

		else												 // Positive Temperature
		{
			Temp = (DATARX[0] << 6) | (DATARX[1] >> 2);
			THERMO_TEMP = ((float) Temp / 4.0);
		}


	/*
	if (v & 0x7) {
		// uh oh, a serious problem!
		return -99999;
	}

	if (v & 0x80000000) {
		// Negative value, drop the lower 18 bits and explicitly extend sign bits.
		v = 0xFFFFC000 | ((v >> 18) & 0x00003FFF);
	} else {
		// Positive value, just drop the lower 18 bits.
		v >>= 18;
	}
	// Serial.println(v, HEX);

	double centigrade = v;

	// LSB = 0.25 degrees C
	centigrade *= 0.25;
	return centigrade;
	*/
}
