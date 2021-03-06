/*
 * gps.c
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |---------------------------------------------------------------------------------
 */


#include <stdio.h>
#include <string.h>
//#include <usart.h>
#include "gps.h"
#include "main.h"

#if (GPS_DEBUG == 1)
#include <usbd_cdc_if.h>
#endif

uint8_t rx_buffer[100];
uint8_t rx_current = 0;
uint8_t rx_index = 0;

extern uint8_t gps_fix_lat;
extern uint8_t gps_fix_long; // beep when we get fix

GPS_t GPS;

#if (GPS_DEBUG == 1)
void GPS_print(char *data){
	char buf[GPSBUFSIZE] = {0,};
	sprintf(buf, "%s\n", data);
	CDC_Transmit_FS((unsigned char *) buf, (uint16_t) strlen(buf));
}
#endif


/*
 * TODO MRT code
 */
#define LED1_GPIO_Port	((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL))//OUT_LED1_GPIO_Port
#define LED1_Pin	((uint16_t)0x0002)//OUT_LED1_Pin
#define LED2_GPIO_Port	((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL))//OUT_LED2_GPIO_Port
#define LED2_Pin	((uint16_t)0x0004)//OUT_LED2_Pin
#define LED3_GPIO_Port	((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL))//OUT_LED3_GPIO_Port
#define LED3_Pin	(uint16_t)0x0008//OUT_LED3_Pin

UART_HandleTypeDef* GPS_USART;
UART_HandleTypeDef* SERIAL_USART;



void GPS_Poll(float *latitude, float *longitude, float *time)
{
	uint16_t max_loop_count = 100;
	uint16_t loop_count = 0;
	int done = 0;
	while(loop_count < max_loop_count && !done){
		HAL_UART_Receive(GPS_USART, (uint8_t*)&rx_current, 1, 100);
		//HAL_UART_Transmit(&huart1, (uint8_t*)&rx_current, 1, 100);
		if (rx_current != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_current;
		} else {
			if(GPS_validate((char*) rx_buffer)){
				if(GPS_parse((char*) rx_buffer)){
					*latitude = GPS.dec_latitude;
					*longitude = GPS.dec_longitude;
					*time = GPS.utc_time;
					done = 1;
				}
			}
			rx_index = 0;
			memset(rx_buffer, 0, sizeof(rx_buffer));
		}

		// f437 usart doesnt have these flags in hardware, use software to clear the flags
		// (check docstring for __HAL_UART_CLEAR_FLAG function)
		__HAL_UART_CLEAR_OREFLAG(GPS_USART);
		__HAL_UART_CLEAR_NEFLAG(GPS_USART);
		__HAL_UART_CLEAR_PEFLAG(GPS_USART);
		__HAL_UART_CLEAR_FEFLAG(GPS_USART);

		loop_count++;
	}
}

int GPS_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;

    i=0;
    calculated_check=0;

    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;

    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }

    if(i >= 75){
        return 0;// the string was too long so return an error
    }

    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found there for invalid

    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
}

int GPS_parse(char *GPSstrParse){
    if(!strncmp(GPSstrParse, "$GNGGA", 6)){
    	if (sscanf(GPSstrParse, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.lock, &GPS.satelites, &GPS.hdop, &GPS.msl_altitude, &GPS.msl_units) >= 1){
    		GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
    		GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
    		return 1;
    	}
    }
    else if (!strncmp(GPSstrParse, "$GNRMC", 6)){
    	if(sscanf(GPSstrParse, "$GNRMC,%f,%f,%c,%f,%c,%f,%f,%d", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.speed_k, &GPS.course_d, &GPS.date) >= 1){
    		GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
    		GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
    		return 1;
    	}


    }
    else if (!strncmp(GPSstrParse, "$GNGLL", 6)){
        if(sscanf(GPSstrParse, "$GNGLL,%f,%c,%f,%c,%f,%c", &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.utc_time, &GPS.gll_status) >= 1){
        	GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
        	GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
        	return 1;
        }

    }
    else if (!strncmp(GPSstrParse, "$GNVTG", 6)){
        if(sscanf(GPSstrParse, "$GNVTG,%f,%c,%f,%c,%f,%c,%f,%c", &GPS.course_t, &GPS.course_t_unit, &GPS.course_m, &GPS.course_m_unit, &GPS.speed_k, &GPS.speed_k_unit, &GPS.speed_km, &GPS.speed_km_unit) >= 1)
            return 0;
    }
    return 0;
}

float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}

void GPS_check_nonzero_data(float latitude, float longitude, uint8_t *gps_fix_lat, uint8_t *gps_fix_long)
{
	if (latitude == 0)
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);

		if (*gps_fix_lat == 1)
		{
			*gps_fix_lat = 0;
			tone(200, 2);
		}
	}
	else
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		if (*gps_fix_lat == 0)
		{
			*gps_fix_lat = 1;
			tone(200, 4);
		}
	}

	if (longitude == 0)
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		if (*gps_fix_long == 1)
		{
			*gps_fix_long = 0;
			tone(200, 2);
		}
	}
	else
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
		if (*gps_fix_long == 0)
		{
			*gps_fix_long = 1;
			tone(200, 4);
		}
	}
}




/*
 * TODO MRT code
 */

void GPS_init(UART_HandleTypeDef* data_uart, UART_HandleTypeDef* transmit_uart){
	GPS_USART = data_uart;
	SERIAL_USART = transmit_uart;
	HAL_UART_Transmit(SERIAL_USART,"\r\n\r\nGPS Init\r\n\r\n",16,HAL_MAX_DELAY);
}
