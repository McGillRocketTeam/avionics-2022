/*
 * gps.h
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef GPS_H_
#define GPS_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#define GPS_DEBUG	0
#define GPSBUFSIZE  128       // GPS buffer size

typedef struct{

    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;

    //Data uart
    UART_HandleTypeDef* uart;

    //User defined functions
    void (*print)(char*);
    void (*tone_freq)(uint32_t duration, uint32_t repeats, uint32_t freq);

} GPS_t;


void GPS_Init(UART_HandleTypeDef* data_uart, void (*gps_print)(char*),
		void (*gps_tone_freq)(uint32_t duration, uint32_t repeats, uint32_t freq));
void GSP_USBPrint(char *data);
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
int GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);
void GPS_check_nonzero_data(float latitude, float longitude, uint8_t *gps_fix_lat, uint8_t *gps_fix_long);
void GPS_Poll(float*, float*, float*);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H_ */
