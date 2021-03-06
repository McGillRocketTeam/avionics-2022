/*
 * gps.h
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 */
#include "main.h"
#define GPS_DEBUG	0
#define GPSBUFSIZE  128       // GPS buffer size

/*
 * TODO MRT code
 */
UART_HandleTypeDef* GPS_USART;


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
} GPS_t;

#if (GPS_DEBUG == 1)
void GPS_print(char *data);
#endif

void GPS_Init(UART_HandleTypeDef* data_uart, UART_HandleTypeDef* transmit_uart);
void GSP_USBPrint(char *data);
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
int GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);
void GPS_check_nonzero_data(float latitude, float longitude, uint8_t *gps_fix_lat, uint8_t *gps_fix_long);
void GPS_Poll(float*, float*, float*);

extern void tone(uint32_t duration, uint32_t repeats);
