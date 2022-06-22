/*
 * gps.h
 *
 *  Originally created by Bulanov Konstantin  (leech001@gmail.com).
 *  modified by McGill Rocket Team avionics team.
 */

#ifndef MRT_GPS_NEO_M8_INC_GPS_H
#define MRT_GPS_NEO_M8_INC_GPS_H

#include "stm32f4xx.h"

#define	GPS_USART	huart6
#define GPSBUFSIZE  128       		// GPS buffer size
#define GPS_RX_DMA_BUF_LEN	175		// characters

typedef struct {

    // calculated values
    double dec_longitude;
    double dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    double nmea_longitude;
    double nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimum Specific GNS Data
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

// functions
HAL_StatusTypeDef GPS_StartDMA(void);
int GPS_validate(char *nmeastr);
int GPS_parse(char *GPSstrParse);
double GPS_nmea_to_dec(double deg_coord, char nsew);
void GPS_check_nonzero_data(float latitude, float longitude);
void GPS_Poll(double*, double*, float*);
void GPS_ParseBuffer(void);

// variables
extern volatile char gps_rx_buf[GPS_RX_DMA_BUF_LEN+1];
extern GPS_t GPS;

#endif /* MRT_GPS_NEO_M8_INC_GPS_H */
