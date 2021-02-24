//
// Created by wenwe on 2021/2/13.
//

#ifndef MINIMAL_FLIGHT_CONTROLLER_GPS_H
#define MINIMAL_FLIGHT_CONTROLLER_GPS_H

#include <memory.h>
#include "usart.h"


enum GPS_MSG {
    GPDTM = 0,
    GPGBS = 1,
    GPGGA = 2,
    GPGLL = 3,
    GPGRS = 4,
    GPGSA = 5,
    GPGST = 6,
    GPGSV = 7,
    GPRMC = 8,
    GPVTG = 9,
    GPZDA = 10,
};

typedef struct GPSInfo {
    float latitude;
    float longitude;
    float height;
    uint8_t n_satellites;
    uint8_t valid;
} GPSInfo;

void gps_mode_close(enum GPS_MSG type);

void gps_mode_open(enum GPS_MSG type);

GPSInfo gps_read();

GPSInfo gps_parse(const char* raw_data);



#endif //MINIMAL_FLIGHT_CONTROLLER_GPS_H
