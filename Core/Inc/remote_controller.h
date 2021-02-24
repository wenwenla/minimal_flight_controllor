//
// Created by WangWen on 2021/2/8.
//

#ifndef MINIMAL_FLIGHT_CONTROLLER_REMOTE_CONTROLLOR_H
#define MINIMAL_FLIGHT_CONTROLLER_REMOTE_CONTROLLOR_H

#include "usart.h"

typedef struct RCInfo {
    uint16_t channel[8];
} RCInfo;

typedef struct RCAttitude {
    float pitch;
    float roll;
    float yaw;
} RCAttitude;

void remote_update();

RCInfo remoter_get_info();

RCAttitude remote_get_attitude(const RCInfo* rc_info);

#endif //MINIMAL_FLIGHT_CONTROLLER_REMOTE_CONTROLLOR_H
