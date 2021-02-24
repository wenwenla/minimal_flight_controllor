//
// Created by WangWen on 2021/2/8.
//

#ifndef MINIMAL_FLIGHT_CONTROLLER_MOTOR_H
#define MINIMAL_FLIGHT_CONTROLLER_MOTOR_H

#include "tim.h"

void motor_init();

void motor_set_speed(int motor_id, uint32_t speed);

void motor_set_speeds(const uint32_t speed[]);

#endif //MINIMAL_FLIGHT_CONTROLLER_MOTOR_H
