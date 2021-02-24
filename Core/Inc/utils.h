//
// Created by WangWen on 2021/2/16.
//

#ifndef MINIMAL_FLIGHT_CONTROLLER_UTILS_H
#define MINIMAL_FLIGHT_CONTROLLER_UTILS_H

#include <stdint.h>

uint32_t clip(uint32_t val, uint32_t lower_bound, uint32_t upper_bound);

void led_flash_forever(uint8_t hz);

#endif //MINIMAL_FLIGHT_CONTROLLER_UTILS_H
