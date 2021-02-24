//
// Created by wenwe on 2021/2/13.
//

#ifndef MINIMAL_FLIGHT_CONTROLLER_OLED_H
#define MINIMAL_FLIGHT_CONTROLLER_OLED_H

#include "i2c.h"

void oled_init();
void oled_set_row(uint8_t row);
void oled_set_col(uint8_t col);
void oled_cls();
void oled_put_str(const char* p_str, uint8_t start_row, uint8_t start_col);
void oled_update();

#endif //MINIMAL_FLIGHT_CONTROLLER_OLED_H
