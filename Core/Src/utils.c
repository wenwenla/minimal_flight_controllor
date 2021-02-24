//
// Created by wenwe on 2021/2/16.
//

#include <utils.h>
#include <math.h>
#include "gpio.h"

uint32_t clip(uint32_t val, uint32_t lower_bound, uint32_t upper_bound) {
    if (val < lower_bound) val = lower_bound;
    else if (val > upper_bound) val = upper_bound;
    return val;
}

void led_flash_forever(uint8_t hz) {
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(round(1000. / hz));
    }
}
