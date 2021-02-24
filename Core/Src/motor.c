//
// Created by WangWen on 2021/2/8.
//

#include <motor.h>

void motor_init() {
    htim2.Instance->CCR1 = 1000;
    htim2.Instance->CCR2 = 1000;
    htim2.Instance->CCR3 = 1000;
    htim2.Instance->CCR4 = 1000;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void motor_set_speed(int motor_id, uint32_t speed) {
    switch (motor_id) {
        case 0:
            htim2.Instance->CCR4 = speed;
            break;
        case 1:
            htim2.Instance->CCR1 = speed;
            break;
        case 2:
            htim2.Instance->CCR3 = speed;
            break;
        case 3:
            htim2.Instance->CCR2 = speed;
            break;
        default:
            break;
    }
}

void motor_set_speeds(const uint32_t *speed) {
    for (int i = 0; i < 4; ++i) {
        motor_set_speed(i, speed[i]);
    }
}
