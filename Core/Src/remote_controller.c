//
// Created by WangWen on 2021/2/8.
//

#include "remote_controller.h"
#include <math.h>


volatile uint8_t is_reading = 0;
uint8_t rc_buffer[64];
uint8_t rc_data[32];
RCInfo g_rc_info;
uint32_t prev_reading_tick = 0;

static void remote_error() {
    for (int i = 0; i < 8; ++i) {
        g_rc_info.channel[i] = 1000;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance) {
		is_reading = 0;
	}
}

void remote_update()
{
    if (is_reading) {
        uint32_t now = HAL_GetTick();
        if (now - prev_reading_tick > 1000) {
            // error
            remote_error();
        }
    } else {
        // find header
        int start_index = -1;
        for (int i = 0; i < 32; ++i) {
            if (rc_buffer[i] == 0x20 && rc_buffer[i + 1] == 0x40) {
                start_index = i;
                break;
            }
        }
        // copy data
        if (start_index != -1) {
            for (int i = start_index; i < start_index + 32; ++i) {
                rc_data[i - start_index] = rc_buffer[i];
            }
        }

        // read new data
        HAL_UART_Receive_DMA(&huart1, rc_buffer, 64);
        is_reading = 1;

        if (start_index == -1) {
            // find no data
            // lock the uav for safety
            remote_error();
        } else {
            // check-sum
            uint16_t check_sum = 0;
            for (int i = 0; i < 30; ++i) {
                check_sum += rc_data[i];
            }
            check_sum = check_sum ^ 0xFFFF;

            if ((check_sum & 0xFF) != rc_data[30] || (check_sum >> 8) != rc_data[31]) {
                // drop data
            } else {
                for (int i = 0; i < 8; ++i) {
                    g_rc_info.channel[i] = (uint16_t) rc_data[2 + 2 * i] | (((uint16_t) rc_data[3 + 2 * i] & 0xf) << 8);
                }
                prev_reading_tick = HAL_GetTick();
            }
        }
    }
}

RCInfo remoter_get_info() {
    return g_rc_info;
}

RCAttitude remote_get_attitude(const RCInfo *rc_info) {
    RCAttitude result;
    result.pitch = (1500.f - (float)rc_info->channel[1]) / 2000.f * M_PI; // [-pi/4, pi/4]
    result.roll = ((float)rc_info->channel[0] - 1500.f) / 2000.f * M_PI;  // [-pi/4, pi/4]
    result.yaw = ((float)rc_info->channel[3] - 1500.f) / 2000.f * M_PI;   // [-pi/4, pi/4]
    return result;
}

