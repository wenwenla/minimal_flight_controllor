/*
 * logger.c
 *
 *  Created on: Feb 20, 2021
 *      Author: WangWen
 */


#include "logger.h"


static char log_buffer[512];


void logger_reset() {
	uint8_t cmd[7] = {'$', '$', '$'};
	HAL_UART_Transmit(&huart3, cmd, 3, 100);
	HAL_Delay(1000);
	cmd[0] = 'r';
	cmd[1] = 'e';
	cmd[2] = 's';
	cmd[3] = 'e';
	cmd[4] = 't';
	cmd[5] = '\r';
	cmd[6] = '\n';
	HAL_UART_Transmit(&huart3, cmd, 7, 100);
	HAL_Delay(1000);
}

void logger_log(const char* str) {
	int index = 0;
	while (index < 512 && str[index] != 0) {
		log_buffer[index] = str[index];
		++index;
	}
	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, log_buffer, index);
	printf("%d\n", status);
}
