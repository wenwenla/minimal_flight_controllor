/*
 * logger.h
 *
 *  Created on: Feb 20, 2021
 *      Author: wenwe
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "usart.h"
#include "main.h"


void logger_reset();

void logger_log(const char* str);


#endif /* INC_LOGGER_H_ */

