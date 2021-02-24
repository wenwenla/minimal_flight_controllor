/*
 * lpf.h
 *
 *  Created on: Feb 21, 2021
 *      Author: WangWen
 */

#ifndef INC_LPF_H_
#define INC_LPF_H_

#include <stdint.h>

typedef struct LPF {
	float k[5];
	float prev_x[2];
	float prev_y[2];
	uint8_t now_cnt;
} LPF;


void lpf_init(LPF* p_lpf, float sample_freq, float cutoff_freq);

float lpf_get_value(LPF* p_lpf, float x);


typedef struct FirstOrderLPF {
	float k;
	float prev_y;
} FirstOrderLPF;

void lpf_first_order_init(FirstOrderLPF* p_lpf);

float lpf_first_order_get_value(FirstOrderLPF* p_lpf, float x);

#endif /* INC_LPF_H_ */
