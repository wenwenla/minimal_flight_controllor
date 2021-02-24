/*
 * lpf.c
 *
 *  Created on: Feb 21, 2021
 *      Author: WangWen
 */

#include "lpf.h"
#include <math.h>


void lpf_init(LPF* p_lpf, float sample_freq, float cutoff_freq) {
	if (p_lpf == NULL) return;
	float ita = 1.0 / tanf(cutoff_freq / sample_freq * (float)M_PI);
	float q = sqrtf(2.0f);
	p_lpf->k[0] = 1.0f / (1.0f + q * ita + ita * ita);
	p_lpf->k[1] = 2 * p_lpf->k[0];
	p_lpf->k[2] = p_lpf->k[0];
	p_lpf->k[3] = 2.0f * (ita * ita - 1.0f) * p_lpf->k[0];
	p_lpf->k[4] = -(1.0f - q * ita + ita * ita) * p_lpf->k[0];
	p_lpf->prev_x[0] = p_lpf->prev_x[1] = 0.0f;
	p_lpf->prev_y[0] = p_lpf->prev_y[1] = 0.0f;
	p_lpf->now_cnt = 0;
}


float lpf_get_value(LPF* p_lpf, float x) {
	if (p_lpf == NULL) return 0.0f;
	float y = 0.0f;
	if (p_lpf->now_cnt <= 1) {
		p_lpf->prev_x[p_lpf->now_cnt] = x;
		p_lpf->prev_y[p_lpf->now_cnt] = x;
		y = x;
	} else {
		y = p_lpf->k[0] * x +
				p_lpf->k[1] * p_lpf->prev_x[1] +
				p_lpf->k[2] * p_lpf->prev_x[0] +
				p_lpf->k[3] * p_lpf->prev_y[1] +
				p_lpf->k[4] * p_lpf->prev_y[0];
		p_lpf->prev_x[0] = p_lpf->prev_x[1];
		p_lpf->prev_x[1] = x;
		p_lpf->prev_y[0] = p_lpf->prev_y[1];
		p_lpf->prev_y[1] = y;
	}
	return y;
}

void lpf_first_order_init(FirstOrderLPF* p_lpf) {
	if (p_lpf == NULL) return;
	p_lpf->k = 0.691f;
	p_lpf->prev_y = 0.0f;
}

float lpf_first_order_get_value(FirstOrderLPF* p_lpf, float x) {
	if (p_lpf == NULL) return 0.0f;
	float result = p_lpf->k * x + (1 - p_lpf->k) * p_lpf->prev_y;
	p_lpf->prev_y = result;
	return result;
}
