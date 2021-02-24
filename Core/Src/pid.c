//
// Created by WangWen on 2021/2/16.
//

#include "pid.h"

float PITCH_PREV_ERROR = 0.0f;
float PITCH_SUM_ERROR = 0.0f;

float ROLL_PREV_ERROR = 0.0f;
float ROLL_SUM_ERROR = 0.0f;

float YAW_PREV_ERROR = 0.0f;
float YAW_SUM_ERROR = 0.0f;


static float clip_range(float input, float lbound, float rbound) {
	if (input < lbound) input = lbound;
	if (input > rbound) input = rbound;
	return input;
}


void pid_init() {
    PITCH_SUM_ERROR = 0.0f;
    PITCH_PREV_ERROR = 0.0f;

    ROLL_SUM_ERROR = 0.0f;
    ROLL_PREV_ERROR = 0.0f;

    YAW_SUM_ERROR = 0.0f;
    YAW_PREV_ERROR = 0.0f;
}

float pid_update_pitch(float now, float target) {
    float p_error = target - now;
    float d_error = (p_error - PITCH_PREV_ERROR) / (2e-3);
    float i_error = PITCH_SUM_ERROR + p_error;

    i_error = clip_range(i_error, -MAX_PITCH_I_ERROR, MAX_PITCH_I_ERROR);

    PITCH_PREV_ERROR = p_error;
    PITCH_SUM_ERROR = i_error;

    // data record for debug purpose
    g_pitch_p_out = p_error;
    g_pitch_i_out = i_error;
    g_pitch_d_out = d_error;
    // data record end

    float result = PID_PITCH_KP * p_error + PID_PITCH_KI * i_error + PID_PITCH_KD * d_error;
    result = clip_range(result, -MAX_PITCH_OUTPUT, MAX_PITCH_OUTPUT);
    return result;
}

float pid_update_roll(float now, float target) {
    float p_error = target - now;
    float d_error = (p_error - ROLL_PREV_ERROR) / (2e-3);
    float i_error = ROLL_SUM_ERROR + p_error;

    i_error = clip_range(i_error, -MAX_ROLL_I_ERROR, MAX_ROLL_I_ERROR);

    ROLL_PREV_ERROR = p_error;
    ROLL_SUM_ERROR = i_error;

    // data record for debug purpose
    g_roll_p_put = p_error;
    g_roll_i_out = i_error;
    g_roll_d_out = d_error;
    // data record end

    float result = PID_ROLL_KP * p_error + PID_ROLL_KI * i_error + PID_ROLL_KD * d_error;
    result = clip_range(result, -MAX_ROLL_OUTPUT, MAX_ROLL_OUTPUT);
    return result;
}

float pid_update_yaw(float now, float target) {
    float p_error = target - now;
    float d_error = (p_error - YAW_PREV_ERROR) / (2e-3);
    float i_error = YAW_SUM_ERROR + p_error;

    i_error = clip_range(i_error, -MAX_ROLL_I_ERROR, MAX_ROLL_I_ERROR);

    YAW_PREV_ERROR = p_error;
    YAW_SUM_ERROR = i_error;

    // data record for debug purpose
    g_yaw_p_out = p_error;
    g_yaw_i_out = i_error;
    g_yaw_d_out = d_error;
    // data record end

    float result = PID_YAW_KP * p_error + PID_YAW_KI * i_error + PID_YAW_KD * d_error;
    result = clip_range(result, -MAX_YAW_OUTPUT, MAX_YAW_OUTPUT);
    return result;
}
