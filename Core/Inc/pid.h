//
// Created by WangWen on 2021/2/16.
//

#ifndef MINIMAL_FLIGHT_CONTROLLER_PID_H
#define MINIMAL_FLIGHT_CONTROLLER_PID_H

#include <stdint.h>

#define PID_PITCH_KP (300.0f)
#define PID_PITCH_KI (0.0f)
#define PID_PITCH_KD (70.0f)

#define PID_ROLL_KP (300.0f)
#define PID_ROLL_KI (0.0f)
#define PID_ROLL_KD (70.0f)

#define PID_YAW_KP (300.0f)
#define PID_YAW_KI (0.0f)
#define PID_YAW_KD (40.0f)

#define MAX_PITCH_I_ERROR (40.0f)
#define MAX_ROLL_I_ERROR (40.0f)
#define MAX_YAW_I_ERROR (20.0f)

#define MAX_PITCH_OUTPUT (400.0f)
#define MAX_ROLL_OUTPUT (400.0f)
#define MAX_YAW_OUTPUT (400.f)

void pid_init();

float pid_update_pitch(float now, float target);

float pid_update_roll(float now, float target);

float pid_update_yaw(float now, float target);


float g_pitch_p_out;
float g_pitch_i_out;
float g_pitch_d_out;

float g_roll_p_put;
float g_roll_i_out;
float g_roll_d_out;

float g_yaw_p_out;
float g_yaw_i_out;
float g_yaw_d_out;

#endif //MINIMAL_FLIGHT_CONTROLLER_PID_H
