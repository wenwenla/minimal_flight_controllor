//
// Created by WangWen on 2021/2/11.
//

#ifndef MINIMAL_FLIGHT_CONTROLLER_MPU6050_H
#define MINIMAL_FLIGHT_CONTROLLER_MPU6050_H

#include "i2c.h"

typedef struct MpuRawData {
    int16_t acc[3];
    int16_t gyro[3];
    int16_t temp;
} MpuRawData;

typedef struct MpuFloatData {
    float acc[3];
    float gyro[3];
    float temp;
} MpuFloatData;

typedef struct Attitude {
    float yaw;
    float pitch;
    float roll;
} Attitude;

HAL_StatusTypeDef mpu6050_init();

MpuRawData mpu6050_get_raw_data();

MpuFloatData mpu6050_process_raw(const struct MpuRawData* p_raw);

Attitude mpu6050_attitude_estimate();

void mpu6050_attitude_reset();

#endif //MINIMAL_FLIGHT_CONTROLLER_MPU6050_H
