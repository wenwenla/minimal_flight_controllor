//
// Created by WangWen on 2021/2/11.
//

#include <mpu6050.h>
#include <math.h>


int16_t g_acc_offset[3] = {0}, g_gyro_offset[3] = {0};

static void auto_calibration() {
    MpuRawData raw_data;
    int32_t acc_offset[3] = {0};
    int32_t gyro_offset[3] = {0};
    HAL_Delay(200); // delay 200 ms
    for (int i = 0; i < 1000; ++i) {
        raw_data = mpu6050_get_raw_data();
        for (int j = 0; j < 3; ++j) {
            acc_offset[j] += raw_data.acc[j];
            gyro_offset[j] += raw_data.gyro[j];
        }
    }
    for (int j = 0; j < 3; ++j) {
        g_acc_offset[j] = -(acc_offset[j] / 1000 + (acc_offset[j] % 1000 > 500));
        g_gyro_offset[j] = -(gyro_offset[j] / 1000 + (gyro_offset[j] % 1000 > 500));
    }
    g_acc_offset[2] += 16384;
}

static HAL_StatusTypeDef mpu6050_write_byte(uint8_t register_addr, uint8_t byte) {
    uint8_t p_data[2];
    p_data[0] = register_addr;
    p_data[1] = byte;
    return HAL_I2C_Master_Transmit(&hi2c1, (0x68 << 1), p_data, 2, 1000);
}

static HAL_StatusTypeDef mpu6050_write(uint8_t register_addr, const uint8_t* data, uint32_t len) {
    uint8_t p_data[32];
    p_data[0] = register_addr;
    for (int i = 0; i < len; ++i) {
        p_data[i + 1] = data[i];
    }
    return HAL_I2C_Master_Transmit(&hi2c1, (0x68 << 1), p_data, len + 1, 100);
}

static HAL_StatusTypeDef mpu6050_read(uint8_t register_addr, uint8_t* buffer, uint32_t bytes) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, (0x68 << 1), &register_addr, 1, 100);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(&hi2c1, (0x68 << 1 | 1), buffer, bytes, 100);
}

HAL_StatusTypeDef mpu6050_init() {
    g_acc_offset[0] = g_acc_offset[1] = g_acc_offset[2] = 0;
    g_gyro_offset[0] = g_gyro_offset[1] = g_gyro_offset[2] = 0;

    HAL_StatusTypeDef status = HAL_OK;
    status = mpu6050_write_byte(0x6B, 0x80);
    if (status != HAL_OK) return status;

    HAL_Delay(1000); // delay 1 s

    status = mpu6050_write_byte(0x6B, 0x01); // PWR_MGMT_1
    if (status != HAL_OK) return status;
    status = mpu6050_write_byte(0x19, 0x03); // Sample Rate Divider 1kHz
    if (status != HAL_OK) return status;
    status = mpu6050_write_byte(0x1A,  0); // CONFIG
    if (status != HAL_OK) return status;
    status = mpu6050_write_byte(0x1B, 0x8); // 500 deg / s
    if (status != HAL_OK) return status;
    status = mpu6050_write_byte(0x1C, 0x10); // 8g
    if (status != HAL_OK) return status;

    auto_calibration();
    return HAL_OK;
}

MpuRawData mpu6050_get_raw_data() {
    MpuRawData result;
    uint8_t buffer[14];
    mpu6050_read(0x3B, buffer, 14);
    for (int i = 0; i < 3; ++i) {
        result.acc[i] = ((int16_t)buffer[2 * i] << 8) + buffer[2 * i + 1] + g_acc_offset[i];
    }
    result.temp = ((int16_t)buffer[6] << 8) + buffer[7];
    for (int i = 0; i < 3; ++i) {
        result.gyro[i] = ((int16_t)buffer[2 * i + 8] << 8) + buffer[2 * i + 9] + g_gyro_offset[i];
    }
    return result;
}

MpuFloatData mpu6050_process_raw(const struct MpuRawData *p_raw) {
    MpuFloatData result = {0};
    if (p_raw == NULL) return result;
    result.temp = (float)p_raw->temp / 340.f + 36.35f;
    for (int i = 0; i < 3; ++i) {
        result.acc[i] = (float)p_raw->acc[i] / 32768.f * 8.f;  // 8G
        result.gyro[i] = (float)p_raw->gyro[i] / 32768.f * 500.0f * M_PI / 180.f;
    }
    return result;
}

float prev_gx = 0.f, prev_gy = 0.f, prev_gz = 0.f;
float prev_final_x = 0.0f, prev_final_y = 0.0f, prev_final_z = 0.0f;
uint32_t prev_ts = 0;

void mpu6050_attitude_reset() {
	prev_gx = 0.0f;
	prev_gy = 0.0f;
	prev_gz = 0.0f;
	prev_final_x = 0.0f;
	prev_final_y = 0.0f;
	prev_final_z = 0.0f;
	prev_ts = 0;
}

Attitude mpu6050_attitude_estimate() {
    Attitude result;
    MpuRawData raw = mpu6050_get_raw_data();
    uint32_t now_ts = HAL_GetTick();

    MpuFloatData data = mpu6050_process_raw(&raw);

    float length = sqrtf(data.acc[0] * data.acc[0] + data.acc[1] * data.acc[1] + data.acc[2] * data.acc[2]);
    float theta_x = -atan2f(-data.acc[1] / length, data.acc[2] / length);
    float theta_y = -asinf(data.acc[0] / length);

    float time_dt = (float)(now_ts - prev_ts) * 1e-3f;

    float dt_x = (prev_gx + data.gyro[0]) * 0.5f * time_dt;
    float dt_y = (prev_gy + data.gyro[1]) * 0.5f * time_dt;

    if (prev_ts == 0) {
        prev_final_x = theta_x;
        prev_final_y = theta_y;
        prev_final_z = 0;
    } else {
        prev_final_x = 0.001f * theta_x + 0.999f * (prev_final_x + dt_x);
        prev_final_y = 0.001f * theta_y + 0.999f * (prev_final_y + dt_y);
        prev_final_z += (prev_gz + data.gyro[2]) * 0.5f * time_dt;
    }
    result.pitch = prev_final_x;
    result.roll = prev_final_y;
    result.yaw = prev_final_z;

    prev_gx = data.gyro[0];
    prev_gy = data.gyro[1];
    prev_gz = data.gyro[2];
    prev_ts = now_ts;
    return result;
}
