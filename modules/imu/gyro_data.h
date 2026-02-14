// Temporarily adding to spectre codebase to make application layer compile
// TODO: replace this when making IMU implementation match dev c implementation
#ifndef GYRO_DATA_H

#define GYRO_DATA_H


#include <stdint.h>
#include "main.h"   
#include "bmi088driver.h"

#define Initial_Tick 6300.0f// the tick when gimbal pointing forward
#define Max_Tick  8192.0f
#define delta_t 0.005f//5ms


/**
 * @brief Sensor data structure for gyroscope and accelerometer readings
 * a is accelerometer, g is gyroscope
 * c_ means chassis sensor, g_ means gimbal sensor
 *
 * IMPORTANT: All IMU data now use consistent units:
 *   - Angles: degrees (float)
 *   - Gyroscope: rad/s (float)
 *   - Accelerometer: m/s² (float)
 */
typedef struct {
    // === 云台IMU (BMI088) ===
    float g_gx, g_gy, g_gz;      // 云台IMU陀螺仪（rad/s）
    float g_ax, g_ay, g_az;      // 云台IMU加速度计（m/s²）

    // 姿态角度（度）
    float yaw;                   // 当前yaw角度（-180~180）
    float pitch;                 // 当前pitch角度（-90~90）
    float roll;                  // 当前roll角度（-180~180）

    // 多圈累积角度（度）
    float yaw_total_angle;       // yaw累积角度（无限制范围）
    int32_t yaw_round_count;     // yaw圈数计数

    float absolute_angle;        // 兼容旧代码

    // === 底盘IMU (WT61C) - 统一单位为float ===
    float c_roll, c_pitch, c_yaw;    // 姿态角度（度）- 改为float
    float c_gx, c_gy, c_gz;          // 陀螺仪（rad/s）- 改为float并统一单位
    float c_ax, c_ay, c_az;          // 加速度计（m/s²）- 改为float

} SensorData;

void gyro_data_init(void);
void gyro_data_update(SensorData *sensor_data);
float gimbal_absolute_angle(SensorData* sensor_data);

// 陀螺仪零偏校准
void gyro_calibrate(void);

// 校准期间的回调函数类型（在每次采样间隔时调用，用于保持云台位置等）
typedef void (*GyroCalibCallback_t)(void);

// 设置校准回调函数
void gyro_calibrate_set_callback(GyroCalibCallback_t callback);

#endif

