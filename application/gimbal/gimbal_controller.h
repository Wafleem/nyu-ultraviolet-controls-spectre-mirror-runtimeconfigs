#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "gyro_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// Gimbal command structure
typedef struct {
    bool enabled;              // Gimbal control enabled
    float pitch_rate;          // Pitch angular rate command (-1.0 to 1.0, normalized)
    float yaw_rate; 
    float yaw_rate_memo;
    float yaw_target_memo;           // Yaw angular rate command (-1.0 to 1.0, normalized)
    bool vision_valid;
    float vision_yaw_err_rad;
    float vision_pitch_err_rad;
    uint32_t vision_ts_ms;
} GimbalCmd;

void last_data(float last_yaw_rate, float last_yaw_target);

/**
 * @brief Pitch control with normalized rate command
 * @param id Motor ID (7=Pitch)
 * @param rate_normalized Normalized pitch rate (-1.0 to 1.0)
 * @param sensor_data Sensor data pointer
 * @return Motor current command
 */

int16_t GimbalController_PitchControl(uint8_t id, float rate_normalized, SensorData* sensor_data);

/**
 * @brief Yaw control with compensation (chassis rotation + gyro feedback)
 * @param rate_normalized Normalized yaw rate (-1.0 to 1.0)
 * @param sensor_data Sensor data pointer
 * @param use_imu_feedback Use IMU gyro for speed feedback (true for spin mode, false for encoder)
 * @return Motor current command
 */
int16_t GimbalController_YawControlWithCompensation(float rate_normalized, SensorData* sensor_data, bool use_imu_feedback);

/**
 * @brief Target angle correction for yaw (chassis compensation)
 * @param sensor_data Sensor data pointer
 */
void GimbalController_TargetAngleCorrection(SensorData* sensor_data);

/**
 * @brief Initialize gimbal application (message subscriptions and control)
 */
void GimbalApp_Init(void);

/**
 * @brief Wait for gimbal to reach initial alignment position
 * @note This function sends gimbal commands and waits for both yaw and pitch
 *       to reach their initial positions before returning.
 *       Timeout: 10 seconds
 */
void Gimbal_WaitForAlignment(void);


#ifdef __cplusplus
}
#endif

#endif // GIMBAL_CONTROLLER_H

