#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_driver.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define MAX_YAW_ANGLE           8192.0f
// #define MAX_YAW_ANGLE_PER_SEC   2000.0f
#define MAX_YAW_ANGLE           360.0f
#define MAX_YAW_ANGLE_CHANGE    150.0f
#define MAX_YAW_TARGET_LEAD       50.0f  // deg: how far angle_target can lead actual yaw (tune up if sluggish)
#define MAX_PITCH_ANGLE         8192.0f   // Can be overwritten by robot config
#define MAX_PITCH_ANGLE_CHANGE  1500.0f

// Gimbal command structure
typedef struct {
    bool enabled;              // Gimbal control enabled
    float pitch_rate;          // Pitch angular rate command (-1.0 to 1.0, normalized)
    float yaw_rate; 
    float yaw_rate_memo;
    float yaw_target_memo;           // Yaw angular rate command (-1.0 to 1.0, normalized)
    bool vision_valid;
    bool aimbot_mode;
    float vision_yaw_err_rad;
    float vision_pitch_err_rad;
    uint32_t vision_ts_ms;
} GimbalCmd;

/**
 * @brief Pitch control with normalized rate command
 * @param id Motor ID (7=Pitch)
 * @param rate_normalized Normalized pitch rate (-1.0 to 1.0)
 * @param sensor_data Sensor data pointer
 * @return Motor current command
 */

int16_t GimbalController_PitchControl(Gimbal_Sensor_Data_t* sensor_data);

/**
 * @brief Yaw control with compensation (chassis rotation + gyro feedback)
 * @param rate_normalized Normalized yaw rate (-1.0 to 1.0)
 * @param sensor_data Sensor data pointer
 * @param use_imu_feedback Use IMU gyro for speed feedback (true for spin mode, false for encoder)
 * @return Motor current command
 */
int16_t GimbalController_YawControlWithCompensation(Gimbal_Sensor_Data_t* sensor_data, bool use_imu_feedback);

/**
 * @brief Update gimbal targets using gimbal cmd data
 */
void GimbalController_UpdateTargets(GimbalCmd *cmd, MotorContext_t *yaw, MotorContext_t *pitch);

/**
 * @brief Initialize gimbal application (message subscriptions and control)
 */
void GimbalApp_Init(void);

/**
 * @brief Run gimbal PID control loop.
 */
void GimbalApp_Tick(void);


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

