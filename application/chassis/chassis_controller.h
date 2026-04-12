#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "main.h"
#include "remote_control.h"
#include "motor_feedback.h"
#include "imu.h"
#include "config_types.h"

// Chassis motor count
#define CHASSIS_MOTOR_COUNT 4

#define CHASSIS_STEER_COUNT 2

// Chassis speed limits (RPM) — runtime-switched for supercap boost
#define CHASSIS_SPEED_NORMAL   7000  // Normal max speed (PMM only)
#define CHASSIS_SPEED_SUPERCAP 8500  // Boosted speed when supercap is discharging


// Chassis controller structure
typedef struct {
    // Drive motor target speeds
    float target_speeds[CHASSIS_MOTOR_COUNT];

    // Running state
    bool running;

    // Drive PID controllers
    PID_Controller speed_pids[CHASSIS_MOTOR_COUNT];

    // Drive motor feedbacks
    Motor_Feedback motor_feedbacks[CHASSIS_MOTOR_COUNT];

    // Drive output currents
    int16_t output_currents[CHASSIS_MOTOR_COUNT];

    // -----------------------------
    // Swerve steering (optional)
    // -----------------------------
    float steer_target_angles[CHASSIS_STEER_COUNT];          // encoder ticks
    PID_Controller steer_pids[CHASSIS_STEER_COUNT];          // angle PID
    Motor_Feedback steer_feedbacks[CHASSIS_STEER_COUNT];     // steer feedback
    int16_t steer_output_currents[CHASSIS_STEER_COUNT];      // steer currents
} ChassisController;


/**
 * @brief Initialize chassis controller
 * @param controller Chassis controller pointer
 */
void ChassisController_Init(ChassisController *controller);

/**
 * @brief Update chassis control logic
 * @param controller Chassis controller pointer
 * @param sensor_data Sensor data pointer
 */
void ChassisController_Update(ChassisController *controller, Gimbal_Sensor_Data_t* sensor_data);

/**
 * @brief Compute chassis motor currents
 * @param controller Chassis controller pointer
 * @param current_tick Current timestamp
 */
void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick);

/**
 * @brief Set chassis target speeds
 * @param controller Chassis controller pointer
 * @param speeds Target speeds array
 */
void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT]);

/**
 * @brief Stop chassis
 * @param controller Chassis controller pointer
 */
void ChassisController_Stop(ChassisController *controller);

/**
 * @brief Get output currents
 * @param controller Chassis controller pointer
 * @return Output currents array pointer
 */
const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller);

/**
 * @brief Check if any motor is running
 * @param controller Chassis controller pointer
 * @return true if any motor is running
 */
bool ChassisController_IsRunning(const ChassisController *controller);

/**
 * @brief Update motor feedback (to be called from CAN receive path)
 * @param controller Chassis controller pointer
 * @param motor_id Motor ID in range 0..3
 * @param angle Encoder angle
 * @param speed Speed (RPM)
 * @param current Current
 * @param temp Temperature
 * @param current_tick Current timestamp
 */
void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick);

/**
 * @brief Initialize chassis application (subscriptions, etc.)
 */
void ChassisApp_Init(void);

#endif // CHASSIS_CONTROLLER_H

