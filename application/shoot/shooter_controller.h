#ifndef SHOOTER_CONTROLLER_H
#define SHOOTER_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "remote_control.h"
#include "motor_feedback.h"
#include "imu.h"
#include "config_types.h"

// Shooter system motor count
#define SHOOTER_MOTOR_COUNT 4

// Shooter system parameters
#define ANTIJAM_ENABLED 0
#define TURNTABLE_CONST_SPEED 5000      // Turntable speed
#define SHOOTER_CONST_SPEED 7500     // Shooter wheel speed
#define SHOOTER_RAMP_STEP 50.0f      // Acceleration step
#define PUSHER_EXTENDED (int32_t)(2.5f * 8192.0f)
#define PUSHER_RETRACTED 0

// Shooter controller structure
typedef struct {
    // Turntable target speed
    float turntable_target;
    float ramped_turntable;
    
    // Shooter wheel target speeds
    float shooter1_target;
    float shooter2_target;
    float ramped_shooter1;
    float ramped_shooter2;

    // Pusher targets
    float pusher_target;
    int32_t pusher_total_angle;
    uint16_t pusher_last_angle;
    bool pusher_initialized;
    
    // Running state
    bool enabled;
    
    // PID controllers (turntable and shooter wheels)
    PID_Controller turntable_pid;
    PID_Controller shooter1_pid;
    PID_Controller shooter2_pid;
    PID_Controller pusher_pid;
    
    // Motor feedbacks (turntable and shooter wheels)
    Motor_Feedback turntable_feedback;
    Motor_Feedback shooter1_feedback;
    Motor_Feedback shooter2_feedback;
    Motor_Feedback pusher_feedback;
    
    // Output currents
    int16_t output_currents[SHOOTER_MOTOR_COUNT];
    int8_t directions[SHOOTER_MOTOR_COUNT];
} ShooterController;

/**
 * @brief Initialize shooter controller
 * @param controller Shooter controller pointer
 */
void ShooterController_Init(ShooterController *controller);

/**
 * @brief Update shooter control logic
 * @param controller Shooter controller pointer
 * @param sensor_data Sensor data pointer
 */
void ShooterController_Update(ShooterController *controller, Gimbal_Sensor_Data_t *sensor_data);

/**
 * @brief Compute shooter system motor currents
 * @param controller Shooter controller pointer
 * @param current_tick Current timestamp
 */
void ShooterController_ComputeCurrents(ShooterController *controller, uint32_t current_tick);

/**
 * @brief Stop shooter system
 * @param controller Shooter controller pointer
 */
void ShooterController_Stop(ShooterController *controller);

/**
 * @brief Update motor feedback
 * @param controller Shooter controller pointer
 * @param motor_id Motor ID (4=turntable, 5=shooter1, 7=shooter2)
 * @param angle Angle
 * @param speed Speed
 * @param current Current
 * @param temp Temperature
 * @param current_tick Current timestamp
 */
void ShooterController_UpdateMotorFeedback(ShooterController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick);

/**
 * @brief Attempt to unjam the feeder
 * @param controller Shooter controller pointer
 */
void ShooterController_Unjam(ShooterController *controller);

#endif // SHOOTER_CONTROLLER_H

