#ifndef CONFIG_TYPES_H
#define CONFIG_TYPES_H

#include <stdint.h>

/**
 * @brief CAN channel enumeration
 */
typedef enum {
    CAN_CHANNEL_1 = 0,
    CAN_CHANNEL_2 = 1,
    CAN_CHANNEL_COUNT
} CAN_Channel_t;

/**
 * @brief Motor type enumeration
 */
typedef enum {
    MOTOR_TYPE_M3508,    // M3508 brushless motor with C620 ESC
    MOTOR_TYPE_GM6020,   // GM6020 brushless gimbal motor
    MOTOR_TYPE_M2006     // M2006 motor (for future use)
} MotorType_e;

/**
 * @brief Motor role enumeration
 * Defines the functional role of each motor in the robot system
 */
typedef enum {
    MOTOR_ROLE_CHASSIS_DRIVE,      // Chassis drive motor (mecanum or swerve)

    MOTOR_ROLE_CHASSIS_STEER,      // Swerve drive steering motor
    MOTOR_ROLE_GIMBAL_YAW,         // Gimbal yaw axis
    MOTOR_ROLE_GIMBAL_PITCH,       // Gimbal pitch axis
    MOTOR_ROLE_SHOOTER_FEED,       // Shooter feed/turntable motor
    MOTOR_ROLE_SHOOTER_FRICTION,   // Shooter friction wheel motor
    MOTOR_ROLE_SHOOTER_PUSH        // Hero Shooter ball pusher
} MotorRole_e;

/**
 * @brief Yaw source enumeration
 * Defines the hardware used to measure the yaw
 */
typedef enum {
    YAW_SOURCE_DEVC,               // Dev C IMU
    YAW_SOURCE_GM6020              // GM6020 motor encoder
} YawSource_e;

/**
 * @brief PID parameters structure
 */
typedef struct {
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain
    float output_max;      // Maximum output value
    float integral_max;    // Maximum integral value (anti-windup)
    float error_max;       // Maximum error value
} PIDParams_t;

/**
 * @brief Motor configuration structure
 * Contains all parameters needed to configure a single motor
 */
typedef struct {
    // Basic identification
    uint8_t motor_id;              // Logical motor ID (0-15)
    MotorType_e type;              // Motor type
    MotorRole_e role;              // Functional role

    // CAN communication parameters
    // Note: each motor category (chassis, gimbal, shooter) must be in separate
    // CAN frames or else there will be synchronization issues  -Ramon 4/25
    CAN_Channel_t can_channel;     // CAN bus channel (CAN_CHANNEL_1 or CAN_CHANNEL_2)
    uint16_t can_rx_id;            // CAN ID for receiving feedback
    uint16_t can_tx_id;            // CAN ID for sending commands (0x200, 0x1FF, or 0x2FF)

    // Motor-specific parameters
    int8_t direction;              // Motor direction: +1 or -1

    // Type-specific limits (only GM6020 uses this; zero-initialized otherwise)
    union {
        // GM6020-specific parameters
        struct {
            float angle_min;            // Minimum angle limit (encoder units)
            float angle_max;            // Maximum angle limit (encoder units)
            float gravity_compensation; // Gravity compensation torque (for pitch axis)
            float initial_angle;        // Initial calibration angle (encoder units)
        } gm6020;
    } limits;

    // PID control parameters
    PIDParams_t pid_outer;         // Outer loop PID (angle/position control)
    PIDParams_t pid_inner;         // Inner loop PID (speed control)

} MotorConfig_t;

/**
 * @brief Robot configuration structure
 * Top-level configuration for a complete robot
 */
typedef struct {
    const char *name;                    // Robot configuration name
    const MotorConfig_t *motor_configs;  // Pointer to motor configuration array
    uint8_t total_motor_count;           // Total number of motors
    uint8_t reverse_chassis;             // Whether to reverse the controller input for driving
    uint8_t enable_startup_alignment;    // Spin gimbal at startup to align head and chassis
    YawSource_e chassis_yaw_source;      // Hardware used to measure chassis yaw
    float aligned_yaw;                   // Depends on yaw source:
                                         // - Dev C: Spectre IMU yaw when ToF is aligned
                                         // - GM6020: GM6020 yaw when head and chassis are aligned
    float supercap_limit;                // Supercap power limit in watts
    float feeder_speed;                  // Feeder/turntable motor speed
    float friction_wheel_speed;          // Friction wheel motor speed
    int32_t pusher_retracted_angle;      // Pusher retracted angle (encoder units)
    int32_t pusher_extended_angle;       // Pusher extended angle (encoder units)
    float yaw_left_scale;               // Yaw rate multiplier going left  (>1 boosts, 0 = use 1.0)
    float yaw_right_scale;              // Yaw rate multiplier going right (<1 reduces, 0 = use 1.0)
} RobotConfig_t;

#endif // CONFIG_TYPES_H
