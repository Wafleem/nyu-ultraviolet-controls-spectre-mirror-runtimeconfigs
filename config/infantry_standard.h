#ifndef INFANTRY_STANDARD_H
#define INFANTRY_STANDARD_H

#include "config_types.h"

/**
 * @brief Standard Infantry Robot Configuration
 *
 * Current configuration:
 * - 4x M3508 chassis motors (mecanum wheels) on CAN1
 * - 2x GM6020 gimbal motors (yaw + pitch) on CAN1
 * - 2x M3508 friction wheels + 1x M2006 feeder on CAN2
 */

// Motor configuration array
static const MotorConfig_t g_motor_configs_infantry_standard[] = {
    // ========== CHASSIS MOTORS (4x M3508) on CAN1 ==========
    // Front-left chassis motor (ID 0)
    {
        .motor_id = 0,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = -1,  // Mecanum kinematics correction
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },  // Speed PID
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }           // Not used
    },

    // Front-right chassis motor (ID 1)
    {
        .motor_id = 1,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x202,
        .can_tx_id = 0x200,
        .tx_slot = 1,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Back-left chassis motor (ID 2)
    {
        .motor_id = 2,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x203,
        .can_tx_id = 0x200,
        .tx_slot = 2,
        .direction = -1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Back-right chassis motor (ID 3)
    {
        .motor_id = 3,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x204,
        .can_tx_id = 0x200,
        .tx_slot = 3,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // ========== GIMBAL MOTORS (2x GM6020) on CAN1 ==========
    // Yaw gimbal motor (rotation) - 1 blink = motor ID 1
    {
        .motor_id = 5,  // Internal ID (avoids conflict with chassis)
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .can_channel = CAN_CHANNEL_1,  // CAN1 - same bus as chassis
        .can_rx_id = 0x205,  // GM6020: 0x204 + 1 = 0x205
        .can_tx_id = 0x1FF,  // Motors 1-4 use 0x1FF
        .tx_slot = 0,        // Motor 1 -> slot 0 (motor_id - 1)
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = -1.0f  // -1 = skip alignment
        },
        .pid_outer = { 0.40f, 0.0f, 0.03f, 300.0f, 300.0f },    // Yaw angle PID
        .pid_inner = { 20.0f, 0.0f, 3.0f, 30000.0f, 4000.0f }    // Yaw speed PID
    },

    // Pitch gimbal motor (up/down) - 2 blinks = motor ID 2
    {
        .motor_id = 6,  // Internal ID
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_PITCH,
        .can_channel = CAN_CHANNEL_1,  // CAN1 - same bus as chassis
        .can_rx_id = 0x206,  // GM6020: 0x204 + 2 = 0x206
        .can_tx_id = 0x1FF,  // Motors 1-4 use 0x1FF
        .tx_slot = 1,        // Motor 2 -> slot 1 (motor_id - 1)
        .direction = -1,     // Pitch direction correction
        .limits.gm6020 = {
            .angle_min = 1000.0f,
            .angle_max = 4000.0f,
            .gravity_compensation = 5000.0f,  // Gravity compensation for pitch
            .initial_angle = 2500.0f  // Center position (needs calibration)
        },
        .pid_outer = { 20.0f, 0.0f, 2.0f, 30000.0f, 25000.0f },  // Pitch PID
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }            // Not used for pitch
    },

    // ========== SHOOTER MOTORS (2x M3508, 1x M2006) on CAN2 ==========
    // Feeder motor (ID 7)
    {
        .motor_id = 7,
        .type = MOTOR_TYPE_M2006,
        .role = MOTOR_ROLE_SHOOTER_FEED,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x207,
        .can_tx_id = 0x1FF,
        .tx_slot = 2,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 1.0f, 0.0f, 0.0f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Friction wheel 1 (ID 5)
    {
        .motor_id = 5,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x205,
        .can_tx_id = 0x1FF,
        .tx_slot = 0,
        .direction = -1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 5.0f, 0.5f, 0.05f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // Friction wheel 2 (ID 6)
    {
        .motor_id = 6,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x206,
        .can_tx_id = 0x1FF,
        .tx_slot = 1,
        .direction = -1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = { 5.0f, 0.5f, 0.05f, 15000.0f, 7500.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    }
};

// Robot configuration structure
static const RobotConfig_t g_robot_config_infantry_standard = {
    .name = "Infantry Standard",
    .chassis_motor_count = 4,
    .gimbal_motor_count = 2,
    .shooter_motor_count = 3,
    .motor_configs = g_motor_configs_infantry_standard,
    .total_motor_count = 9,    // 4 chassis + 2 gimbal + 3 shooter
    .enable_imu_calibration = 0
};

#endif // INFANTRY_STANDARD_H
