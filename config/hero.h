#ifndef HERO_H
#define HERO_H

#include "config_types.h"

/**
 * @brief Hero Robot Configuration
 *
 * Current configuration:
 * - 4x M3508 chassis motors (mecanum wheels) on CAN1
 * - 2x GM6020 gimbal motors (yaw + pitch) on CAN1
 * - 2x M3508 + 1x M2006 shooter motors on CAN2
 */

// Motor configuration array
static const MotorConfig_t g_motor_configs_hero[] = {
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
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f, 10000.0f },  // Speed PID
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }           // Not used
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
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f, 10000.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
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
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f, 10000.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
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
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f, 10000.0f },
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
    },

    // ========== GIMBAL MOTORS (2x GM6020) on CAN1 ==========
    // Yaw gimbal motor (rotation) - 1 blink
    {
        .motor_id = 4,  // Internal ID
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .can_channel = CAN_CHANNEL_1,  // CAN1 - same bus as chassis
        .can_rx_id = 0x209,  // GM6020: 0x204 + 1 = 0x205
        .can_tx_id = 0x2FF,  // Motors 1-4 use 0x1FF
        .tx_slot = 0,        // Motor 1 -> slot 0 (motor_id - 1)
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = -1.0f //4096.0f  // Center position (needs calibration) Currently set to -1 to skip
        },
        .pid_outer = { 13.5f, 0.0f, 0.3f, 300.0f, 300.0f, 180.0f },    // Yaw angle PID
        .pid_inner = { 50.0f, 0.0f, 3.0f, 25000.0f, 4000.0f, 10000.0f }    // Yaw speed PID
    },

    // Pitch gimbal motor (up/down) - 2 blinks
    {
        .motor_id = 5,  // Internal ID
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_PITCH,
        .can_channel = CAN_CHANNEL_2,  // CAN1 - same bus as chassis
        .can_rx_id = 0x206,  // GM6020: 0x204 + 2 = 0x206
        .can_tx_id = 0x1FF,  // Motors 1-4 use 0x1FF
        .tx_slot = 1,        // Motor 2 -> slot 1 (motor_id - 1)
        .direction = -1,     // Pitch direction correction
        .limits.gm6020 = {
            .angle_min = 4200.0f,
            .angle_max = 5300.0f,
            .gravity_compensation = 8000.0f,  // Gravity compensation for pitch
            .initial_angle = 4700.0f  // Center position (needs calibration)
        },
        .pid_outer = { 40.0f, 1.0f, 40.0f, 25000.0f, 25000.0f, 150.0f },  // Pitch PID
        .pid_inner = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }            // Not used for pitch
    },

    // ========== SHOOTER MOTORS (2x M3508, 1x M2006) on CAN2 ==========
    // Feeder motor
    {
        .motor_id = 6,
        .type = MOTOR_TYPE_M2006,
        .role = MOTOR_ROLE_SHOOTER_FEED,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = {5.0f, 1.0f, 0.0f, 15000.0f, 7500.0f, 10000.0f },
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // Friction wheel 1
    {
        .motor_id = 7,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x202,
        .can_tx_id = 0x200,
        .tx_slot = 1,
        .direction = -1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = {1.0f, 0.5f, 0.05f, 15000.0f, 7500.0f, 10000.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // Friction wheel 2
    {
        .motor_id = 8,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x203,
        .can_tx_id = 0x200,
        .tx_slot = 2,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = {1.0f, 0.5f, 0.05f, 15000.0f, 7500.0f, 10000.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    },

    // Ball Pusher
    {
        .motor_id = 9,
        .type = MOTOR_TYPE_M2006,
        .role = MOTOR_ROLE_SHOOTER_PUSH,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x204,
        .can_tx_id = 0x200,
        .tx_slot = 3,
        .direction = +1,
        .limits.m3508 = {
            .speed_limit = 10000.0f
        },
        .pid_outer = {0.1f, 0.0f, 0.0f, 15000.0f, 7500.0f, 100000.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    }
};

// Robot configuration structure
static const RobotConfig_t g_robot_config_hero = {
    .name = "Hero",
    .chassis_motor_count = 4,
    .gimbal_motor_count = 2,
    .shooter_motor_count = 4,
    .motor_configs = g_motor_configs_hero,
    .total_motor_count = 10,    // 4 chassis + 2 gimbal + 4 shooter
    .enable_imu_calibration = 0,
    .chassis_yaw_source = YAW_SOURCE_GM6020,
    .aligned_yaw = 55.0f,
    .feeder_speed = 4000.0f,
    .friction_wheel_speed = 10000.0f,
    .pusher_retracted_angle = 0.0f,
    .pusher_extended_angle = (int32_t)(-10.0f * 8192.0f)
};

#endif // HERO_H
