#ifndef STANDARD_2026_H
#define STANDARD_2026_H

#include "config_types.h"

/**
 * @brief 2026 Standard Robot Configuration
 *
 * Current configuration:
 * - 4x M3508 chassis motors (mecanum wheels) on CAN1
 * - 1x M3508 gimbal yaw (belt drive) on CAN1
 * - 1x GM6020 gimbal pitch on CAN1
 * - 1x M2006 P36 indexer + 2x M3508 shooter friction wheels on CAN2
 */

// Motor configuration array
static const MotorConfig_t g_motor_configs_standard_2026[] = {
    // =CHASSIS MOTORS (4x M3508)=
    // Front-left chassis motor
    {
        .motor_id = 0,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .direction = -1,
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f, 10000.0f }   // Speed PID
    },

    // Front-right chassis motor
    {
        .motor_id = 1,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x202,
        .can_tx_id = 0x200,
        .direction = +1,
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f, 10000.0f }
    },

    // Back-left chassis motor
    {
        .motor_id = 2,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x203,
        .can_tx_id = 0x200,
        .direction = -1,
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f, 10000.0f }
    },

    // Back-right chassis motor
    {
        .motor_id = 3,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x204,
        .can_tx_id = 0x200,
        .direction = +1,
        .pid_outer = { 10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f, 10000.0f }
    },

    // =GIMBAL MOTORS=
    // Yaw motor - M3508 with belt drive, ESC ID 5, 5 blinks
    {
        .motor_id = 4,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x205,
        .can_tx_id = 0x1FF,
        .direction = +1,
        .pid_outer = { 50.0f, 0.0f, 0.0f, 7500.0f, 75.0f, 45.0f },     // Yaw angle PID (needs tuning)
        .pid_inner = { 10.0f, 0.0f, 0.0f, 16000.0f, 4000.0f, 10000.0f }   // Yaw speed PID (needs tuning)
//.pid_inner = { 7.0f, 0.0f, 0.0f, 16000.0f, 500.0f }    // Disabling yaw PID due to loose belt
    },

    // Pitch gimbal motor (up/down) - GM6020 ID 2, 2 blinks
    {
        .motor_id = 5,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_PITCH,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x206,  // GM6020 ID 2: 0x204 + 2
        .can_tx_id = 0x1FF,  // GM6020 motors 1-4
        .direction = +1,
        .limits.gm6020 = {
            .angle_min = 5270.0f,
            .angle_max = 6000.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = 5600.0f
        },
        .pid_outer = { 50.0f, 0.0f, 10.0f, 25000.0f, 11000.0f, 200.0f }
    },

    // ==SHOOTER MOTORS (1x M2006 indexer + 2x M3508 friction)==
    // Indexer motor - M2006 P36, feeds directly into shooter
    {
        .motor_id = 6,
        .type = MOTOR_TYPE_M2006,
        .role = MOTOR_ROLE_SHOOTER_FEED,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x201,  // ESC ID 1 on CAN2
        .can_tx_id = 0x200,
        .direction = +1,
        .pid_outer = { 2.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 10000.0f }
    },

    // Friction wheel 1 - M3508
    {
        .motor_id = 7,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x202,
        .can_tx_id = 0x200,
        .direction = -1,
        .pid_outer = {1.0f, 0.5f, 0.05f, 15000.0f, 7000.0f, 10000.0f}
    },

    // Friction wheel 2 - M3508
    {
        .motor_id = 8,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_SHOOTER_FRICTION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x203,
        .can_tx_id = 0x200,
        .direction = +1,
        .pid_outer = {1.0f, 0.5f, 0.05f, 15000.0f, 7500.0f, 10000.0f}
    }
};

// Robot configuration structure
static const RobotConfig_t g_robot_config_standard_2026 = {
    .name = "2026_Standard",
    .motor_configs = g_motor_configs_standard_2026,
    .total_motor_count = 9,    // 4 chassis + 2 gimbal + 3 shooter
    .reverse_chassis = 0,
    .enable_startup_alignment = 0,
    .chassis_yaw_source = YAW_SOURCE_DEVC,
    .power_limit_source = POWER_LIMIT_SOURCE_SUPERCAP,
    .aligned_yaw = 0.0f,
    .supercap_limit = 75.0f,
    .feeder_speed = 3000.0f,
    .friction_wheel_speed = 7000.0f,
    .pusher_extended_angle = 0.0f,
    .yaw_left_scale  = 1.2f,    // belt resists left  — boost it (tune up if still slow)
    .yaw_right_scale = 0.8f,    // belt assists right — reduce it (tune down if still fast)
};

#endif // STANDARD_2026_H
