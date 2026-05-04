#ifndef UNKNOWN_H
#define UNKNOWN_H

#include "config_types.h"
#include <stddef.h>

/**
 * @brief Unknown Robot Configuration
 */

// Robot configuration structure
static const RobotConfig_t g_robot_config_unknown = {
    .name = "Unknown",
    .motor_configs = NULL,
    .total_motor_count = 0,
    .chassis_yaw_source = YAW_SOURCE_DEVC,
    .aligned_yaw = 0.0f,
    .supercap_limit = 75.0f,
};

#endif // UNKNOWN_H
