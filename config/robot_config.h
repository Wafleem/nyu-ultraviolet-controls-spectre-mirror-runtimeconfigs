#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "config_types.h"
#include "infantry_standard.h"

/**
 * @brief Get the active robot configuration
 * @return Pointer to the active robot configuration structure
 *
 * Currently hardcoded to infantry_standard.
 * To add ROBOT_TYPE CMake selection later, use the pattern from NYUSH:
 *   cmake -S . -B build -DROBOT_TYPE=infantry_standard
 */
const RobotConfig_t* RobotConfig_Get(void);

#endif // ROBOT_CONFIG_H
