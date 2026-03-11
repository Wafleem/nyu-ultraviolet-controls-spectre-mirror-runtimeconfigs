#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "config_types.h"
#include "ref_structs.h"

/**
 * @brief Get the active robot configuration
 * @return Pointer to the active robot configuration structure
 */
const RobotConfig_t* RobotConfig_Get(robot_id_t robot_id);

#endif // ROBOT_CONFIG_H
