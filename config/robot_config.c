#include "robot_config.h"
#include "unknown.h"
#include "hero.h"
#include "standard_2025.h"
#include "standard_2026.h"

const RobotConfig_t* RobotConfig_Get(robot_id_t robot_id)
{
    switch (robot_id) {
        case RED_HERO:
        case BLUE_HERO:
            return &g_robot_config_hero;
    
        case RED_STANDARD_1:
        case RED_STANDARD_2:
        case BLUE_STANDARD_1:
        case BLUE_STANDARD_2:
            return &g_robot_config_standard_2026;

        case RED_STANDARD_3:
        case BLUE_STANDARD_3:
            return &g_robot_config_standard_2025;
    
        default:
            return &g_robot_config_unknown;
    }
}
