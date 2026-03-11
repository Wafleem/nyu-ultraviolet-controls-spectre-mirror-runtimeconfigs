#include "robot_config.h"
#include "unknown.h"
#include "hero.h"
#include "infantry_standard.h"

const RobotConfig_t* RobotConfig_Get(robot_id_t robot_id)
{
    switch (robot_id) {
        case RED_HERO:
        case BLUE_HERO:
            return &g_robot_config_hero;

        case RED_STANDARD_1:
        case RED_STANDARD_2:
        case RED_STANDARD_3:
        case BLUE_STANDARD_1:
        case BLUE_STANDARD_2:
        case BLUE_STANDARD_3:
            return &g_robot_config_infantry_standard;

        default:
            return &g_robot_config_unknown;
    }
}