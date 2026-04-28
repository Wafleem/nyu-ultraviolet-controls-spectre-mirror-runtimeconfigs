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

/**
 * @brief Get PMM power limit in watts based on robot type
 *
 * Per RoboMaster rules: Hero/Sentry = 100W, Standard = 75W.
 * Default to 75W for unknown/pre-identified robots.
 */
float PMMLimit_Get(robot_id_t id)
{
    switch (id) {
        case RED_HERO:
        case BLUE_HERO:
        case RED_SENTRY:
        case BLUE_SENTRY:
            return 100.0f;
        case RED_STANDARD_1:
        case RED_STANDARD_2:
        case RED_STANDARD_3:
        case BLUE_STANDARD_1:
        case BLUE_STANDARD_2:
        case BLUE_STANDARD_3:
            return 75.0f;
        default:
            return 75.0f;
    }
}

/**
 * @brief Per-robot supercap charging power ceiling in watts.
 *
 * Mirrors the chassis PMM limit policy so the supercap cannot out-draw the
 * referee-system budget for this robot type.
 * TODO: CHECK HEALTH IF STANDARD TO SEE IF IN POWER OR ARMOR MODE. ASSUMING ARMOR MODE.
 */
float SupercapLimit_Get(robot_id_t id)
{
  switch (id) {
    case RED_HERO:
    case BLUE_HERO:
    case RED_SENTRY:
    case BLUE_SENTRY:
        return 100.0f;
    case RED_STANDARD_1:
    case RED_STANDARD_2:
    case RED_STANDARD_3:
    case BLUE_STANDARD_1:
    case BLUE_STANDARD_2:
    case BLUE_STANDARD_3:
        return 75.0f;
    default:
        return 75.0f;
  }
}
