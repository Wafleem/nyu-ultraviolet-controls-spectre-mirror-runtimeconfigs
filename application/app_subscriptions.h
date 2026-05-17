#ifndef APP_SUBSCRIPTIONS_H
#define APP_SUBSCRIPTIONS_H

#include <stdint.h>
#include "chassis_controller.h"
#include "shooter_controller.h"

// Initialize application controllers (event-driven)
void ChassisApp_Init(const RobotConfig_t *robot_config);
void ShooterApp_Init(const RobotConfig_t *config);

#endif // APP_SUBSCRIPTIONS_H

