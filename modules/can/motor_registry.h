#ifndef MOTOR_REGISTRY_H
#define MOTOR_REGISTRY_H

#include <stdint.h>
#include <stdbool.h>
#include "config_types.h"
#include "motor_feedback.h"

/**
 * @brief Motor Registry Module
 *
 * This module provides a runtime registry for motor configurations,
 * enabling dynamic lookup of motors by CAN RX ID or motor ID.
 * It bridges the gap between static compile-time configurations
 * and runtime CAN message processing.
 */

// Maximum number of motors supported
#define MOTOR_REGISTRY_MAX_MOTORS 16

/**
 * @brief Motor registry entry
 * Combines motor configuration with runtime feedback data
 */
typedef struct {
    const MotorConfig_t *config;     // Pointer to motor configuration
    bool active;                     // true if this slot is occupied

    // Feedback data storage (union to save memory)
    union {
        Motor_Feedback m3508_feedback;   // M3508/M2006 feedback
        struct {
            uint16_t angle_raw;          // Raw encoder value
            int16_t speed_rpm;           // Speed in RPM
            int16_t current;             // Motor current
            uint32_t last_update_time;   // Last update tick
        } gm6020_feedback;
    } feedback;
} MotorRegistryEntry_t;

/**
 * @brief Motor registry structure
 */
typedef struct {
    MotorRegistryEntry_t entries[MOTOR_REGISTRY_MAX_MOTORS];
    uint8_t motor_count;             // Number of registered motors
    CAN_Channel_t can_channel;       // CAN channel for this registry
} MotorRegistry_t;

void MotorRegistry_Init(MotorRegistry_t *registry,
                       const RobotConfig_t *robot_config,
                       CAN_Channel_t can_channel);

const MotorConfig_t* MotorRegistry_FindByRxId(const MotorRegistry_t *registry,
                                             uint16_t can_rx_id);

const MotorConfig_t* MotorRegistry_FindByMotorId(const MotorRegistry_t *registry,
                                                uint8_t motor_id);

void MotorRegistry_UpdateFeedback(MotorRegistry_t *registry,
                                 uint8_t motor_id,
                                 const void *feedback);

const void* MotorRegistry_GetFeedback(const MotorRegistry_t *registry,
                                     uint8_t motor_id);

uint8_t MotorRegistry_GetMotorCount(const MotorRegistry_t *registry);

#endif // MOTOR_REGISTRY_H
