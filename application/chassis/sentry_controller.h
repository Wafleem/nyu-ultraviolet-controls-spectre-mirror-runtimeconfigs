// sentry_controller.h
#ifndef SENTRY_CONTROLLER_H
#define SENTRY_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "main.h"
#include "remote_control.h"
#include "motor_feedback.h"
#include "gyro_data.h"
#include "config_types.h"

#define SENTRY_DRIVE_MOTOR_COUNT 4
#define SENTRY_STEER_MOTOR_COUNT 2

// Same scale idea as chassis demo
#define SENTRY_DRIVE_TARGET_SPEED 7000

typedef struct {
    // Targets
    float drive_target_speeds[SENTRY_DRIVE_MOTOR_COUNT];  // RPM targets
    float steer_target_angles[SENTRY_STEER_MOTOR_COUNT];  // encoder ticks 0..8191

    // Running state
    bool running;

    // PIDs
    PID_Controller drive_speed_pids[SENTRY_DRIVE_MOTOR_COUNT];
    PID_Controller steer_angle_pids[SENTRY_STEER_MOTOR_COUNT];

    // Feedbacks (indexed by local index 0..N-1)
    Motor_Feedback drive_feedbacks[SENTRY_DRIVE_MOTOR_COUNT];
    Motor_Feedback steer_feedbacks[SENTRY_STEER_MOTOR_COUNT];

    // Outputs
    int16_t drive_output_currents[SENTRY_DRIVE_MOTOR_COUNT];
    int16_t steer_output_currents[SENTRY_STEER_MOTOR_COUNT];
} SentryController;

void SentryController_Init(SentryController *controller);
void SentryController_Update(SentryController *controller, SensorData *sensor_data);
void SentryController_ComputeCurrents(SentryController *controller, uint32_t current_tick);

void SentryController_SetDriveTargetSpeeds(SentryController *controller, const float speeds[SENTRY_DRIVE_MOTOR_COUNT]);
void SentryController_SetSteerTargetAngles(SentryController *controller, const float angles[SENTRY_STEER_MOTOR_COUNT]);

void SentryController_Stop(SentryController *controller);
bool SentryController_IsRunning(const SentryController *controller);

void SentryController_UpdateDriveFeedback(SentryController *controller, uint8_t index,
                                         uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick);

void SentryController_UpdateSteerFeedback(SentryController *controller, uint8_t index,
                                         uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick);

static int16_t SteerController_CascadeControl(
    uint8_t motor_id,
    float rate_normalized
);

// App wrapper
void SentryApp_Init(void);
SentryController* SentryApp_GetController(void);

#endif // SENTRY_CONTROLLER_H
