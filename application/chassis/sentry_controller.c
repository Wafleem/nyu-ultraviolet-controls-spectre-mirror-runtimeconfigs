// sentry_controller.c (renamed API to match chassis_controller)
// Implements the SAME exported symbols as chassis_controller.c, but for Sentry Swerve:
// - 4x M3508 drive motors (role: MOTOR_ROLE_CHASSIS_DRIVE)
// - 2x GM6020 steer motors (role: MOTOR_ROLE_CHASSIS_STEER)

#include "chassis_controller.h"   // IMPORTANT: use the common header name
#include "motor_driver.h"
#include <string.h>
#include <math.h>
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "can_comm.h"
#include "printing.h"
#include "cmd_controller.h"
#include <stdint.h>

#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

// Math constants
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static SensorData s_last_sensor;
static ChassisController s_ctrl;

// Motor configuration (dynamically assigned during init)
static uint8_t s_drive_motor_ids[CHASSIS_MOTOR_COUNT];
static int8_t  s_drive_motor_directions[CHASSIS_MOTOR_COUNT];
static uint8_t s_drive_motor_count = 0;

#define CHASSIS_STEER_COUNT 2
static uint8_t s_steer_motor_ids[CHASSIS_STEER_COUNT];
static int8_t  s_steer_motor_directions[CHASSIS_STEER_COUNT];
static uint8_t s_steer_motor_count = 0;

static void ResetPidIntegrals(ChassisController *controller)
{
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        controller->speed_pids[i].integral = 0.0f;
    }
    for (int i = 0; i < CHASSIS_STEER_COUNT; i++) {
        controller->steer_pids[i].integral = 0.0f;
    }
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) { return 0; }
    float measured = feedback->speed; // drive: RPM
    return (int16_t)PID_Calculate(pid, target, measured);
}

static int16_t ComputeSingleSteerCurrent(PID_Controller *pid, float target_angle, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) { return 0; }

    // Use encoder angle (0..8191) for steering position PID
    float measured_angle = (float)feedback->angle;

    // Shortest-path wrap in ticks
    float err = target_angle - measured_angle;
    if (err >  4096.0f) target_angle -= 8192.0f;
    if (err < -4096.0f) target_angle += 8192.0f;

    return (int16_t)PID_Calculate(pid, target_angle, measured_angle);
}

void ChassisController_Init(ChassisController *controller)
{
    if (controller == NULL) return;

    memset(controller, 0, sizeof(ChassisController));

    // Find drive motors by role
    s_drive_motor_count = MotorDriver_FindByRole(
        MOTOR_ROLE_CHASSIS_DRIVE,
        s_drive_motor_ids,
        CHASSIS_MOTOR_COUNT
    );

    // Find steer motors by role (requires enum MOTOR_ROLE_CHASSIS_STEER)
    s_steer_motor_count = MotorDriver_FindByRole(
        MOTOR_ROLE_CHASSIS_STEER,
        s_steer_motor_ids,
        CHASSIS_STEER_COUNT
    );

    // DEBUG: Print found steer motors
    USB_CDC_Printf("[CHASSIS_INIT] Found %d steer motors: ", s_steer_motor_count);
    for (uint8_t i = 0; i < s_steer_motor_count; i++) {
        USB_CDC_Printf("%d ", s_steer_motor_ids[i]);
    }
    USB_CDC_Printf("\r\n");

    // Init drive PIDs from motor contexts
    for (uint8_t i = 0; i < s_drive_motor_count; i++) {
        MotorContext_t *ctx = MotorDriver_GetContext(s_drive_motor_ids[i]);
        if (ctx && ctx->config) {
            s_drive_motor_directions[i] = ctx->config->direction;

            PID_Init(&controller->speed_pids[i],
                     ctx->config->pid_outer.kp,
                     ctx->config->pid_outer.ki,
                     ctx->config->pid_outer.kd,
                     ctx->config->pid_outer.output_max,
                     ctx->config->pid_outer.integral_max);

            controller->target_speeds[i] = 0.0f;
        }
    }

    // Init steer PIDs from motor contexts (outer PID is angle/position PID per your config)
    for (uint8_t i = 0; i < s_steer_motor_count; i++) {
        MotorContext_t *ctx = MotorDriver_GetContext(s_steer_motor_ids[i]);
        if (ctx && ctx->config) {
            s_steer_motor_directions[i] = ctx->config->direction;

            PID_Init(&controller->steer_pids[i],
                     ctx->config->pid_outer.kp,
                     ctx->config->pid_outer.ki,
                     ctx->config->pid_outer.kd,
                     ctx->config->pid_outer.output_max,
                     ctx->config->pid_outer.integral_max);

            controller->steer_target_angles[i] = (float)ctx->config->limits.gm6020.initial_angle;
        }
    }
}

void ChassisController_Update(ChassisController *controller, SensorData *sensor_data)
{
    (void)sensor_data;
    if (controller == NULL) return;

    float vx_norm = s_last_cmd.vx;
    float vy_norm = s_last_cmd.vy;
    float wz_norm = s_last_cmd.wz;

    float scale = (float)CHASSIS_DEMO_TARGET_SPEED / 2.0f;

    // cmd_controller already did coordinate transform (per your comment in chassis_controller.c)
    float vx = vx_norm * scale;
    float vy = vy_norm * scale;

    // ===== Simple Drive Logic =====
    // Drive motors: controlled by vy (left/right stick)
    // Steer motors: controlled by vx (forward/backward stick)

    // Drive targets: all wheels use vy speed only (left/right stick)
    if (s_drive_motor_count >= 4) {
        for (uint8_t i = 0; i < s_drive_motor_count; i++) {
            controller->target_speeds[i] = s_drive_motor_directions[i] * vy;
        }
    }

    // Steer targets: not used (steer uses manual speed control via vx)
    // No need to set steer_target_angles here




    controller->running = s_last_cmd.enabled;
}

static int16_t SteerController_CascadeControl(
    uint8_t motor_id,
    float rate_normalized
)
{
    MotorContext_t *steer = MotorDriver_GetContext(motor_id);
    if (!steer || !steer->angle_initialized || !steer->config) {
        // DEBUG: Print why steer control is returning 0
        static uint32_t last_debug = 0;
        if (HAL_GetTick() - last_debug > 500) {
            USB_CDC_Printf("[STEER_CASCADE] Motor %d: steer=%p init=%d cfg=%p\r\n",
                          motor_id,
                          steer,
                          (steer ? steer->angle_initialized : -1),
                          (steer ? steer->config : NULL));
            last_debug = HAL_GetTick();
        }
        return 0;
    }

    // ==========================
    // Joystick → angle target
    // ==========================
    const float sensitivity = 50.0f;   // ticks per update (tune)
    steer->angle_target += (float)steer->config->direction
                           * sensitivity
                           * rate_normalized;

    // Encoder range
    float enc_max = (steer->config->limits.gm6020.angle_max > 0.0f)
                    ? steer->config->limits.gm6020.angle_max
                    : 8192.0f;

    // Wrap target
    if (steer->angle_target >= enc_max)
        steer->angle_target -= enc_max;
    else if (steer->angle_target < 0.0f)
        steer->angle_target += enc_max;

    // ==========================
    // Angle error (shortest path)
    // ==========================
    float current_angle = (float)steer->angle_raw;
    float angle_error = steer->angle_target - current_angle;

    if (fabsf(angle_error) < 1.0f)
        angle_error = 0.0f;

    if (angle_error >  enc_max / 2.0f) angle_error -= enc_max;
    if (angle_error < -enc_max / 2.0f) angle_error += enc_max;

    // ==========================
    // OUTER: angle → speed
    // ==========================
    float cmd_angle_to_speed =
        PID_Calculate(&steer->pid_outer, 0.0f, -angle_error);

    // RPM limit (stability)
    const float rpm_limit = 300.0f;
    if (cmd_angle_to_speed >  rpm_limit) cmd_angle_to_speed =  rpm_limit;
    if (cmd_angle_to_speed < -rpm_limit) cmd_angle_to_speed = -rpm_limit;

    // ==========================
    // INNER: speed → current
    // ==========================
    float speed_feedback = (float)steer->speed_rpm;

    float cmd_speed_to_current =
        PID_Calculate(&steer->pid_inner,
                      cmd_angle_to_speed,
                      speed_feedback);

    // Current clamp
    const float current_limit = 25000.0f;
    if (cmd_speed_to_current >  current_limit) cmd_speed_to_current =  current_limit;
    if (cmd_speed_to_current < -current_limit) cmd_speed_to_current = -current_limit;

    return (int16_t)cmd_speed_to_current;
}


void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;

    static uint32_t last = 0;
    if (HAL_GetTick() - last > 200) {
        last = HAL_GetTick();

        USB_CDC_Printf("[CHASSIS_CUR:SENTRY] en=%d\r\n", (int)s_last_cmd.enabled);

        // Drive debug
        for (uint8_t i = 0; i < s_drive_motor_count; i++) {
            uint32_t dt = HAL_GetTick() - controller->motor_feedbacks[i].last_update_time;
            USB_CDC_Printf(
                "  [DRV] i=%d id=%d tgt=%.1f fb=%.1f dt=%lu out=%d\r\n",
                i,
                s_drive_motor_ids[i],
                controller->target_speeds[i],
                controller->motor_feedbacks[i].speed,
                (unsigned long)dt,
                controller->output_currents[i]
            );
        }

        // Steer debug
        for (uint8_t i = 0; i < s_steer_motor_count; i++) {
            uint32_t dt = HAL_GetTick() - controller->steer_feedbacks[i].last_update_time;
            USB_CDC_Printf(
                "  [STR] i=%d id=%d tgt=%.1f ang=%u spd=%.1f dt=%lu out=%d\r\n",
                i,
                s_steer_motor_ids[i],
                controller->steer_target_angles[i],
                controller->steer_feedbacks[i].angle,
                controller->steer_feedbacks[i].speed,
                (unsigned long)dt,
                controller->steer_output_currents[i]
            );
        }
    }

    // Drive currents
    for (int i = 0; i < s_drive_motor_count; i++) {
        int16_t motor_current = ComputeSingleMotorCurrent(
            &controller->speed_pids[i],
            controller->target_speeds[i],
            &controller->motor_feedbacks[i],
            current_tick
        );
        controller->output_currents[i] = motor_current;
        MotorDriver_SendCurrent(s_drive_motor_ids[i], motor_current);
    }

    // Steer currents
    // ===== Steer currents (manual speed control) =====
    for (int i = 0; i < s_steer_motor_count; i++) {
        int16_t motor_current =
            SteerController_CascadeControl(
                s_steer_motor_ids[i],
                s_last_cmd.vx        // vx (forward/backward stick) controls steering speed
            );

        controller->steer_output_currents[i] = motor_current;
        MotorDriver_SendCurrent(s_steer_motor_ids[i], motor_current);
    }


    MotorDriver_FlushAll();
}

void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT])
{
    if (controller == NULL || speeds == NULL) return;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        controller->target_speeds[i] = speeds[i];
    }
}

void ChassisController_SetSteerTargetAngles(ChassisController *controller, const float angles[CHASSIS_STEER_COUNT])
{
    if (controller == NULL || angles == NULL) return;
    for (int i = 0; i < CHASSIS_STEER_COUNT; i++) {
        controller->steer_target_angles[i] = angles[i];
    }
}

void ChassisController_Stop(ChassisController *controller)
{
    if (controller == NULL) return;
    controller->running = false;
    ResetPidIntegrals(controller);

    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        controller->target_speeds[i] = 0.0f;
    }
    // Steer holds by default
}

const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller)
{
    if (controller == NULL) return NULL;
    return controller->output_currents;
}

bool ChassisController_IsRunning(const ChassisController *controller)
{
    if (controller == NULL) return false;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (controller->target_speeds[i] != 0.0f) return true;
    }
    return false;
}

// Drive feedback updater: motor_id is local index 0..3 (same as your existing chassis_controller)
void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id,
                                          uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
    controller->motor_feedbacks[motor_id].angle = angle;
    controller->motor_feedbacks[motor_id].speed = speed;
    controller->motor_feedbacks[motor_id].current = current;
    controller->motor_feedbacks[motor_id].temp = temp;
    controller->motor_feedbacks[motor_id].last_update_time = current_tick;
}

// Steer feedback updater: index 0..1
void ChassisController_UpdateSteerFeedback(ChassisController *controller, uint8_t index,
                                          uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL || index >= CHASSIS_STEER_COUNT) return;
    controller->steer_feedbacks[index].angle = angle;
    controller->steer_feedbacks[index].speed = speed;
    controller->steer_feedbacks[index].current = current;
    controller->steer_feedbacks[index].temp = temp;
    controller->steer_feedbacks[index].last_update_time = current_tick;
}

// =====================
// Subscription callbacks
// =====================
static void on_chassis_cmd(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(ChassisCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
        ChassisController_Update(&s_ctrl, &s_last_sensor);
        ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
    }
}

static void on_imu_update(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

static void on_motor_feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;

        // Drive motor match -> update drive feedback by LOCAL INDEX 0..3
        for (uint8_t i = 0; i < s_drive_motor_count; i++) {
            if (m->id == s_drive_motor_ids[i]) {
                ChassisController_UpdateMotorFeedback(&s_ctrl, i, m->angle, m->speed, m->current, m->temp, m->tick_ms);
                return;
            }
        }
    }
}

// NEW: GM6020 feedback callback for steer motors
static void on_gm6020_feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size == sizeof(GM6020FeedbackEvent)) {
        const GM6020FeedbackEvent *m = (const GM6020FeedbackEvent *)ev->data;

        // Steer motor match -> update steer feedback by LOCAL INDEX 0..1
        for (uint8_t i = 0; i < s_steer_motor_count; i++) {
            if (m->id == s_steer_motor_ids[i]) {
                ChassisController_UpdateSteerFeedback(&s_ctrl, i, m->angle, m->speed, m->current, 0, m->tick_ms);
                return;
            }
        }
    }
}

void ChassisApp_Init(void)
{
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));

    ChassisController_Init(&s_ctrl);

    (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
    (void)MsgCenter_Subscribe(TOPIC_GM6020_FEEDBACK, on_gm6020_feedback, NULL);  // Subscribe to GM6020 feedback for steer motors
}

ChassisController* ChassisApp_GetController(void)
{
    return &s_ctrl;
}

/**
 * @brief Wait for swerve steer motors to align to initial position
 * @note Should be called after ChassisApp_Init() and CAN bus stabilization
 */
void Sentry_WaitForSteerAlignment(void)
{
    const float ALIGNMENT_THRESHOLD = 100.0f;  // encoder ticks tolerance
    const uint32_t TIMEOUT_MS = 5000;  // 5 seconds timeout
    const uint32_t CHECK_INTERVAL_MS = 50;

    USB_CDC_Printf("[Sentry] Waiting for steer motors to align to initial position...\r\n");

    uint32_t start_time = HAL_GetTick();
    bool all_aligned = false;

    while (HAL_GetTick() - start_time < TIMEOUT_MS && !all_aligned) {
        // Send zero command to hold position (motors will go to initial_angle)
        // Just dispatch messages to process feedback
        MsgCenter_Dispatch();

        // Send currents to hold steer motors at target position
        ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());

        HAL_Delay(CHECK_INTERVAL_MS);

        // Check if all steer motors have reached target position
        all_aligned = true;
        for (uint8_t i = 0; i < s_steer_motor_count; i++) {
            MotorContext_t *steer = MotorDriver_GetContext(s_steer_motor_ids[i]);
            if (steer && steer->angle_initialized) {
                float target = steer->angle_target;
                float current = (float)steer->angle_raw;
                float error = fabsf(target - current);

                // Handle wraparound (0-8192 encoder range)
                if (error > 4096.0f) {
                    error = 8192.0f - error;
                }

                USB_CDC_Printf("[Sentry] Steer motor %d: target=%.1f current=%.1f error=%.1f\r\n",
                              s_steer_motor_ids[i], target, current, error);

                if (error > ALIGNMENT_THRESHOLD) {
                    all_aligned = false;
                }
            } else {
                all_aligned = false;
            }
        }
    }

    if (all_aligned) {
        USB_CDC_Printf("[Sentry] Steer alignment complete!\r\n");
    } else {
        USB_CDC_Printf("[Sentry] Warning: Steer alignment timeout, continuing anyway...\r\n");
    }
}
