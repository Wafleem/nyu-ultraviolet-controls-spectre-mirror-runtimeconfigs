#include "gimbal_controller.h"
#include "pid.h"
#include "message_center.h"
#include <math.h>
#include <string.h>
#include "printing.h"
#include "stm32h7xx_hal.h"
#include "logger.h"

// Motor IDs (dynamically assigned during init)
static uint8_t s_pitch_motor_id = 0xFF;
static uint8_t s_yaw_motor_id = 0xFF;

// Static state for application
static GimbalCmd s_last_cmd;
static Gimbal_Sensor_Data_t s_last_sensor;
static bool s_initialized = false;
static bool s_yaw_target_seeded = false;
static uint32_t last_tick = 0;
static float dt = 0.0f;

int16_t GimbalController_PitchControl(Gimbal_Sensor_Data_t* sensor_data)
{
    (void)sensor_data;  // Not used for pitch
    MotorContext_t *c = MotorDriver_GetContext(s_pitch_motor_id);
    if (!c || !c->angle_initialized) {
        return 0;
    }

    // Determine if this is pitch motor (has angle limits)
    bool is_pitch_motor = (c->role == MOTOR_ROLE_GIMBAL_PITCH);
    float max_encoder = (c->config->limits.gm6020.angle_max > 0.0f) ?
                        c->config->limits.gm6020.angle_max : MAX_PITCH_ANGLE;

    float current_angle = (float)c->angle_raw;
    float error = c->angle_target - current_angle;
    if (error > max_encoder / 2.0f)
        error -= max_encoder;
    else if (error < -max_encoder / 2.0f)
        error += max_encoder;

    float cmd = PID_Calculate(&c->pid_outer, error, 0.0f);

    if (is_pitch_motor) {
        float ang01 = current_angle - c->config->limits.gm6020.initial_angle;
        float ang_rad = ang01 * (2 * M_PI) / MAX_PITCH_ANGLE;
        float gravity_ff = c->config->direction * c->config->limits.gm6020.gravity_compensation * cosf(ang_rad);
        cmd += gravity_ff;
    }
    float max_abs = 25000.0f;
    if (cmd >  max_abs) cmd =  max_abs;
    if (cmd < -max_abs) cmd = -max_abs;

    // LOG_CSV(LOG_TAG_GIM, "PITCH_CSV,%.2f,%.2f,%d,%.2f,%.2f",
    //                c->angle_target,
    //                current_angle,
    //                c->speed_rpm,
    //                cmd,
    //                error);

    return (int16_t)cmd;
}

int16_t GimbalController_YawControlWithCompensation(Gimbal_Sensor_Data_t* sensor_data, bool use_imu_feedback)
{
    MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
    if (!yaw || !yaw->angle_initialized) return 0;

    float current = sensor_data->ekf_yaw;
    float angle_error = yaw->angle_target - current;

    // small deadband
    if (fabsf(angle_error) < 1.5f)
        angle_error = 0.0f;

   
    // wrap error into [-MAX/2, MAX/2]
    if (angle_error >  MAX_YAW_ANGLE / 2.0f) angle_error -= MAX_YAW_ANGLE;
    if (angle_error < -MAX_YAW_ANGLE / 2.0f) angle_error += MAX_YAW_ANGLE;

    // ==========================
    // OUTER LOOP: angle → speed
    // ==========================
    float cmd_angle_to_speed = PID_Calculate(&yaw->pid_outer, 0.0f, -angle_error);

    // Speed feedback source selection:
    // - Spin mode: Use IMU gyro for absolute yaw stability
    // - Normal mode: Use motor encoder for better response
    // float speed_feedback;
    // if (use_imu_feedback) {
    //     // IMU gyro feedback (for spin mode)
    //     // Convert IMU gyro (rad/s) to RPM: 1 rad/s = 30/π RPM ≈ 9.549 RPM
    //     speed_feedback = -s_last_sensor.gyro_z * 30.0f / (float)M_PI;  // negative because IMU z-axis convention
    // } else {
    //     // Motor encoder speed feedback (for normal mode)
    //     speed_feedback = (float)yaw->speed_rpm;
    // }
    float speed_feedback = (float)yaw->speed_rpm;

    float cmd_speed_to_current =
        PID_Calculate(&yaw->pid_inner, cmd_angle_to_speed, speed_feedback);

    // Logging for tuning/debug
    // Columns: angle_target, ekf_yaw, speed_setpoint, actual_rpm, gyro_z, current_cmd, angle_error, feedback_current
    LOG_CSV(LOG_TAG_GIM, "YAW_CSV,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%d",
          yaw->angle_target,
          s_last_sensor.ekf_yaw,
          cmd_angle_to_speed,
          speed_feedback,
          s_last_sensor.gyro_z,
          cmd_speed_to_current,
          angle_error,
          (int)yaw->feedback_current);

    return (int16_t)cmd_speed_to_current;
}


void GimbalController_UpdateTargets(GimbalCmd *cmd, MotorContext_t *yaw, MotorContext_t *pitch) {
    static bool s_yaw_vision_active = false;
    static bool s_pitch_vision_active = false;
    static bool s_yaw_spin_hold_active = false;
    bool use_vision_target = cmd->vision_valid && cmd->aimbot_mode;
    bool use_spin_hold = (!use_vision_target) && (cmd->yaw_rate_memo > 0.5f);

    // Don't do anything if yaw or pitch are null
    if (yaw == NULL || pitch == NULL) {
        return;
    }

    // Get gimbal config or defaults
    float pitch_min, pitch_max, pitch_direction, yaw_direction;
    if (pitch->config != NULL) {
        pitch_max = pitch->config->limits.gm6020.angle_max;
        pitch_min = pitch->config->limits.gm6020.angle_min;
        pitch_direction = pitch->config->direction;
    } else {
        pitch_max = MAX_PITCH_ANGLE;
        pitch_min = 0.0f;
        pitch_direction = 1.0f;
    }
    if (yaw->config != NULL) {
        yaw_direction = yaw->config->direction;
    } else {
        yaw_direction = 1.0f;
    }

    float current_yaw = s_last_sensor.ekf_yaw;
    // Continuous angle control: update target angle every cycle when vision is valid
    if (use_vision_target) {
        if (yaw && yaw->angle_initialized) {
            const float angles_per_rad = MAX_YAW_ANGLE / (2.0f * (float)M_PI);
            float err_ticks = cmd->vision_yaw_err_rad * angles_per_rad;
            // Update target angle continuously based on current angle + vision error
            yaw->angle_target = (float)current_yaw + err_ticks;

            if (!s_yaw_vision_active) {
                PID_Reset(&yaw->pid_outer);
                PID_Reset(&yaw->pid_inner);
            }
            s_yaw_vision_active = true;
        }

        // Vision pitch tracking: set pitch target from vision error
        // Same approach as yaw but with angle limit clamping
        if (pitch && pitch->angle_initialized) {
            const float pitch_ticks_per_rad = pitch_max / (2.0f * (float)M_PI);
            float pitch_err_ticks = cmd->vision_pitch_err_rad * pitch_ticks_per_rad;
            pitch->angle_target = (float)pitch->angle_raw + pitch_err_ticks;

            if (!s_pitch_vision_active) {
                PID_Reset(&pitch->pid_outer);
            }
            s_pitch_vision_active = true;
        }
    } else {
        s_yaw_vision_active = false;
        s_pitch_vision_active = false;
    }

    // Spin-hold mode: hold gimbal absolute yaw (deg) using gimbal IMU yaw_total_angle.
    // Target is carried via yaw_target_memo; enable flag via yaw_rate_memo.
    if (use_spin_hold) {
        if (yaw && yaw->angle_initialized) {
            // Calculate angle error with proper wrapping to [-180, 180] range
            float yaw_err_deg = cmd->yaw_target_memo - current_yaw;

            // Wrap error to shortest path
            while (yaw_err_deg > 180.0f) yaw_err_deg -= 360.0f;
            while (yaw_err_deg < -180.0f) yaw_err_deg += 360.0f;

            // angle_target lives in the same [0, 360] degree frame as
            // (shifted) ekf_yaw, because MAX_YAW_ANGLE == 360 and the outer
            // yaw loop compares angle_target against ekf_yaw directly.
            float angles_per_deg = MAX_YAW_ANGLE / 360.0f;
            float err_ticks = yaw_err_deg * angles_per_deg;
            yaw->angle_target = (float)current_yaw + err_ticks;

            // Beyblading diagnostic: target(deg), current(deg), raw err, wrapped err,
            // encoder angle_raw, computed angle_target (ticks)
            LOG_CSV(LOG_TAG_GIM, "SPINHOLD,%.2f,%.2f,%.2f,%.1f,%.1f\r\n",
                    cmd->yaw_target_memo,
                    current_yaw,
                    yaw_err_deg,
                    (float)yaw->angle_raw,
                    yaw->angle_target);

            if (!s_yaw_spin_hold_active) {
                PID_Reset(&yaw->pid_outer);
                PID_Reset(&yaw->pid_inner);
            }
            s_yaw_spin_hold_active = true;
        } else {
            s_yaw_spin_hold_active = false;
        }
    } else {
        s_yaw_spin_hold_active = false;
    }

    // Update angle target based on joystick input and dt
    if (!use_vision_target) {
        if (!use_spin_hold) {
            yaw->angle_target += yaw_direction * powf(cmd->yaw_rate, 3.0f) * MAX_YAW_ANGLE_CHANGE * dt;
        }
        pitch->angle_target += pitch_direction * MAX_PITCH_ANGLE_CHANGE * powf(cmd->pitch_rate, 3.0f) * dt;
    }

    // Clamp target lead: prevent angle_target from running away from actual yaw
    // when the motor can't keep up (e.g. belt drive asymmetry). Without this,
    // releasing the stick after sustained hard input leaves a large accumulated
    // error that causes violent oscillation.
    {
        float lead = yaw->angle_target - current_yaw;
        while (lead >  MAX_YAW_ANGLE / 2.0f) lead -= MAX_YAW_ANGLE;
        while (lead < -MAX_YAW_ANGLE / 2.0f) lead += MAX_YAW_ANGLE;
        if      (lead >  MAX_YAW_TARGET_LEAD) yaw->angle_target = current_yaw + MAX_YAW_TARGET_LEAD;
        else if (lead < -MAX_YAW_TARGET_LEAD) yaw->angle_target = current_yaw - MAX_YAW_TARGET_LEAD;
    }

    // Set target angles to be in valid angle space
    while (yaw->angle_target >= MAX_YAW_ANGLE) yaw->angle_target -= MAX_YAW_ANGLE;
    while (yaw->angle_target < 0.0f) yaw->angle_target += MAX_YAW_ANGLE;
    if (pitch->angle_target > pitch_max) pitch->angle_target = pitch_max;
    if (pitch->angle_target < pitch_min) pitch->angle_target = pitch_min;
}


// Application layer: Message subscription callbacks
static void on_gimbal_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(GimbalCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(GimbalCmd));
        
        // Execute gimbal control when command arrives
        if (s_last_cmd.enabled) {
            MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
            MotorContext_t *pitch = MotorDriver_GetContext(s_pitch_motor_id);

            // Seed yaw angle_target from ekf_yaw on the first enabled cycle so
            // the controller starts with zero error regardless of IMU init timing.
            if (yaw && yaw->angle_initialized && !s_yaw_target_seeded) {
                yaw->angle_target = s_last_sensor.ekf_yaw;
                PID_Reset(&yaw->pid_outer);
                PID_Reset(&yaw->pid_inner);
                s_yaw_target_seeded = true;
            }
            bool use_spin_hold = (!s_last_cmd.vision_valid) && (s_last_cmd.yaw_rate_memo > 0.5f);

            // Update dt
            uint32_t now = HAL_GetTick();
            dt = (last_tick > 0) ? (now - last_tick) / 1000.0f : 0.005f; // seconds
            last_tick = now;

            // Compute motor currents
            GimbalController_UpdateTargets(&s_last_cmd, yaw, pitch);
            int16_t pitch_current = GimbalController_PitchControl(&s_last_sensor);
            int16_t yaw_current = GimbalController_YawControlWithCompensation(&s_last_sensor, use_spin_hold);

            // Send motor currents (module layer handles CAN)
            MotorDriver_SendCurrent(s_pitch_motor_id, pitch_current);
            MotorDriver_SendCurrent(s_yaw_motor_id, yaw_current);
            MotorDriver_FlushAll();
        } else {
            // Gimbal disabled, send zero current
            MotorDriver_SendCurrent(s_pitch_motor_id, 0);
            MotorDriver_SendCurrent(s_yaw_motor_id, 0);
            MotorDriver_FlushAll();
        }
    }
}

static void on_imu_update(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(Gimbal_Sensor_Data_t)) {
        memcpy(&s_last_sensor, ev->data, sizeof(Gimbal_Sensor_Data_t));
    }
}

void GimbalApp_Init(void) {
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));

    // Find gimbal motors by role (module layer handles config)
    uint8_t pitch_motors[1];
    uint8_t yaw_motors[1];

    if (MotorDriver_FindByRole(MOTOR_ROLE_GIMBAL_PITCH, pitch_motors, 1) > 0) {
        s_pitch_motor_id = pitch_motors[0];
    }

    if (MotorDriver_FindByRole(MOTOR_ROLE_GIMBAL_YAW, yaw_motors, 1) > 0) {
        s_yaw_motor_id = yaw_motors[0];
    }

    // Subscribe to messages
    if (!s_initialized) {
        (void)MsgCenter_Subscribe(TOPIC_GIMBAL_CMD, on_gimbal_cmd, NULL);
        (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    }

    s_initialized = true;
}

/**
 * @brief Wait for gimbal to reach initial alignment position
 * @note This function sends gimbal commands and waits for both yaw and pitch
 *       to reach their initial positions before returning.
 */
void Gimbal_WaitForAlignment(void)
{
    const float ALIGNMENT_THRESHOLD = 50.0f;  // encoder ticks
    const uint32_t TIMEOUT_MS = 10000;  // 10 seconds timeout
    const uint32_t CHECK_INTERVAL_MS = 100;

    USB_CDC_Printf("[Gimbal] Waiting for gimbal alignment...\r\n");

    // Create gimbal command to enable gimbal and hold initial position
    GimbalCmd cmd = {
        .enabled = true,
        .pitch_rate = 0.0f,
        .yaw_rate = 0.0f,
        .yaw_rate_memo = 0.0f,
        .yaw_target_memo = 0.0f,
        .vision_valid = false,
        .vision_yaw_err_rad = 0.0f,
        .vision_pitch_err_rad = 0.0f,
        .vision_ts_ms = 0
    };

    uint32_t start_time = HAL_GetTick();

    while (HAL_GetTick() - start_time < TIMEOUT_MS) {
        // Continuously send command to keep gimbal control
        MsgCenter_Publish(TOPIC_GIMBAL_CMD, &cmd, sizeof(cmd), 0);
        // Dispatcher task will process messages asynchronously

        HAL_Delay(CHECK_INTERVAL_MS);

        // Check if gimbal has reached target position
        MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
        MotorContext_t *pitch = MotorDriver_GetContext(s_pitch_motor_id);

        if (yaw && pitch && yaw->angle_initialized && pitch->angle_initialized) {
            float yaw_error = fabsf(yaw->angle_target - (float)yaw->angle_raw);
            float pitch_error = fabsf(pitch->angle_target - (float)pitch->angle_raw);

            // Handle yaw wraparound (0-MAX/2 range)
            if (yaw_error > MAX_YAW_ANGLE / 2) {
                yaw_error = MAX_YAW_ANGLE - yaw_error;
            }

            USB_CDC_Printf("[Gimbal] Yaw error: %.1f, Pitch error: %.1f\r\n",
                          yaw_error, pitch_error);

            if (yaw_error < ALIGNMENT_THRESHOLD && pitch_error < ALIGNMENT_THRESHOLD) {
                USB_CDC_Printf("[Gimbal] Alignment complete!\r\n");
                return;
            }
        }
    }

    USB_CDC_Printf("[Gimbal] Warning: Alignment timeout, continuing anyway...\r\n");
}
