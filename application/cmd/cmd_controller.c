#include "cmd_controller.h"
#include "message_center.h"
#include "remote_control.h"
#include "imu.h"
#include "vision_comm.h"
#include "referee.h"
#include "ref_structs.h"
#include "can_comm.h"
#include "can_manager.h"
#include "printing.h"
#include "logger.h"
#include "logger_config.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define SAMPLE_COUNT 10
#define REFRESH_HZ   200
#define REFRESH_DT   (1.0 / REFRESH_HZ)
#define VISION_CMD_TIMEOUT_MS 80u

// ==========================
// Small gyro (spinning) mode
// ==========================
// Trigger: left switch in MID position.
// Behavior:
//  - Chassis: constant spin + allow translation (field-oriented control).
//  - Gimbal yaw: hold gimbal IMU absolute yaw steady (controlled by gimbal module's internal PID).
//
// NOTE: Tune these parameters on robot if needed.
#define SPIN_WZ_NORM                 (0.33f)   // chassis spin rate command (normalized, 0-1)
#define SPIN_TRANSLATE_LIMIT_NORM    (1.00f)   // max translation velocity in spin mode (normalized)
#define SPIN_GIMBAL_YAW_ADJ_DEG_PER_S (150.0f) // manual yaw adjustment rate when in spin mode (deg/s)

static bool  s_spin_mode = false;
static float s_spin_hold_yaw_deg = 0.0f;       // target absolute yaw (deg, gimbal IMU yaw_total_angle)

// ==========================
// Gimbal-oriented follow mode
// ==========================
// Trigger: left switch in UP position.
// Behavior:
//  - Chassis: movement direction follows gimbal orientation (field-oriented control).
//  - Chassis does NOT auto-spin (wz controlled manually by joystick).
//  - Gimbal: normal manual control.
static bool s_gimbal_follow_mode = true;





static float yaw_storage=0.0f;
static uint32_t s_last_spin_dbg_tick = 0; // rate limiter for SPINDBG prints (tagged)

// Local state storage
static const RobotConfig_t *s_robot_config;
static RC_ctrl_t s_last_rc;
static Gimbal_Sensor_Data_t s_gimbal_imu;
static ChassisIMUFeedbackEvent s_chassis_imu;
static ChassisCalibEvent s_chassis_calib;
static Vision_Recv_s s_last_vision;
static CanRxFrame s_last_can;
static bool s_initialized = false;
static projectile_allowance_t s_last_allowance;
static int s_shoot_count = 0;
static uint8_t s_yaw_motor_id = 0xFF;

// RC health tracking
static uint32_t s_last_rc_update_time = 0;
static bool s_rc_ever_connected = false;  // Track if we've ever received valid RC data
static bool s_waiting_for_neutral = true;     // Track if the stick had reached neutral position at least once
#define RC_TIMEOUT_MS 100  // 100ms timeout (DR16 sends at ~100Hz = 10ms, so ~10 missed frames)

// Command messages to publish
static ChassisCmd s_chassis_cmd;
static ShootCmd s_shoot_cmd;
static GimbalCmd s_gimbal_cmd;

// Deadband for joystick input
#define JOYSTICK_DEADBAND 100

// RC channel valid ranges (after offset subtraction: -660 to +660)
// At startup with no RC: channels = 0 - 1024 = -1024 (INVALID!)
static bool is_rc_data_valid(const RC_ctrl_t *rc) {
    if (!rc) return false;

    // Check if all channels are within valid range
    // Valid range after offset: -784 to +784 (raw 240-1808, offset 1024)
    for (int i = 0; i < 6; i++) {
        if (rc->rc.ch[i] < -1000 || rc->rc.ch[i] > 1000) {
            return false;  // Out of valid range (likely uninitialized or corrupt)
        }
    }

    // Check switches are valid (1, 2, or 3)
    for (int i = 0; i < 4; i++) {
        if (rc->rc.s[i] < 1 || rc->rc.s[i] > 3) {
            return false;
        }
    }

    return true;
}

// Normalize angle to [-180, 180] range
static float normalize_angle_360(float angle_deg)
{
    while (angle_deg > 360.0f) angle_deg -= 360.0f;
    while (angle_deg < 0.0f) angle_deg += 360.0f;
    return angle_deg;
}

// Rotate a 2D vector from gimbal frame to chassis frame
// offset_angle_deg: gimbal yaw - chassis yaw (how much gimbal is rotated relative to chassis)
// Uses counter-clockwise rotation matrix R(θ)
static void gimbal_to_chassis_frame(float vx_g, float vy_g, float offset_angle_deg, float *vx_c, float *vy_c)
{
    const float angle_rad = offset_angle_deg * (float)M_PI / 180.0f;
    const float c = cosf(angle_rad);
    const float s = sinf(angle_rad);
    // R(θ) = [cos(θ)  -sin(θ)]
    //        [sin(θ)   cos(θ)]
    *vx_c = c * vx_g - s * vy_g;
    *vy_c = s * vx_g + c * vy_g;
}


// Callback for RC update
static void on_rc_update(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(RC_ctrl_t)) {
        memcpy(&s_last_rc, ev->data, sizeof(RC_ctrl_t));

        // Update timestamp and mark as connected (if data is valid)
        if (is_rc_data_valid(&s_last_rc)) {
            s_last_rc_update_time = HAL_GetTick();
            if (!s_rc_ever_connected) {
                s_rc_ever_connected = true;
                USB_CDC_Printf("[RC] Remote control connected!\r\n");
            }
        }
    }
}
static void on_chassis_imu(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(ChassisIMUFeedbackEvent)) {
        memcpy(&s_chassis_imu, ev->data, sizeof(ChassisIMUFeedbackEvent));
    }
}
static void on_chassis_calib(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(ChassisCalibEvent)) {
        memcpy(&s_chassis_calib, ev->data, sizeof(ChassisCalibEvent));
        USB_CDC_Printf("CALIB,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
            s_chassis_calib.offset[0], s_chassis_calib.offset[1], s_chassis_calib.offset[2],
            s_chassis_calib.normal[0], s_chassis_calib.normal[1], s_chassis_calib.normal[2]);
    }
}
// Callback for IMU update
static void on_gimbal_imu(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(Gimbal_Sensor_Data_t)) {
        memcpy(&s_gimbal_imu, ev->data, sizeof(Gimbal_Sensor_Data_t));
    }
}

// Callback for vision data update
static void on_vision_update(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(Vision_Recv_s)) {
        memcpy(&s_last_vision, ev->data, sizeof(Vision_Recv_s));
    }
}

static void on_shoot_data(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(shoot_data_t)) {
        s_shoot_count++;
        LOG_INFO(LOG_TAG_DEBUG, "Projectiles Shot: %d", s_shoot_count);
    }
}

static void on_projectile_allowance(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(projectile_allowance_t)) {
        memcpy(&s_last_allowance, ev->data, sizeof(projectile_allowance_t));
    }
}

static void on_can_rx(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(CanRxFrame)) {
        memcpy(&s_last_can, ev->data, sizeof(CanRxFrame));
    }
}

// Apply deadband to joystick input
static int16_t apply_deadband(int16_t value, int16_t deadband) {
    if (value > -deadband && value < deadband) {
        return 0;
    }
    return value;
}

// Process chassis control commands
static void process_chassis_command(const RC_ctrl_t *rc, const Gimbal_Sensor_Data_t *sensor, bool spin_mode, bool gimbal_follow_mode) {
    if (rc == NULL) {
        // RC disconnected, stop chassis
        s_chassis_cmd.vx = 0.0f;
        s_chassis_cmd.vy = 0.0f;
        s_chassis_cmd.wz = 0.0f;
        s_chassis_cmd.enabled = false;
        return;
    }

    // Extract joystick values with deadband
    int16_t vx_raw = apply_deadband((int16_t)(rc->rc.ch[2]), JOYSTICK_DEADBAND);
    int16_t vy_raw = apply_deadband((int16_t)(rc->rc.ch[3]), JOYSTICK_DEADBAND);
    int16_t wz_raw = 0;  // Chassis rotate is not mapped to keybind

    // Convert to normalized values (-1.0 to 1.0)
    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    // Transform joystick values into gimbal frame
    float vx_f = (float)vx_raw / max_input;
    float vy_f = (float)vy_raw / max_input;
    float wz_n = (float)wz_raw / max_input;
    if (s_robot_config->reverse_chassis) {
        vx_f = -vx_f;
        vy_f = -vy_f;
    }

    if ((spin_mode || gimbal_follow_mode) && sensor != NULL) {
        // Spin mode OR Gimbal-follow mode: joystick input is in gimbal frame.
        // Need to convert joystick input from gimbal frame to chassis frame.

        // Calculate offset angle: chassis yaw - gimbal yaw
        // No separate chassis IMU on this hardware, so offset is 0
        // (chassis frame == gimbal frame for field-oriented control)
        float offset_angle = 0.0f;
        if (s_robot_config->chassis_yaw_source == YAW_SOURCE_DEVC) {
            float gimbal_yaw_norm = normalize_angle_360(sensor->ekf_yaw);
            float chassis_yaw = normalize_angle_360((float)s_chassis_imu.yaw);    
            offset_angle = normalize_angle_360(chassis_yaw - gimbal_yaw_norm);
        }
        else if (s_robot_config->chassis_yaw_source == YAW_SOURCE_GM6020) {
            MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
            if (yaw && yaw->angle_initialized) {
                float current_yaw = (float)yaw->angle_raw * 360 / 8192;
                // Negating offset because GM6020 axes seem reversed compared to chassis frame
                offset_angle = -normalize_angle_360(current_yaw - s_robot_config->aligned_yaw);
            }
        }

        // Rotate joystick input from gimbal frame to chassis frame
        float vx_c = 0.0f, vy_c = 0.0f;
        gimbal_to_chassis_frame(vx_f, vy_f, offset_angle, &vx_c, &vy_c);

        if (spin_mode) {
            // Spin mode: chassis auto-rotates at constant speed
            const float omega = SPIN_WZ_NORM;

            // Limit translation velocity to prevent wheel saturation
            // Use L2 norm (magnitude) instead of L1 norm for better control
            float mag = sqrtf(vx_c * vx_c + vy_c * vy_c);
            if (mag > SPIN_TRANSLATE_LIMIT_NORM) {
                float scale = SPIN_TRANSLATE_LIMIT_NORM / mag;
                vx_c *= scale;
                vy_c *= scale;
            }

            // Swap vx_c and vy_c to match chassis coordinate system, negate vy for correct direction
            s_chassis_cmd.vx = vx_c;
            s_chassis_cmd.vy = vy_c;
            s_chassis_cmd.wz = omega;
            // In spin mode we always enable chassis so it keeps rotating even with sticks centered.
            s_chassis_cmd.enabled = true;
        } else {
            // Gimbal-follow mode: chassis does NOT auto-rotate, manual wz control
            // Swap vx_c and vy_c to match chassis coordinate system, negate vy for correct direction
            s_chassis_cmd.vx = vx_c;
            s_chassis_cmd.vy = vy_c;
            s_chassis_cmd.wz = wz_n;
            // Enable chassis if any joystick is moved
            s_chassis_cmd.enabled = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);
        }
    } else {
        // Normal (original) behavior
        s_chassis_cmd.vx = vx_f;
        s_chassis_cmd.vy = vy_f;
        s_chassis_cmd.wz = wz_n;
        // Enable chassis if any joystick is moved
        s_chassis_cmd.enabled = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);
    }
}

// Process shooter control commands
static void process_shooter_command(const RC_ctrl_t *rc) {
    if (rc == NULL) {
        // RC disconnected, disable shooter
        s_shoot_cmd.friction_enabled = false;
        s_shoot_cmd.feed_enabled = false;
        return;
    }

    // Mid right switch controls shooter
    // Down: friction + feed enabled
    // Mid: friction enabled only
    // Up: all disabled
    bool right_switch_down = switch_is_down(rc->rc.s[2]);
    bool right_switch_mid = switch_is_mid(rc->rc.s[2]);
    
    s_shoot_cmd.friction_enabled = (right_switch_down || right_switch_mid);
    s_shoot_cmd.feed_enabled = right_switch_down;
}

// Process gimbal control commands
static void process_gimbal_command(const RC_ctrl_t *rc, const Gimbal_Sensor_Data_t *sensor, bool spin_mode, bool aimbot_mode) {
    if (rc == NULL) {
        // RC disconnected, disable gimbal
        s_gimbal_cmd.enabled = false;
        s_gimbal_cmd.pitch_rate = 0.0f;
        s_gimbal_cmd.yaw_rate = 0.0f;
        s_gimbal_cmd.vision_valid = false;
        s_gimbal_cmd.aimbot_mode = false;
        s_gimbal_cmd.vision_yaw_err_rad = 0.0f;
        s_gimbal_cmd.vision_pitch_err_rad = 0.0f;
        s_gimbal_cmd.vision_ts_ms = 0;
        return;
    }
    
    // Gimbal always enabled
    s_gimbal_cmd.enabled = true;
    
    // Right stick controls gimbal (ch0=yaw, ch1=pitch)
    // Apply deadband and normalize to -1.0 to 1.0
    int16_t yaw_raw = apply_deadband((int16_t)(-rc->rc.ch[0]), JOYSTICK_DEADBAND);
    int16_t pitch_raw = apply_deadband((int16_t)(rc->rc.ch[1]), JOYSTICK_DEADBAND);

    // RC signal glitch filter: reject sudden jumps >1000 units (likely signal noise/interference)
    if (abs(yaw_storage - yaw_raw) > 1000) {
        yaw_raw = yaw_storage;
    }

    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    float yaw_rate_manual = (float)yaw_raw / max_input;
    s_gimbal_cmd.pitch_rate = (float)pitch_raw / max_input;

    if (spin_mode && sensor != NULL) {
        // Allow manual yaw adjustment by shifting hold target (deg/s).
        s_spin_hold_yaw_deg += yaw_rate_manual * SPIN_GIMBAL_YAW_ADJ_DEG_PER_S * (float)REFRESH_DT;

        // Provide absolute yaw hold target to gimbal controller.
        // We reuse existing memo fields to avoid changing message struct.
        // - yaw_rate_memo: acts as a boolean flag (1.0 = spin hold active)
        // - yaw_target_memo: hold target absolute yaw (deg, gimbal IMU yaw_total_angle frame)
        s_gimbal_cmd.yaw_rate_memo = 1.0f;
        s_gimbal_cmd.yaw_target_memo = s_spin_hold_yaw_deg;

        // Keep yaw_rate at 0 in spin mode; gimbal controller will compute angle_target directly.
        s_gimbal_cmd.yaw_rate = 0.0f;
    } else {
        // Normal (original) behavior
        s_gimbal_cmd.yaw_rate = yaw_rate_manual;
        s_gimbal_cmd.yaw_rate_memo = 0.0f;
        s_gimbal_cmd.yaw_target_memo = 0.0f;
    }
    s_gimbal_cmd.aimbot_mode = aimbot_mode;
    yaw_storage = yaw_raw;
}

static void process_vision_command(Vision_Recv_s* vision) {
    if (vision->updated) {
        vision->updated = 0;
        s_gimbal_cmd.vision_ts_ms = HAL_GetTick();
        if (vision->target_state != NO_TARGET) {
            s_gimbal_cmd.vision_valid = true;
            s_gimbal_cmd.vision_yaw_err_rad = vision->yaw;
            s_gimbal_cmd.vision_pitch_err_rad = vision->pitch;
        } else {
            s_gimbal_cmd.vision_valid = false;
            s_gimbal_cmd.vision_yaw_err_rad = 0.0f;
            s_gimbal_cmd.vision_pitch_err_rad = 0.0f;
        }
    } else {
        uint32_t now = HAL_GetTick();
        if (s_gimbal_cmd.vision_valid && (now - s_gimbal_cmd.vision_ts_ms > VISION_CMD_TIMEOUT_MS)) {
            s_gimbal_cmd.vision_valid = false;
        }
    }
    LOG_INFO(LOG_TAG_VIS, "VIS,%d,%d,%d,%.4f,%.4f,v=%d\r\n",
            s_last_vision.fire_mode,
            s_last_vision.target_state,
            s_last_vision.target_type,
            s_last_vision.pitch,
            s_last_vision.yaw,
            s_gimbal_cmd.vision_valid);
}

void CmdController_Init(const RobotConfig_t *robot_config) {
    memset(&s_last_rc, 0, sizeof(s_last_rc));
    memset(&s_gimbal_imu, 0, sizeof(Gimbal_Sensor_Data_t));
    memset(&s_chassis_imu, 0, sizeof(ChassisIMUFeedbackEvent));
    memset(&s_last_vision, 0, sizeof(s_last_vision));
    memset(&s_chassis_cmd, 0, sizeof(s_chassis_cmd));
    memset(&s_shoot_cmd, 0, sizeof(s_shoot_cmd));
    memset(&s_gimbal_cmd, 0, sizeof(s_gimbal_cmd));

    if (!s_initialized) {
        (void)MsgCenter_Subscribe(TOPIC_CHASSIS_IMU, on_chassis_imu, NULL);
        (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CALIB, on_chassis_calib, NULL);
        (void)MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);
        (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_gimbal_imu, NULL);
        (void)MsgCenter_Subscribe(TOPIC_VISION_DATA, on_vision_update, NULL);
        (void)MsgCenter_Subscribe(TOPIC_SHOOT_DATA, on_shoot_data, NULL);
        (void)MsgCenter_Subscribe(TOPIC_PROJECTILE_ALLOWANCE, on_projectile_allowance, NULL);
        // Avoid subscription overhead if CAN logger is disabled
        if (LOG_ENABLE_CAN) {
            (void)MsgCenter_Subscribe(TOPIC_CAN_RX, on_can_rx, NULL);
        }
    }

    if (robot_config != NULL) {
        s_robot_config = robot_config;
    }

    uint8_t yaw_motors[1];
    if (MotorDriver_FindByRole(MOTOR_ROLE_GIMBAL_YAW, yaw_motors, 1) > 0) {
        s_yaw_motor_id = yaw_motors[0];
    }

    s_initialized = true;
}

void CmdController_Task(uint32_t current_tick) {
    (void)current_tick;

    if (!s_initialized) {
        return;
    }

    // ========== RC HEALTH CHECK (FAIL-SAFE) ==========
    // Check if RC is healthy: must have valid data AND recent update
    uint32_t time_since_last_rc = HAL_GetTick() - s_last_rc_update_time;
    bool rc_timeout = (s_rc_ever_connected && time_since_last_rc > RC_TIMEOUT_MS);
    bool rc_invalid_data = !is_rc_data_valid(&s_last_rc);
    bool rc_healthy = s_rc_ever_connected && !rc_timeout && !s_last_rc.rc.failsafe_active;

    // If RC is unhealthy, pass NULL to stop robot gracefully
    const RC_ctrl_t *rc_ptr = rc_healthy ? &s_last_rc : NULL;

    // Log RC state changes
    static bool last_rc_healthy = false;
    static bool prev_rc_failsafe_active = false;

    // If RC turns on, wait for its sticks to be in neutral
    if (!s_last_rc.rc.failsafe_active && prev_rc_failsafe_active){
        s_waiting_for_neutral = true;
    }
    // If sticks are in neutral, stop waiting. Otherwise, mark RC as unhealthy
    if (!s_last_rc.rc.failsafe_active && s_waiting_for_neutral){
        if (-JOYSTICK_DEADBAND <= s_last_rc.rc.ch[2] && s_last_rc.rc.ch[2] <= JOYSTICK_DEADBAND){
            s_waiting_for_neutral = false;
        } else {
            rc_healthy = false;
            s_waiting_for_neutral = true;
        }
    }
    
    if (rc_healthy != last_rc_healthy) {
        if (!rc_healthy) {
            if (rc_timeout) {
                USB_CDC_Printf("[RC] SIGNAL LOST - Robot stopped (timeout: %lu ms)\r\n", time_since_last_rc);
            } else if (rc_invalid_data) {
                USB_CDC_Printf("[RC] Waiting for valid RC data...\r\n");
            }
        } else {
            USB_CDC_Printf("[RC] Signal restored - Robot active\r\n");
        }
        last_rc_healthy = rc_healthy;
        prev_rc_failsafe_active = s_last_rc.rc.failsafe_active;
    }

    // If RC is unhealthy, disable all modes and pass NULL (robot stops)
    if (!rc_healthy) {
        s_spin_mode = false;

        // Disable chassis and shooter (no RC = no movement/shooting)
        s_chassis_cmd.vx = 0.0f;
        s_chassis_cmd.vy = 0.0f;
        s_chassis_cmd.wz = 0.0f;
        s_chassis_cmd.enabled = false;
        s_shoot_cmd.friction_enabled = false;
        s_shoot_cmd.feed_enabled = false;

        // But still allow vision to drive the gimbal
        s_gimbal_cmd.enabled = true;
        s_gimbal_cmd.pitch_rate = 0.0f;
        s_gimbal_cmd.yaw_rate = 0.0f;
        s_gimbal_cmd.yaw_rate_memo = 0.0f;
        s_gimbal_cmd.yaw_target_memo = 0.0f;

        // Process vision data even without RC
        process_vision_command(&s_last_vision);

        (void)MsgCenter_Publish(TOPIC_CHASSIS_CMD, &s_chassis_cmd, sizeof(s_chassis_cmd), 0);
        (void)MsgCenter_Publish(TOPIC_SHOOT_CMD, &s_shoot_cmd, sizeof(s_shoot_cmd), 0);
        (void)MsgCenter_Publish(TOPIC_GIMBAL_CMD, &s_gimbal_cmd, sizeof(s_gimbal_cmd), 0);
        return;
    }

    // ========== NORMAL OPERATION (RC HEALTHY) ==========
    // Mode selection based on left switch positions:
    // Mid Left Down  -> Small gyro mode (chassis auto-spins, gimbal holds yaw)
    // Far Left Down  -> Gimbal-follow mode (movement follows gimbal orientation, no auto-spin)
    // Both up -> Normal mode (chassis frame movement)
    // Left switch s[0] is the supercap discharge trigger.
    // Switch DOWN = discharge, anything else = charge.
    bool supercap_discharge_now = switch_is_down(s_last_rc.rc.s[0]);
    bool spin_now = switch_is_down(s_last_rc.rc.s[1]);
    bool aimbot_now = switch_is_up(s_last_rc.rc.s[3]);

    // Spin mode rising edge: latch current gimbal absolute yaw as hold target.
    if (spin_now && !s_spin_mode)
    {
        s_spin_hold_yaw_deg = s_gimbal_imu.ekf_yaw;
    }

    // Supercap (Wraith) discharge command edge detection. Send only on transitions.
    static bool s_supercap_discharge_prev = false;
    static bool s_supercap_initialized    = false;
    if (!s_supercap_initialized || supercap_discharge_now != s_supercap_discharge_prev) {
        (void)CAN_Manager_SendSupercapDischarge(supercap_discharge_now);
        LOG_INFO(LOG_TAG_CAN, "[Wraith] discharge cmd -> %s\r\n",
                 supercap_discharge_now ? "START" : "STOP");
        s_supercap_discharge_prev = supercap_discharge_now;
        s_supercap_initialized    = true;
    }

    s_spin_mode = spin_now;

    // Process control input (rc_ptr is guaranteed non-NULL here)
    process_chassis_command(rc_ptr, &s_gimbal_imu, s_spin_mode, s_gimbal_follow_mode);
    process_shooter_command(rc_ptr);
    process_gimbal_command(rc_ptr, &s_gimbal_imu, s_spin_mode, aimbot_now);
    process_vision_command(&s_last_vision);

    // Debug output
    LOG_INFO(LOG_TAG_IMU, "IMU,%.2f,%.2f,%.2f,%d,%d,%d,%.2f\r\n",
        s_gimbal_imu.ekf_yaw,
        s_gimbal_imu.ekf_pitch,
        s_gimbal_imu.ekf_roll,
        s_chassis_imu.yaw,
        s_chassis_imu.pitch,
        s_chassis_imu.roll,
        s_gimbal_imu.ekf_yaw - (float)s_chassis_imu.yaw
    );
    if (HAL_GetTick() - s_last_spin_dbg_tick >= 100) {
        s_last_spin_dbg_tick = HAL_GetTick();

        float yaw_err_deg = s_spin_hold_yaw_deg - s_gimbal_imu.ekf_yaw;
        LOG_CSV(LOG_TAG_CMD, "SPINDBG,%lu,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\r\n",
                       (unsigned long)s_last_spin_dbg_tick,
                       (unsigned int)(s_spin_mode ? 1U : 0U),
                       (unsigned int)((uint8_t)s_last_rc.rc.s[0]),
                       (unsigned int)((uint8_t)s_last_rc.rc.s[1]),
                       0.0f /* no chassis IMU */,
                       s_gimbal_imu.ekf_yaw,
                       s_spin_hold_yaw_deg,
                       yaw_err_deg,
                       s_gimbal_cmd.yaw_rate,
                       s_chassis_cmd.vx,
                       s_chassis_cmd.vy,
                       s_chassis_cmd.wz);
    }

    // Publish commands to message center (from task context, don't block)
    (void)MsgCenter_Publish(TOPIC_CHASSIS_CMD, &s_chassis_cmd, sizeof(s_chassis_cmd), 0);
    (void)MsgCenter_Publish(TOPIC_SHOOT_CMD, &s_shoot_cmd, sizeof(s_shoot_cmd), 0);
    (void)MsgCenter_Publish(TOPIC_GIMBAL_CMD, &s_gimbal_cmd, sizeof(s_gimbal_cmd), 0);
}
