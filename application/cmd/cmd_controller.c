#include "cmd_controller.h"
#include "message_center.h"
#include "remote_control.h"
#include "imu.h"
#include "vision_comm.h"
#include "referee.h"
#include "ref_structs.h"
#include "can_comm.h"
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
#define SPIN_GIMBAL_YAW_ADJ_DEG_PER_S (120.0f) // manual yaw adjustment rate when in spin mode (deg/s)

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
static bool s_gimbal_follow_mode = false;





static float yaw_storage=0.0f;
static uint32_t s_last_spin_dbg_tick = 0; // rate limiter for SPINDBG prints (tagged)

// Local state storage
static RC_ctrl_t s_last_rc;
static Gimbal_Sensor_Data_t s_last_sensor;
static Vision_Recv_s s_last_vision;
static CanRxFrame s_last_can;
static bool s_initialized = false;

// RC health tracking
static uint32_t s_last_rc_update_time = 0;
static bool s_rc_ever_connected = false;  // Track if we've ever received valid RC data
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
static float normalize_angle_180(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
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

// Callback for IMU update
static void on_imu_update(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(Gimbal_Sensor_Data_t)) {
        memcpy(&s_last_sensor, ev->data, sizeof(Gimbal_Sensor_Data_t));
    }
}

// Callback for vision data update
static void on_vision_update(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(Vision_Recv_s)) {
        memcpy(&s_last_vision, ev->data, sizeof(Vision_Recv_s));
    }
}

static void on_can_rx(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(CanRxFrame)) {
        memcpy(&s_last_can, ev->data, sizeof(CanRxFrame));
        LOG_INFO(LOG_TAG_CAN, "ID: %x, Data (%u bytes): %x %x %x %x %x %x %x %x",
                s_last_can.std_id, s_last_can.dlc,
                s_last_can.data[0], s_last_can.data[1],
                s_last_can.data[2], s_last_can.data[3],
                s_last_can.data[4], s_last_can.data[5],
                s_last_can.data[6], s_last_can.data[7]);
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
    int16_t wz_raw = apply_deadband((int16_t)(rc->rc.ch[4]), JOYSTICK_DEADBAND);

    // Disabling rotation for now in case people don't know about knob controls
    wz_raw = 0;

    // Convert to normalized values (-1.0 to 1.0)
    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    float vx_f = -(float)vx_raw / max_input; //added back minus sign to flip forward/backward
    float vy_f = -(float)vy_raw / max_input;
    float wz_n = (float)wz_raw / max_input;

    if ((spin_mode || gimbal_follow_mode) && sensor != NULL) {
        // Spin mode OR Gimbal-follow mode: joystick input is in gimbal frame.
        // Need to convert joystick input from gimbal frame to chassis frame.

        // Calculate offset angle: chassis yaw - gimbal yaw
        // No separate chassis IMU on this hardware, so offset is 0
        // (chassis frame == gimbal frame for field-oriented control)
        float gimbal_yaw_norm = normalize_angle_180(sensor->ekf_yaw);
        float offset_angle = normalize_angle_180(0.0f - gimbal_yaw_norm);

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
            s_chassis_cmd.vx = vy_c;
            s_chassis_cmd.vy = -vx_c;
            s_chassis_cmd.wz = omega;
            // In spin mode we always enable chassis so it keeps rotating even with sticks centered.
            s_chassis_cmd.enabled = true;
        } else {
            // Gimbal-follow mode: chassis does NOT auto-rotate, manual wz control
            // Swap vx_c and vy_c to match chassis coordinate system, negate vy for correct direction
            s_chassis_cmd.vx = vy_c;
            s_chassis_cmd.vy = -vx_c;
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
static void process_gimbal_command(const RC_ctrl_t *rc, const Gimbal_Sensor_Data_t *sensor, bool spin_mode) {
    if (rc == NULL) {
        // RC disconnected, disable gimbal
        s_gimbal_cmd.enabled = false;
        s_gimbal_cmd.pitch_rate = 0.0f;
        s_gimbal_cmd.yaw_rate = 0.0f;
        s_gimbal_cmd.vision_valid = false;
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
    
    if (s_last_vision.updated) {
        s_last_vision.updated = 0;
        s_gimbal_cmd.vision_ts_ms = HAL_GetTick();
        if (s_last_vision.target_state != NO_TARGET) {
            s_gimbal_cmd.vision_valid = true;
            s_gimbal_cmd.vision_yaw_err_rad = s_last_vision.yaw;
            s_gimbal_cmd.vision_pitch_err_rad = s_last_vision.pitch;
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
    yaw_storage = yaw_raw;
}

void CmdController_Init(void) {
    if (s_initialized) {
        return;
    }
    memset(&s_last_rc, 0, sizeof(s_last_rc));
    memset(&s_last_sensor, 0, sizeof(Gimbal_Sensor_Data_t));
    memset(&s_last_vision, 0, sizeof(s_last_vision));
    memset(&s_chassis_cmd, 0, sizeof(s_chassis_cmd));
    memset(&s_shoot_cmd, 0, sizeof(s_shoot_cmd));
    memset(&s_gimbal_cmd, 0, sizeof(s_gimbal_cmd));

    (void)MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_VISION_DATA, on_vision_update, NULL);
    // Avoid subscription overhead if CAN logger is disabled
    if (LOG_ENABLE_CAN) {
        (void)MsgCenter_Subscribe(TOPIC_CAN_RX, on_can_rx, NULL);
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
    bool rc_healthy = s_rc_ever_connected && !rc_timeout;

    // If RC is unhealthy, pass NULL to stop robot gracefully
    const RC_ctrl_t *rc_ptr = rc_healthy ? &s_last_rc : NULL;

    // Log RC state changes
    static bool last_rc_healthy = false;
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
    }

    // If RC is unhealthy, disable all modes and pass NULL (robot stops)
    if (!rc_healthy) {
    s_spin_mode = false;
    s_gimbal_follow_mode = false;

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
    if (s_last_vision.updated) {
        s_last_vision.updated = 0;
        s_gimbal_cmd.vision_ts_ms = HAL_GetTick();
        if (s_last_vision.target_state != NO_TARGET) {
            s_gimbal_cmd.vision_valid = true;
            s_gimbal_cmd.vision_yaw_err_rad = s_last_vision.yaw;
            s_gimbal_cmd.vision_pitch_err_rad = s_last_vision.pitch;
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
    static uint32_t last_vis_dbg = 0;
    if (HAL_GetTick() - last_vis_dbg >= 200) {  // 5Hz
        last_vis_dbg = HAL_GetTick();
        USB_CDC_Printf("VIS,%d,%d,%d,%.4f,%.4f,v=%d\r\n",
                    s_last_vision.fire_mode,
                    s_last_vision.target_state,
                    s_last_vision.target_type,
                    s_last_vision.pitch,
                    s_last_vision.yaw,
                    s_gimbal_cmd.vision_valid);
}

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
    bool gimbal_follow_now = switch_is_down(s_last_rc.rc.s[0]);
    bool spin_now = switch_is_down(s_last_rc.rc.s[1]);

    // Spin mode rising edge: latch current gimbal absolute yaw as hold target
    if (spin_now && !s_spin_mode)
    {
        s_spin_hold_yaw_deg = s_last_sensor.ekf_yaw;
    }

    s_gimbal_follow_mode = gimbal_follow_now;
    s_spin_mode = spin_now;

    // Process control input (rc_ptr is guaranteed non-NULL here)
    process_chassis_command(rc_ptr, &s_last_sensor, s_spin_mode, s_gimbal_follow_mode);
    process_shooter_command(rc_ptr);
    process_gimbal_command(rc_ptr, &s_last_sensor, s_spin_mode);

    // Debug output for spin mode (every 100ms)
    if (HAL_GetTick() - s_last_spin_dbg_tick >= 100) {
        s_last_spin_dbg_tick = HAL_GetTick();

        float yaw_err_deg = s_spin_hold_yaw_deg - s_last_sensor.ekf_yaw;
        LOG_CSV(LOG_TAG_CMD, "SPINDBG,%lu,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\r\n",
                       (unsigned long)s_last_spin_dbg_tick,
                       (unsigned int)(s_spin_mode ? 1U : 0U),
                       (unsigned int)((uint8_t)s_last_rc.rc.s[0]),
                       (unsigned int)((uint8_t)s_last_rc.rc.s[1]),
                       0.0f /* no chassis IMU */,
                       s_last_sensor.ekf_yaw,
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

