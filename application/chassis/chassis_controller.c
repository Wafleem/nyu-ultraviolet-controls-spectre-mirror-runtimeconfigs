#include "chassis_controller.h"
#include "motor_driver.h"
#include <string.h>
#include <math.h>
#include "message_center.h"
#include "remote_control.h"
#include "can_comm.h"
#include "printing.h"
#include "logger.h"
#include "ref_structs.h"
#include "cmd_controller.h"
#include "robot_config.h"

#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

// PMM power threshold: during supercap discharge, PMM draws idle trickle power (~1.6W).
// Only engage power scaling when PMM is actually delivering significant power.
#define PMM_TRICKLE_THRESHOLD_W 2.0f

typedef struct { float x; float y; } Pair;

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static Gimbal_Sensor_Data_t s_last_sensor;
static ChassisController s_ctrl;
static SupercapFeedbackEvent s_last_supercap;
static PowerFeedbackEvent s_last_power;     // PDB 0x404 reading; used when POWER_LIMIT_SOURCE_PDB
static robot_status_t s_last_robot_status;
static const RobotConfig_t *s_robot_config = NULL;
static bool s_initialized = false;

// Chassis speed limit: switches between normal and supercap-boosted
static int s_chassis_max_speed = CHASSIS_SPEED_NORMAL;

// Chassis motor configuration (dynamically assigned during init)
static uint8_t s_chassis_motor_ids[CHASSIS_MOTOR_COUNT];
static int8_t s_motor_directions[CHASSIS_MOTOR_COUNT];
static uint8_t s_chassis_motor_count = 0;

static void ResetPidIntegrals(ChassisController *controller)
{
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->speed_pids[i].integral = 0.0f; }
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) { return 0; }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

void ChassisController_Init(ChassisController *controller)
{
    if (controller == NULL) return;
    
    memset(controller, 0, sizeof(ChassisController));

    // Find chassis motors by role (module layer handles config)
    s_chassis_motor_count = MotorDriver_FindByRole(MOTOR_ROLE_CHASSIS_DRIVE,
                                                     s_chassis_motor_ids,
                                                     CHASSIS_MOTOR_COUNT);


    // Get motor directions and initialize PIDs from motor contexts
    for (uint8_t i = 0; i < s_chassis_motor_count; i++) {
        MotorContext_t *ctx = MotorDriver_GetContext(s_chassis_motor_ids[i]);
        if (ctx && ctx->config) {
            s_motor_directions[i] = ctx->config->direction;

            // Initialize PID from config
            PID_Init(&controller->speed_pids[i],
                     ctx->config->pid_outer.kp,
                     ctx->config->pid_outer.ki,
                     ctx->config->pid_outer.kd,
                     ctx->config->pid_outer.output_max,
                     ctx->config->pid_outer.integral_max,
                     ctx->config->pid_outer.error_max);

            controller->target_speeds[i] = 0.0f;
        }
    }
}

void ChassisController_Update(ChassisController *controller, Gimbal_Sensor_Data_t* sensor_data)
{
    if (controller == NULL) return;

    float vx_norm = s_last_cmd.vx;
    float vy_norm = s_last_cmd.vy;
    float wz_norm = s_last_cmd.wz;
    float scale = (float)s_chassis_max_speed / 2.0f;
    float omega = wz_norm * scale;

    // Coordinate transformation is already done in cmd_controller for spin mode
    // So we directly use the vx/vy from command without additional transformation
    float vx = vx_norm * scale;
    float vy = -vy_norm * scale;  // Negate vy to fix direction - Safal 1-17-2026. 
    controller->target_speeds[0] = s_motor_directions[0] * (vx - vy - omega);
    controller->target_speeds[1] = s_motor_directions[1] * (vx + vy + omega);
    controller->target_speeds[2] = s_motor_directions[2] * (vx + vy - omega);
    controller->target_speeds[3] = s_motor_directions[3] * (vx - vy + omega);

    controller->running = s_last_cmd.enabled;
}

void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;
    
    // Debug output for chassis motor currents (every 200ms)
    LOG_INFO(LOG_TAG_CHA, "en=%d\r\n", (int)s_last_cmd.enabled);
    for (uint8_t i = 0; i < s_chassis_motor_count; i++) {
        LOG_INFO(
            LOG_TAG_CHA,
            " i=%d id=%d tgt=%.1f fb=%.1f dt=%lu out=%d\r\n",
            i,
            s_chassis_motor_ids[i],
            controller->target_speeds[i],
            controller->motor_feedbacks[i].speed,
            (unsigned long)(HAL_GetTick() - controller->motor_feedbacks[i].last_update_time),
            controller->output_currents[i]
        );
    }


    // Compute current for each chassis motor
    for (int i = 0; i < s_chassis_motor_count; i++) {
        int16_t motor_current = ComputeSingleMotorCurrent(
            &controller->speed_pids[i],
            controller->target_speeds[i],
            &controller->motor_feedbacks[i],
            current_tick
        );
        controller->output_currents[i] = motor_current;
    }

    // ===== POWER LIMITING: branch on configured source =====
    // Supercap robots use Wraith pmm_w (CAN 0x405). PDB-only robots (e.g. Hero without
    // supercap mounted) fall back to PDB total power (CAN 0x404). Speed boost is
    // supercap-only: PDB robots never raise s_chassis_max_speed.
    const PowerLimitSource_e src = (s_robot_config != NULL)
        ? s_robot_config->power_limit_source
        : POWER_LIMIT_SOURCE_SUPERCAP;
    float target_w = s_last_robot_status.chassis_power_limit * 0.90f;  // 90% headroom before hard cap
    float measured_w = 0.0f;
    bool engage_limiter = false;
    bool supercap_discharging = false;

    if (src == POWER_LIMIT_SOURCE_SUPERCAP) {
        supercap_discharging = (s_last_supercap.mode == 2);  // DISCHARGING
        // During supercap discharge, PMM draws only trickle power, so we skip.
        if (!supercap_discharging && s_last_supercap.pmm_w > PMM_TRICKLE_THRESHOLD_W) {
            measured_w = s_last_supercap.pmm_w;
            engage_limiter = true;
        }
        // Debug_Printf("src=SCAP pmm=%.1fW cap=%.1f%% mode=%d discharging=%d\r\n",
        //          (double)s_last_supercap.pmm_w, (double)s_last_supercap.voltage_pct,
        //          (int)s_last_supercap.mode, (int)supercap_discharging);
    } else {  // POWER_LIMIT_SOURCE_PDB
        if (s_last_power.power > PMM_TRICKLE_THRESHOLD_W) {
            measured_w = s_last_power.power;
            engage_limiter = true;
        }
        //Debug_Printf("src=PDB power=%.1fW\r\n", (double)s_last_power.power);
    }

    // Speed boost: only meaningful when a real supercap is discharging.
    s_chassis_max_speed = (src == POWER_LIMIT_SOURCE_SUPERCAP && supercap_discharging)
        ? CHASSIS_SPEED_SUPERCAP : CHASSIS_SPEED_NORMAL;

    if (engage_limiter && measured_w > target_w) {
        float ratio = target_w / measured_w;
        if (ratio > 1.0f) ratio = 1.0f;  // Never scale UP, only down

        LOG_INFO(LOG_TAG_CHA, "POWER SCALE: src=%s meas=%.1fW limit=%.0fW scale=%.3f\r\n",
                 (src == POWER_LIMIT_SOURCE_SUPERCAP) ? "SCAP" : "PDB",
                 measured_w, s_last_robot_status.chassis_power_limit, ratio);

        for (int j = 0; j < s_chassis_motor_count; j++) {
            controller->output_currents[j] = (int16_t)((float)controller->output_currents[j] * ratio);
        }
    }

    // Send all computed (and possibly scaled) currents to motors
    for (int i = 0; i < s_chassis_motor_count; i++) {
        MotorDriver_SendCurrent(s_chassis_motor_ids[i], controller->output_currents[i]);
    }
}

void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT])
{
    if (controller == NULL || speeds == NULL) return;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->target_speeds[i] = speeds[i]; }
}

void ChassisController_Stop(ChassisController *controller)
{
    if (controller == NULL) return;
    controller->running = false;
    ResetPidIntegrals(controller);
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->target_speeds[i] = 0.0f; }
}

const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller)
{
    if (controller == NULL) return NULL;
    return controller->output_currents;
}

bool ChassisController_IsRunning(const ChassisController *controller)
{
    if (controller == NULL) return false;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { if (controller->target_speeds[i] != 0) { return true; } }
    return false;
}

void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{

    if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
    controller->motor_feedbacks[motor_id].angle = angle;
    controller->motor_feedbacks[motor_id].speed = speed;
    controller->motor_feedbacks[motor_id].current = current;
    controller->motor_feedbacks[motor_id].temp = temp;
    controller->motor_feedbacks[motor_id].last_update_time = current_tick;
}

// Subscription callbacks
static void on_chassis_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(ChassisCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
    }
}

static void on_imu_update(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(Gimbal_Sensor_Data_t)) {
        memcpy(&s_last_sensor, ev->data, sizeof(Gimbal_Sensor_Data_t));
    }
}

static void on_motor_feedback(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;

        // Check if this motor is a chassis motor
        for (uint8_t i = 0; i < s_chassis_motor_count; i++) {
            if (m->id == s_chassis_motor_ids[i]) {
                // Found matching chassis motor, update feedback at index i
                ChassisController_UpdateMotorFeedback(&s_ctrl, i, m->angle, m->speed, m->current, m->temp, m->tick_ms);
                break;
            }
        }
    }
}

static void on_chassis_power(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(PowerFeedbackEvent)) {
        memcpy(&s_last_power, ev->data, sizeof(PowerFeedbackEvent));
    }
}

static void on_supercap_feedback(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(SupercapFeedbackEvent)) {
        memcpy(&s_last_supercap, ev->data, sizeof(SupercapFeedbackEvent));

        static const char *const mode_names[] = {
            "DISABLED", "CHARGING", "DISCHARGING", "UNDERVOLTAGE"
        };
        const char *mode_str = (s_last_supercap.mode < 4) ? mode_names[s_last_supercap.mode] : "UNKNOWN";

        LOG_INFO(LOG_TAG_DEBUG,
                "[Wraith] PMM=%.1fW Chassis=%.1fW Cap=%.1f%% Mode=%s tick=%lu\r\n",
                (double)s_last_supercap.pmm_w, (double)s_last_supercap.chassis_w,
                (double)s_last_supercap.voltage_pct, mode_str,
                (unsigned long)s_last_supercap.tick_ms);
    }
}

static void on_robot_status(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(robot_status_t)) {
        memcpy(&s_last_robot_status, ev->data, sizeof(robot_status_t));
        LOG_INFO(LOG_TAG_DEBUG, "ROBOT_ID=%u,CHASSIS_LIMIT=%u\r\n",
                 s_last_robot_status.robot_id, s_last_robot_status.chassis_power_limit);
    }
}

void ChassisApp_Init(const RobotConfig_t *robot_config) {

    s_robot_config = robot_config;
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));
    memset(&s_last_power, 0, sizeof(s_last_power));
    memset(&s_last_supercap, 0, sizeof(s_last_supercap));

    // Initialize chassis controller (module layer handles config)
    ChassisController_Init(&s_ctrl);

    if (!s_initialized) {
        (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
        (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
        (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
        (void)MsgCenter_Subscribe(TOPIC_SUPERCAP_FEEDBACK, on_supercap_feedback, NULL);
        (void)MsgCenter_Subscribe(TOPIC_CHASSIS_POWER, on_chassis_power, NULL);
        (void)MsgCenter_Subscribe(TOPIC_ROBOT_STATUS, on_robot_status, NULL);
    }
    s_initialized = true;
}

void ChassisApp_Tick(void) {
    if (!s_initialized) return;
    ChassisController_Update(&s_ctrl, &s_last_sensor);
    ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
}

ChassisController* ChassisApp_GetController(void) {
    return &s_ctrl;
}
