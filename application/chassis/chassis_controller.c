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

#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

typedef struct { float x; float y; } Pair;

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static Gimbal_Sensor_Data_t s_last_sensor;
static ChassisController s_ctrl;
static PowerFeedbackEvent s_last_power;
static robot_status_t s_last_robot_status;
static bool s_initialized = false;

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
                     ctx->config->pid_outer.integral_max);

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
    float scale = (float)CHASSIS_DEMO_TARGET_SPEED / 2.0f;
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

        // Send motor current (module layer handles CAN)
        MotorDriver_SendCurrent(s_chassis_motor_ids[i], motor_current);
    }

    // Flush all pending motor commands
    MotorDriver_FlushAll();
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
        // Update controller and compute currents when command arrives
        ChassisController_Update(&s_ctrl, &s_last_sensor);
        ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
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

static void on_chassis_power_update(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(PowerFeedbackEvent)) {
        memcpy(&s_last_power, ev->data, sizeof(PowerFeedbackEvent));
        LOG_INFO(LOG_TAG_DEBUG, "POWER=%f\r\n", (double)s_last_power.power);
    }
}

static void on_robot_status(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(robot_status_t)) {
        memcpy(&s_last_robot_status, ev->data, sizeof(robot_status_t));
        LOG_INFO(LOG_TAG_DEBUG, "ROBOT_ID=%u\r\n", s_last_robot_status.robot_id);
    }
}

void ChassisApp_Init(void) {
    
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));

    // Initialize chassis controller (module layer handles config)
    ChassisController_Init(&s_ctrl);

    if (!s_initialized) {
        (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
        (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
        (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
        (void)MsgCenter_Subscribe(TOPIC_CHASSIS_POWER, on_chassis_power_update, NULL);
        (void)MsgCenter_Subscribe(TOPIC_ROBOT_STATUS, on_robot_status, NULL);
    }
    s_initialized = true;
}

ChassisController* ChassisApp_GetController(void) {
    return &s_ctrl;
}
