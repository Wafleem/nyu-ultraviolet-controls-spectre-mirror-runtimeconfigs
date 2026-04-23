#include "FreeRTOS.h"
#include "timers.h"
#include "shooter_controller.h"
#include "motor_driver.h"
#include <string.h>
#include "message_center.h"
#include "remote_control.h"
#include "can_comm.h"
#include "cmd_controller.h"
#include "printing.h"
#include "logger.h"

// ---- debug print throttle ----
static uint32_t s_last_print_ms = 0;
#define SHOOTER_PRINT_PERIOD_MS 100U  // print every 100ms
#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

// Static variables for app wrapper
static ShootCmd s_last_cmd;
static ShootCmd s_prev_cmd;
static Gimbal_Sensor_Data_t s_last_sensor;
static ShooterController s_ctrl;
static bool s_initialized = false;
static bool is_feeding_once = false;
static RobotConfig_t s_shooter_config;

// Anti Jam
static bool is_jammed = false;
static TimerHandle_t s_jam_timer = NULL;
static TimerHandle_t s_feed_timer = NULL;
static uint32_t s_unjam_start = 0;
static bool s_unjam_active = false;

// Shooter motor configuration (dynamically assigned during init)
static uint8_t s_feed_motor_id = 0xFF;      // Turntable/feed motor
static uint8_t s_friction1_motor_id = 0xFF; // Friction wheel 1
static uint8_t s_friction2_motor_id = 0xFF; // Friction wheel 2
static uint8_t s_pusher_motor_id = 0xFF;    // Ball Pusher

static float RampTowards(float current, float target, float step)
{
    if (current < target) { current += step; if (current > target) current = target; }
    else if (current > target) { current -= step; if (current < target) current = target; }
    return current;
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) { return 0; }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

static int16_t ComputePusherCurrent(ShooterController *controller)
{
    // Deadband the pusher
    int32_t error = (int32_t)(controller->pusher_target - controller->pusher_total_angle);
    if (abs(error) < 50) {
        controller->pusher_pid.integral = 0;
        return 0;
    }

    int16_t current_cmd = PID_Calculate(&controller->pusher_pid, controller->pusher_target, controller->pusher_total_angle);

    // Speed damping: oppose motor velocity to prevent overshoot
    float speed_damping = 1.5f * (float)controller->pusher_feedback.speed;
    int32_t damped = (int32_t)current_cmd - (int32_t)speed_damping;
    return (int16_t)damped;
}

static void ToggleJamDetection(void) {
    if (s_jam_timer != NULL) {
        return;
    }
    // When feeder turns on, start checking for jams
    if (!s_prev_cmd.feed_enabled && s_last_cmd.feed_enabled) {
        xTimerStart(s_jam_timer, 0);
    }
    // When feeder turns off, stop checking for jams
    if (s_prev_cmd.feed_enabled && !s_last_cmd.feed_enabled) {
        xTimerStop(s_jam_timer, 0);
        is_jammed = false;
    }
}

void ShooterController_Init(ShooterController *controller)
{
    if (controller == NULL) return;
    memset(controller, 0, sizeof(ShooterController));

    // Find shooter motors by role (module layer handles config)
    uint8_t feed_motors[1];
    uint8_t friction_motors[2];
    uint8_t push_motors[1];

    // Find feed motor
    if (MotorDriver_FindByRole(MOTOR_ROLE_SHOOTER_FEED, feed_motors, 1) > 0) {
        s_feed_motor_id = feed_motors[0];

        // Initialize turntable PID from motor context
        MotorContext_t *ctx = MotorDriver_GetContext(s_feed_motor_id);
        if (ctx && ctx->config) {
            PID_Init(&controller->turntable_pid,
                     ctx->config->pid_outer.kp,
                     ctx->config->pid_outer.ki,
                     ctx->config->pid_outer.kd,
                     ctx->config->pid_outer.output_max,
                     ctx->config->pid_outer.integral_max,
                     ctx->config->pid_outer.error_max);
            controller->directions[0] = ctx->config->direction;
        }
    }

    // Find friction wheels
    uint8_t friction_count = MotorDriver_FindByRole(MOTOR_ROLE_SHOOTER_FRICTION, friction_motors, 2);
    if (friction_count > 0) {
        s_friction1_motor_id = friction_motors[0];

        // Initialize shooter1 PID from motor context
        MotorContext_t *ctx = MotorDriver_GetContext(s_friction1_motor_id);
        if (ctx && ctx->config) {
            PID_Init(&controller->shooter1_pid,
                     ctx->config->pid_outer.kp,
                     ctx->config->pid_outer.ki,
                     ctx->config->pid_outer.kd,
                     ctx->config->pid_outer.output_max,
                     ctx->config->pid_outer.integral_max,
                     ctx->config->pid_outer.error_max);
            controller->directions[1] = ctx->config->direction;
        }
    }

    if (friction_count > 1) {
        s_friction2_motor_id = friction_motors[1];

        // Initialize shooter2 PID from motor context
        MotorContext_t *ctx = MotorDriver_GetContext(s_friction2_motor_id);
        if (ctx && ctx->config) {
            PID_Init(&controller->shooter2_pid,
                     ctx->config->pid_outer.kp,
                     ctx->config->pid_outer.ki,
                     ctx->config->pid_outer.kd,
                     ctx->config->pid_outer.output_max,
                     ctx->config->pid_outer.integral_max,
                     ctx->config->pid_outer.error_max);
            controller->directions[2] = ctx->config->direction;
        }
    }

    // Find ball pusher
    if (MotorDriver_FindByRole(MOTOR_ROLE_SHOOTER_PUSH, push_motors, 1) > 0) {
        s_pusher_motor_id = push_motors[0];
        MotorContext_t *ctx = MotorDriver_GetContext(s_pusher_motor_id);
        if (ctx && ctx->config) {
            PID_Init(&controller->pusher_pid,
                     ctx->config->pid_outer.kp,
                     ctx->config->pid_outer.ki,
                     ctx->config->pid_outer.kd,
                     ctx->config->pid_outer.output_max,
                     ctx->config->pid_outer.integral_max,
                     ctx->config->pid_outer.error_max);
            controller->directions[3] = ctx->config->direction; 
        }
    }

}

void ShooterController_Update(ShooterController *controller, Gimbal_Sensor_Data_t* sensor_data)
{
    if (controller == NULL) return;
    (void)sensor_data;  // Not needed anymore

    // Unjam the feeder if software anti-jam is enabled
    if (ANTIJAM_ENABLED && is_jammed) {
        ShooterController_Unjam(controller);
        return;
    }
    
    // Use standardized command from cmd_controller
    controller->enabled = s_last_cmd.friction_enabled;
    
    // Set turntable target (only feed when feed_enabled)
    float turntable_target = 0.0f;
    if (s_pusher_motor_id != 0xFF) {
        if (s_last_cmd.feed_enabled && !s_prev_cmd.feed_enabled) {
            is_feeding_once = true;
            xTimerStart(s_feed_timer, 0);
        }
        if (!s_last_cmd.feed_enabled && s_prev_cmd.feed_enabled) {
            is_feeding_once = false;
            xTimerStop(s_feed_timer, 0);
        }
        turntable_target = (s_last_cmd.feed_enabled && is_feeding_once) ? s_shooter_config.feeder_speed * controller->directions[0] : 0.0f;
    } else {
        turntable_target = s_last_cmd.feed_enabled ? s_shooter_config.feeder_speed * controller->directions[0] : 0.0f;
    }
    
    
    
    // Set shooter wheel targets
    float shooter1_target = s_last_cmd.friction_enabled ? s_shooter_config.friction_wheel_speed * controller->directions[1] : 0.0f;
    float shooter2_target = s_last_cmd.friction_enabled ? s_shooter_config.friction_wheel_speed * controller->directions[2] : 0.0f;

    // Set pusher target
    controller->pusher_target = s_last_cmd.feed_enabled ? s_shooter_config.pusher_retracted_angle : s_shooter_config.pusher_extended_angle;
    
    // Apply ramping
    controller->ramped_turntable = RampTowards(controller->ramped_turntable, turntable_target, SHOOTER_RAMP_STEP);
    controller->ramped_shooter1 = RampTowards(controller->ramped_shooter1, shooter1_target, SHOOTER_RAMP_STEP);
    controller->ramped_shooter2 = RampTowards(controller->ramped_shooter2, shooter2_target, SHOOTER_RAMP_STEP);
}

void ShooterController_ComputeCurrents(ShooterController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;

    // Compute currents for each shooter motor
    controller->output_currents[0] = ComputeSingleMotorCurrent(&controller->turntable_pid, controller->ramped_turntable, &controller->turntable_feedback, current_tick);
    controller->output_currents[1] = ComputeSingleMotorCurrent(&controller->shooter1_pid, controller->ramped_shooter1, &controller->shooter1_feedback, current_tick);
    controller->output_currents[2] = ComputeSingleMotorCurrent(&controller->shooter2_pid, controller->ramped_shooter2, &controller->shooter2_feedback, current_tick);
    controller->output_currents[3] = ComputePusherCurrent(controller);

    // Send motor currents (module layer handles CAN)
    if (s_feed_motor_id != 0xFF) {
        MotorDriver_SendCurrent(s_feed_motor_id, controller->output_currents[0]);
    }
    if (s_friction1_motor_id != 0xFF) {
        MotorDriver_SendCurrent(s_friction1_motor_id, controller->output_currents[1]);
    }
    if (s_friction2_motor_id != 0xFF) {
        MotorDriver_SendCurrent(s_friction2_motor_id, controller->output_currents[2]);
    }
    if (s_pusher_motor_id != 0xFF) {
        MotorDriver_SendCurrent(s_pusher_motor_id, controller->output_currents[3]);
    }

    // Flush all pending motor commands
    MotorDriver_FlushAll();

        // --- Tuning print: target vs actual shooter speeds  ---
    if (current_tick - s_last_print_ms >= SHOOTER_PRINT_PERIOD_MS) {
        s_last_print_ms = current_tick;

        int16_t s1 = controller->shooter1_feedback.speed;
        int16_t s2 = controller->shooter2_feedback.speed;
        int16_t s3 = controller->turntable_feedback.speed;
        int16_t a4 = controller->pusher_total_angle;

        LOG_INFO(LOG_TAG_SHO, "Shooter: actual %d %d %d %d, target %.2f %.2f %.2f %.2f, currents %d %d %d %d",
            s1, s2, s3, a4,
            controller->ramped_shooter1, controller->ramped_shooter2, controller->ramped_turntable, controller->pusher_target,
            controller->output_currents[1], controller->output_currents[2], controller->output_currents[0], controller->output_currents[3]);
    }
}

void ShooterController_Stop(ShooterController *controller)
{
    if (controller == NULL) return;
    controller->enabled = false;
    controller->turntable_target = 0.0f;
    controller->shooter1_target = 0.0f;
    controller->shooter2_target = 0.0f;
    controller->pusher_target = 0.0f;
    controller->turntable_pid.integral = 0.0f;
    controller->shooter1_pid.integral = 0.0f;
    controller->shooter2_pid.integral = 0.0f;
    controller->pusher_pid.integral = 0.0f; 
}

void ShooterController_UpdateMotorFeedback(ShooterController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL) return;

    Motor_Feedback *feedback = NULL;

    // Dynamically match motor_id to feedback structure
    if (motor_id == s_feed_motor_id) {
        feedback = &controller->turntable_feedback;
    }
    else if (motor_id == s_friction1_motor_id) {
        feedback = &controller->shooter1_feedback;
    }
    else if (motor_id == s_friction2_motor_id) {
        feedback = &controller->shooter2_feedback;
    }
    else if (motor_id == s_pusher_motor_id) {
        feedback = &controller->pusher_feedback;

        // Initialize pusher angles
        if (!controller->pusher_initialized) {
            controller->pusher_last_angle = angle;
            controller->pusher_total_angle = 0;
            controller->pusher_initialized = true;
        }

        // Accumulate angle changes
        int32_t delta = (int32_t)angle - (int32_t)controller->pusher_last_angle;
        // Handle wraparound (0–8191)
        if (delta > 4096) delta -= 8192;
        else if (delta < -4096) delta += 8192;

        controller->pusher_total_angle += delta;
        controller->pusher_last_angle = angle;
    }
    else {
        return;  // Not a shooter motor
    }

    if (feedback != NULL) {
        feedback->angle = angle;
        feedback->speed = speed;
        feedback->current = current;
        feedback->temp = temp;
        feedback->last_update_time = current_tick;
    }
}

void ShooterController_Unjam(ShooterController *controller)
{
    if (controller == NULL) return;

    if (!s_unjam_active) {
        s_unjam_active = true;
        s_unjam_start = xTaskGetTickCount();
    }

    float unjam_speed = -s_shooter_config.feeder_speed * controller->directions[0];
    controller->ramped_turntable = unjam_speed;

    int16_t cmd = (int16_t)(-3000);  // tune this 

    if (s_feed_motor_id != 0xFF) {
        MotorDriver_SendCurrent(s_feed_motor_id, cmd);
    }

    LOG_WARN(LOG_TAG_SHO, "UNJAM ACTIVE: reversing feeder");

    if (xTaskGetTickCount() - s_unjam_start > 3000) {
        is_jammed = false;
        s_unjam_active = false;
    }
}

// Subscription callbacks
static void on_shoot_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(ShootCmd)) {
        s_prev_cmd = s_last_cmd;  // store previous state
        memcpy(&s_last_cmd, ev->data, sizeof(ShootCmd));

        // Update controller and compute currents when command arrives
        ShooterController_Update(&s_ctrl, &s_last_sensor);
        ShooterController_ComputeCurrents(&s_ctrl, HAL_GetTick());
        if (ANTIJAM_ENABLED) {
            ToggleJamDetection();
        }
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

        // Check if this motor is a shooter motor
        if (m->id == s_feed_motor_id || m->id == s_friction1_motor_id || m->id == s_friction2_motor_id || m->id == s_pusher_motor_id) {
            ShooterController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}

static void on_jam_check(TimerHandle_t xTimer) 
{
    int16_t speed = s_ctrl.turntable_feedback.speed;
    is_jammed = (-50 < speed && speed < 50);
}

static void on_feed_once(TimerHandle_t xTimer)
{
    is_feeding_once = false;
}

void ShooterApp_Init(const RobotConfig_t *config) {
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));

    // Initialize shooter controller (module layer handles config)
    ShooterController_Init(&s_ctrl);

    if (!s_initialized) {
        (void)MsgCenter_Subscribe(TOPIC_SHOOT_CMD, on_shoot_cmd, NULL);
        (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
        (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
        s_jam_timer = xTimerCreate("JamTimer", pdMS_TO_TICKS(3000), pdTRUE, NULL, on_jam_check);
        s_feed_timer = xTimerCreate("FeedTimer", pdMS_TO_TICKS(1000), pdFALSE, NULL, on_feed_once);
    }
    if (config) {
        s_shooter_config = *config;
    }
    s_initialized = true;
}

ShooterController* ShooterApp_GetController(void) {
    return &s_ctrl;
}


