#include "FreeRTOS.h"
#include "message_center.h"
#include "ref_structs.h"
#include "remote_control.h"
#include "printing.h"
#include "vision_comm.h"
#include "sdcard.h"
#include <string.h>

static uint8_t sd_ready = 0;
static uint8_t header_written = 0;
static TickType_t flush_period_ticks;
static TickType_t last_flush_time;

static power_heat_data_t s_log_heat;
static robot_status_t s_log_robot_status;
static shoot_data_t s_log_shoot;
static projectile_allowance_t s_log_allowance;
static RC_ctrl_t s_log_rc;
static Vision_Recv_s s_log_vision;

static uint8_t s_log_heat_valid = 0;
static uint8_t s_log_robot_status_valid = 0;
static uint8_t s_log_shoot_valid = 0;
static uint8_t s_log_allowance_valid = 0;
static uint8_t s_log_rc_valid = 0;
static uint8_t s_log_vision_valid = 0;


static void on_logger_robot_status(const MsgEvent *ev, void *user_data);
static void on_logger_heat(const MsgEvent *ev, void *user_data);
static void on_logger_shoot(const MsgEvent *ev, void *user_data);
static void on_logger_allowance(const MsgEvent *ev, void *user_data);
static void on_logger_rc(const MsgEvent *ev, void *user_data);
static void on_logger_vision(const MsgEvent *ev, void *user_data);


void SDCard_Logger_Init(void) {
    flush_period_ticks = pdMS_TO_TICKS(1000);  // flush every 1s
    last_flush_time = xTaskGetTickCount();

    // Subscribe telemetry topics
    MsgCenter_Subscribe(TOPIC_BARREL_HEAT, on_logger_heat, NULL);
    MsgCenter_Subscribe(TOPIC_ROBOT_STATUS, on_logger_robot_status, NULL);
    MsgCenter_Subscribe(TOPIC_SHOOT_DATA, on_logger_shoot, NULL);
    MsgCenter_Subscribe(TOPIC_PROJECTILE_ALLOWANCE, on_logger_allowance, NULL);
    MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_logger_rc, NULL);
    MsgCenter_Subscribe(TOPIC_VISION_DATA, on_logger_vision, NULL);
}

void SDCard_Logger_Task(void) {
    if (!sd_ready && SDCard_Init() == 0 && SDCard_IsReady() && SDCard_Open("match_log.csv") == 0) {
        sd_ready = 1;
        header_written = 0;
        Debug_Printf("[LoggerTask] SD ready\r\n");
    }

    if (!sd_ready || !SDCard_IsReady()) {
        sd_ready = 0;
        header_written = 0;
        return;
    }

    if (!header_written) {
        SDCard_Writeln(
        "tick_ms,"
        "robot_id,robot_lvl,curr_HP,max_HP,pw_limit,"
        "buffer_energy,shooter_heat_17mm,"
        "cool_val,heat_limit,"
        "pjt_type,shooter_num,launch_freq,pjt_speed,"
        "rc_num,"
        "ch0,ch1,ch2,ch3,ch4,ch5,"
        "s0,s1,s2,s3,failsafe,"
        "v_yaw,v_pitch,v_fire_mode,v_target_state,v_target_type,v_updated"
        );
        SDCard_Flush();
        header_written = 1;
    }

    uint32_t tick_ms = HAL_GetTick();

    // ---------------- referee ----------------
    uint32_t robot_id = 0;
    uint32_t robot_level = 0;
    uint32_t current_hp = 0;
    uint32_t max_hp = 0;
    uint32_t power_limit = 0;
    uint32_t buffer_energy = 0;
    uint32_t shooter_heat_17mm = 0;
    uint32_t cooling_value = 0;
    uint32_t heat_limit = 0;
    uint32_t projectile_type = 0;
    uint32_t shooter_number = 0;
    uint32_t launch_freq = 0;
    float projectile_speed = 0.0f;

    // ---------------- RC ----------------
    uint32_t rc_frame_count = RC_GetFrameCount();
    int32_t rc_ch0 = 0;
    int32_t rc_ch1 = 0;
    int32_t rc_ch2 = 0;
    int32_t rc_ch3 = 0;
    int32_t rc_ch4 = 0;
    int32_t rc_ch5 = 0;
    int32_t rc_s0 = 0;
    int32_t rc_s1 = 0;
    int32_t rc_s2 = 0;
    int32_t rc_s3 = 0;
    uint32_t rc_failsafe = 0;

    // ---------------- Vision ----------------
    float vision_yaw = 0.0f;
    float vision_pitch = 0.0f;
    uint32_t vision_fire_mode = 0;
    uint32_t vision_target_state = 0;
    uint32_t vision_target_type = 0;
    uint32_t vision_updated = 0;

    if (s_log_robot_status_valid) {
        robot_id = s_log_robot_status.robot_id;
        robot_level = s_log_robot_status.robot_level;
        current_hp = s_log_robot_status.current_HP;
        max_hp = s_log_robot_status.maximum_HP;
        power_limit = s_log_robot_status.chassis_power_limit;
        cooling_value = s_log_robot_status.shooter_barrel_cooling_value;
        heat_limit = s_log_robot_status.shooter_barrel_heat_limit;
    }

    if (s_log_heat_valid) {
        buffer_energy = s_log_heat.buffer_energy;
        shooter_heat_17mm = s_log_heat.shooter_heat_17mm;
    }

    if (s_log_shoot_valid) {
        projectile_type = s_log_shoot.projectile_type;
        shooter_number = s_log_shoot.shooter_number;
        launch_freq = s_log_shoot.launching_frequency;
        projectile_speed = s_log_shoot.projectile_speed;
    }

    if (s_log_rc_valid) {
        rc_ch0 = (int32_t)s_log_rc.rc.ch[0];
        rc_ch1 = (int32_t)s_log_rc.rc.ch[1];
        rc_ch2 = (int32_t)s_log_rc.rc.ch[2];
        rc_ch3 = (int32_t)s_log_rc.rc.ch[3];
        rc_ch4 = (int32_t)s_log_rc.rc.ch[4];
        rc_ch5 = (int32_t)s_log_rc.rc.ch[5];
        rc_s0 = (int32_t)s_log_rc.rc.s[0];
        rc_s1 = (int32_t)s_log_rc.rc.s[1];
        rc_s2 = (int32_t)s_log_rc.rc.s[2];
        rc_s3 = (int32_t)s_log_rc.rc.s[3];
        rc_failsafe = (uint32_t)s_log_rc.rc.failsafe_active;
    }

    if (s_log_vision_valid) {
        vision_yaw = s_log_vision.yaw;
        vision_pitch = s_log_vision.pitch;
        vision_fire_mode = (uint32_t)s_log_vision.fire_mode;
        vision_target_state = (uint32_t)s_log_vision.target_state;
        vision_target_type = (uint32_t)s_log_vision.target_type;
        vision_updated = (uint32_t)s_log_vision.updated;
    }

    int write_status = SDCard_Writeln(
        "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%.2f,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%.2f,%.2f,%lu,%lu,%lu,%lu",
        (unsigned long)tick_ms,
        (unsigned long)robot_id,
        (unsigned long)robot_level,
        (unsigned long)current_hp,
        (unsigned long)max_hp,
        (unsigned long)power_limit,
        (unsigned long)buffer_energy,
        (unsigned long)shooter_heat_17mm,
        (unsigned long)cooling_value,
        (unsigned long)heat_limit,
        (unsigned long)projectile_type,
        (unsigned long)shooter_number,
        (unsigned long)launch_freq,
        projectile_speed,
        (unsigned long)rc_frame_count,
        (long)rc_ch0,
        (long)rc_ch1,
        (long)rc_ch2,
        (long)rc_ch3,
        (long)rc_ch4,
        (long)rc_ch5,
        (long)rc_s0,
        (long)rc_s1,
        (long)rc_s2,
        (long)rc_s3,
        (unsigned long)rc_failsafe,
        vision_yaw,
        vision_pitch,
        (unsigned long)vision_fire_mode,
        (unsigned long)vision_target_state,
        (unsigned long)vision_target_type,
        (unsigned long)vision_updated
    );
    if (write_status < 0) {
        Debug_Printf("[LoggerTask] Write failed\r\n");
        SDCard_Close();
        SDCard_Deinit();
        sd_ready = 0;
        header_written = 0;
    }

    if ((xTaskGetTickCount() - last_flush_time) >= flush_period_ticks) {
        SDCard_Flush();
        last_flush_time = xTaskGetTickCount();
    }
}


static void on_logger_robot_status(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(robot_status_t)) {
        memcpy(&s_log_robot_status, ev->data, sizeof(robot_status_t));
        s_log_robot_status_valid = 1;
    }
}

static void on_logger_heat(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(power_heat_data_t)) {
        memcpy(&s_log_heat, ev->data, sizeof(power_heat_data_t));
        s_log_heat_valid = 1;
    }
}

static void on_logger_shoot(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(shoot_data_t)) {
        memcpy(&s_log_shoot, ev->data, sizeof(shoot_data_t));
        s_log_shoot_valid = 1;
    }
}

static void on_logger_allowance(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(projectile_allowance_t)) {
        memcpy(&s_log_allowance, ev->data, sizeof(projectile_allowance_t));
        s_log_allowance_valid = 1;
    }
}


static void on_logger_rc(const MsgEvent *ev, void *user_data)
{
  (void)user_data;
  if (ev->size == sizeof(RC_ctrl_t)) {
    memcpy(&s_log_rc, ev->data, sizeof(RC_ctrl_t));
    s_log_rc_valid = 1;
  }
}

static void on_logger_vision(const MsgEvent *ev, void *user_data)
{
  (void)user_data;

  if (ev->size == sizeof(Vision_Recv_s)) {
    memcpy(&s_log_vision, ev->data, sizeof(Vision_Recv_s));
    s_log_vision_valid = 1;
  }
}
