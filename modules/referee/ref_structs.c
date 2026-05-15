/**
 * ref_structs.c
 *
 * This puts the contents of referee system frames into data structures.
 * Adapted from: https://github.com/RoboMaster/Development-Board-C-Examples
 */
#include "ref_structs.h"
#include "ref_protocol.h"
#include "message_center.h"
#include "printing.h"
#include "string.h"
#include "stdio.h"

#if 0  // ===== HUD-extension code (commented out for Friday-baseline test) =====
#include "main.h"

#define RETICLE_X_PX 960
#define RETICLE_Y_PX 540
#define HEAT_WARN_NUM 80u
#define HEAT_WARN_DEN 100u
#define VISION_STALE_MS 100u

static struct {
    float    sc_voltage_pct;
    uint8_t  sc_mode;
    uint8_t  vision_target_state;
    uint32_t last_vision_tick_ms;
    uint8_t  spin_mode;
    uint8_t  gimbal_follow;
    uint8_t  aimbot_engaged;
} s_hud_state;

void hud_state_set_supercap(float voltage_pct, uint8_t mode) {
    s_hud_state.sc_voltage_pct = voltage_pct;
    s_hud_state.sc_mode = mode;
}

void hud_state_set_vision(uint8_t target_state, uint32_t tick_ms) {
    s_hud_state.vision_target_state = target_state;
    s_hud_state.last_vision_tick_ms = tick_ms;
}

void hud_state_set_opstate(uint8_t spin_mode, uint8_t gimbal_follow, uint8_t aimbot_engaged) {
    s_hud_state.spin_mode = spin_mode;
    s_hud_state.gimbal_follow = gimbal_follow;
    s_hud_state.aimbot_engaged = aimbot_engaged;
}
#endif  // ===== end HUD-extension code =====

frame_header_struct_t referee_receive_header;

game_state_t game_state;
game_result_t game_result;
game_robot_HP_t game_robot_HP;

event_data_t event_data;
referee_warning_t referee_warning;
dart_info_t dart_info;

robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t robot_pos;
buff_t buff;
robot_hurt_t robot_hurt;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
rfid_status_t rfid_status;
dart_client_cmd_t dart_client_cmd;
ground_robot_position_t ground_robot_position;
radar_mark_data_t radar_mark_data;
sentry_info_t sentry_info;
radar_info_t radar_info;

robot_interaction_data_t robot_interaction_data;
custom_robot_data_t custom_robot_data;
map_command_t map_command;
remote_control_t remote_control;
map_robot_data_t map_robot_data;
custom_client_data_t custom_client_data;
map_data_t map_data;
robot_custom_data_1_t robot_custom_data_1;
robot_custom_data_2_t robot_custom_data_2;

uint8_t get_robot_id(void) {
    return robot_status.robot_id;
}

#if 0  // unused for Friday-baseline test
void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat) {
    if (heat_limit) *heat_limit = robot_status.shooter_barrel_heat_limit;
    if (heat)       *heat       = power_heat_data.shooter_heat_17mm;
}
#endif

// Initialize referee structs
void ref_structs_init(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(game_state_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));

    memset(&event_data, 0, sizeof(event_data_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&dart_info, 0, sizeof(dart_info_t));

    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff, 0, sizeof(buff_t));
    memset(&robot_hurt, 0, sizeof(robot_hurt_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    memset(&rfid_status, 0, sizeof(rfid_status_t));
    memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
    memset(&sentry_info, 0, sizeof(sentry_info_t));
    memset(&radar_info, 0, sizeof(radar_info_t));

    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&custom_robot_data, 0, sizeof(custom_robot_data_t));
    memset(&map_command, 0, sizeof(map_command_t));
    memset(&remote_control, 0, sizeof(remote_control_t));
    memset(&map_robot_data, 0, sizeof(map_robot_data_t));
    memset(&custom_client_data, 0, sizeof(custom_client_data_t));
    memset(&map_data, 0, sizeof(map_data_t));
    memset(&robot_custom_data_1, 0, sizeof(robot_custom_data_1_t));
    memset(&robot_custom_data_2, 0, sizeof(robot_custom_data_2_t));
}

// Parse referee structs from the referee frame content
void ref_structs_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(game_state_t));
            USB_CDC_Printf("[GAME] progress=%u remain=%us\r\n",
                           game_state.game_progress, game_state.stage_remain_time);
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
        }
        break;

        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&event_data, frame + index, sizeof(event_data_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
        }
        break;
        case DART_INFO_CMD_ID:
        {
            memcpy(&dart_info, frame + index, sizeof(dart_info_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            (void)MsgCenter_Publish(TOPIC_ROBOT_STATUS, &robot_status, sizeof(robot_status), 0);
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            (void)MsgCenter_Publish(TOPIC_BARREL_HEAT, &power_heat_data, sizeof(power_heat_data), 0);
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
        }
        break;
        case BUFF_CMD_ID:
        {
            memcpy(&buff, frame + index, sizeof(buff_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt, frame + index, sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            (void)MsgCenter_Publish(TOPIC_SHOOT_DATA, &shoot_data, sizeof(shoot_data), 0);
        }
        break;
        case PROJECTILE_ALLOWANCE_CMD_ID:
        {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
            (void)MsgCenter_Publish(TOPIC_PROJECTILE_ALLOWANCE, &projectile_allowance, sizeof(projectile_allowance), 0);
        }
        break;
        case RFID_STATE_CMD_ID:
        {
            memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
        }
        break;
        case DART_CLIENT_CMD_ID:
        {
            memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
        }
        break;
        case GROUND_POSITION_CMD_ID:
        {
            memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
        }
        break;
        case RADAR_MARK_CMD_ID:
        {
            memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
        }
        break;
        case SENTRY_INFO_CMD_ID:
        {
            memcpy(&sentry_info, frame + index, sizeof(sentry_info_t));
        }
        break;
        case RADAR_INFO_CMD_ID:
        {
            memcpy(&radar_info, frame + index, sizeof(radar_info_t));
        }
        break;

        case ROBOT_INTERACTION_DATA_CMD_ID:
        {
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
        }
        break;
        case CUSTOM_ROBOT_DATA_CMD_ID:
        {
            memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data_t));
        }
        case MAP_COMMAND_CMD_ID:
        {
            memcpy(&map_command, frame + index, sizeof(map_command_t));
        }
        break;
        case REMOTE_CONTROL_CMD_ID:
        {
            memcpy(&remote_control, frame + index, sizeof(remote_control_t));
        }
        break;
        case MAP_ROBOT_CMD_ID:
        {
            memcpy(&map_robot_data, frame + index, sizeof(map_robot_data_t));
        }
        break;
        case CUSTOM_CLIENT_CMD_ID:
        {
            memcpy(&custom_client_data, frame + index, sizeof(custom_client_data_t));
        }
        break;
        case MAP_DATA_CMD_ID:
        {
            memcpy(&map_data, frame + index, sizeof(map_data_t));
        }
        break;
        case ROBOT_CUSTOM_DATA_1_CMD_ID:
        {
            memcpy(&robot_custom_data_1, frame + index, sizeof(robot_custom_data_1_t));
        }
        break;
        case ROBOT_CUSTOM_DATA_2_CMD_ID:
        {
            memcpy(&robot_custom_data_2, frame + index, sizeof(robot_custom_data_2_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

#if 0  // ===== fill helpers (commented out, only single rectangle below) =====
static void fill_name(interaction_figure_t *f, const char name[3])
{
    f->figure_name[0] = (uint8_t)name[0];
    f->figure_name[1] = (uint8_t)name[1];
    f->figure_name[2] = (uint8_t)name[2];
}

static void fill_line(interaction_figure_t *f, const char name[3],
                      uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                      hud_color_t color, uint16_t width)
{
    fill_name(f, name);
    f->operate_type = ADD;
    f->figure_type  = LINE;
    f->layer        = 0;
    f->color        = color;
    f->width        = width;
    f->start_x      = x0;
    f->start_y      = y0;
    f->details_d    = x1;
    f->details_e    = y1;
}

static void fill_rect(interaction_figure_t *f, const char name[3],
                      uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                      hud_color_t color, uint16_t width)
{
    fill_name(f, name);
    f->operate_type = ADD;
    f->figure_type  = RECTANGLE;
    f->layer        = 0;
    f->color        = color;
    f->width        = width;
    f->start_x      = x0;
    f->start_y      = y0;
    f->details_d    = x1;
    f->details_e    = y1;
}

static void fill_circle(interaction_figure_t *f, const char name[3],
                        uint16_t cx, uint16_t cy, uint16_t radius,
                        hud_color_t color, uint16_t width)
{
    fill_name(f, name);
    f->operate_type = ADD;
    f->figure_type  = CIRCLE;
    f->layer        = 0;
    f->color        = color;
    f->width        = width;
    f->start_x      = cx;
    f->start_y      = cy;
    f->details_c    = radius;
}
#endif

void build_hud_data(uint8_t *buf)
{
    robot_interaction_data_t data;
    memset(&data, 0, sizeof(data));

    data.data_cmd_id = UI_CMD_GRAPHIC_ONE;  // 0x0101 single-graphic sub-cmd
    data.sender_id   = robot_status.robot_id;
    data.receiver_id = robot_status.robot_id + 0x100;

    data.interaction_figure.figure_name[0] = 'r';
    data.interaction_figure.figure_name[1] = '0';
    data.interaction_figure.figure_name[2] = '0';
    data.interaction_figure.operate_type = ADD;
    data.interaction_figure.figure_type  = RECTANGLE;
    data.interaction_figure.layer        = 1;
    data.interaction_figure.color        = WHITE;
    data.interaction_figure.details_a    = 0;
    data.interaction_figure.details_b    = 0;
    data.interaction_figure.width        = 3;
    data.interaction_figure.start_x      = 1000;
    data.interaction_figure.start_y      = 500;
    data.interaction_figure.details_c    = 0;
    data.interaction_figure.details_d    = 1500;
    data.interaction_figure.details_e    = 700;

    memcpy(buf, &data, sizeof(data));
}

