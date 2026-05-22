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
#include "logger.h"
#include "string.h"
#include "stdio.h"

#include "main.h"

#define BASE_SIGHT_X              960u
#define BASE_SIGHT_Y              540u
#define SMM_AIM_X                 BASE_SIGHT_X
#define SMM_AIM_Y                 ((uint16_t)(BASE_SIGHT_Y - 55u))
#define AIM_HALF_LEN              34u
#define AIM_RADIUS                18u
#define AIM_WIDTH                 3u

#define STATUS_TEXT_X             1260u
#define SPIN_TEXT_Y               700u
#define CAP_TEXT_Y                635u

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

void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat) {
    if (heat_limit) *heat_limit = robot_status.shooter_barrel_heat_limit;
    if (heat)       *heat       = power_heat_data.shooter_heat_17mm;
}

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
            LOG_INFO(LOG_TAG_REF, "GAME progress=%u remain=%us",
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
        break;
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
    f->figure_type  = LINE;
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
    f->figure_type  = CIRCLE;
    f->layer        = 0;
    f->color        = color;
    f->width        = width;
    f->start_x      = cx;
    f->start_y      = cy;
    f->details_c    = radius;
}

static void fill_text(client_custom_character_t *data, const char name[3],
                      const char *text, uint16_t x, uint16_t y,
                      hud_color_t color, uint16_t font_size,
                      uint16_t line_width, hud_operation_t op)
{
    size_t len = strlen(text);
    if (len > sizeof(data->data)) {
        len = sizeof(data->data);
    }

    fill_name(&data->interaction_figure, name);
    data->interaction_figure.operate_type = op;
    data->interaction_figure.figure_type  = CHARACTER;
    data->interaction_figure.layer        = 0;
    data->interaction_figure.color        = color;
    data->interaction_figure.details_a    = font_size;
    data->interaction_figure.details_b    = (uint16_t)len;
    data->interaction_figure.width        = line_width;
    data->interaction_figure.start_x      = x;
    data->interaction_figure.start_y      = y;

    memcpy(data->data, text, len);
}

static hud_color_t cap_mode_color(void)
{
    switch (s_hud_state.sc_mode) {
        case 2: return CYAN;
        case 4: return GREEN;
        case 1: return YELLOW;
        case 3: return MAGENTA;
        default: return WHITE;
    }
}

static const char *cap_mode_text(void)
{
    switch (s_hud_state.sc_mode) {
        case 2: return "CAP: ON       ";
        case 4: return "CAP: READY    ";
        case 1: return "CAP: CHARGING ";
        case 3: return "CAP: LOW      ";
        default: return "CAP: UNKNOWN  ";
    }
}

void build_hud_data_for_robot(uint8_t *buf, hud_operation_t op, uint8_t robot_id)
{
    client_custom_graphic_seven_t data;
    memset(&data, 0, sizeof(data));

    data.data_cmd_id = UI_CMD_GRAPHIC_SEVEN;
    data.sender_id   = robot_id;
    data.receiver_id = (uint16_t)robot_id + 0x100;

    if (op == DELETE) {
        const char legacy_names[7][3] = {
            {'s', 'm', 'l'},
            {'s', 'm', 'r'},
            {'s', 'm', 't'},
            {'s', 'm', 'b'},
            {'s', 'p', 'n'},
            {'c', 'a', 'p'},
            {'c', 'f', 'l'},
        };
        for (int i = 0; i < 7; i++) {
            data.interaction_figure[i].operate_type = DELETE;
            fill_name(&data.interaction_figure[i], legacy_names[i]);
        }
    } else {
        fill_line(&data.interaction_figure[0], "aih",
                  SMM_AIM_X - AIM_HALF_LEN, SMM_AIM_Y,
                  SMM_AIM_X + AIM_HALF_LEN, SMM_AIM_Y,
                  WHITE, AIM_WIDTH);

        fill_line(&data.interaction_figure[1], "aiv",
                  SMM_AIM_X, SMM_AIM_Y - AIM_HALF_LEN,
                  SMM_AIM_X, SMM_AIM_Y + AIM_HALF_LEN,
                  WHITE, AIM_WIDTH);

        fill_circle(&data.interaction_figure[2], "air",
                    SMM_AIM_X, SMM_AIM_Y, AIM_RADIUS,
                    WHITE, 2);

        const char unused_names[4][3] = {
            {'s', 'm', 'l'},
            {'s', 'm', 'r'},
            {'s', 'm', 't'},
            {'s', 'm', 'b'},
        };
        for (int i = 3; i < 7; i++) {
            data.interaction_figure[i].operate_type = DELETE;
            fill_name(&data.interaction_figure[i], unused_names[i - 3]);
        }
        for (int i = 0; i < 3; i++) {
            data.interaction_figure[i].operate_type = op;
        }
    }

    // Log HUD state every call (rate limiting disabled for LOG_TAG_HUD)
    {
        uint16_t heat_limit = 0, heat = 0;
        get_shoot_heat_limit_and_heat(&heat_limit, &heat);
        LOG_DEBUG(LOG_TAG_HUD,
            "op=%s sc=%.1f%%(%u) aim=%u spn=%u fol=%u vis=%u heat=%u/%u",
            (op == ADD) ? "ADD" : "EDIT",
            (double)s_hud_state.sc_voltage_pct,
            (unsigned)s_hud_state.sc_mode,
            (unsigned)s_hud_state.aimbot_engaged,
            (unsigned)s_hud_state.spin_mode,
            (unsigned)s_hud_state.gimbal_follow,
            (unsigned)s_hud_state.vision_target_state,
            (unsigned)heat,
            (unsigned)heat_limit);
    }

    memcpy(buf, &data, sizeof(data));
}

void build_hud_text_for_robot(uint8_t *buf, hud_operation_t op, uint8_t robot_id, hud_text_slot_t slot)
{
    client_custom_character_t data;
    memset(&data, 0, sizeof(data));

    data.data_cmd_id = UI_CMD_CHARACTER;
    data.sender_id   = robot_id;
    data.receiver_id = (uint16_t)robot_id + 0x100;

    switch (slot) {
        case HUD_TEXT_SPIN:
            fill_text(&data, "spt",
                      s_hud_state.spin_mode ? "SPIN: ON " : "SPIN: OFF",
                      STATUS_TEXT_X, SPIN_TEXT_Y,
                      s_hud_state.spin_mode ? CYAN : WHITE, 22u, 2u, op);
            break;
        case HUD_TEXT_CAP:
        default:
            fill_text(&data, "cpt", cap_mode_text(),
                      STATUS_TEXT_X, CAP_TEXT_Y,
                      cap_mode_color(), 22u, 2u, op);
            break;
        case HUD_TEXT_CLEAR_AIM:
            fill_text(&data, "smm", "", STATUS_TEXT_X, CAP_TEXT_Y, WHITE, 1u, 1u, DELETE);
            break;
    }

    memcpy(buf, &data, sizeof(data));
}

void build_hud_data(uint8_t *buf, hud_operation_t op)
{
    build_hud_data_for_robot(buf, op, robot_status.robot_id);
}

void build_delete_all_for_robot(uint8_t *buf, uint8_t robot_id)
{
    client_delete_layer_t data;
    memset(&data, 0, sizeof(data));

    data.data_cmd_id = UI_CMD_DELETE_LAYER;
    data.sender_id   = robot_id;
    data.receiver_id = (uint16_t)robot_id + 0x100;
    data.layer_delete.delete_type = 2;
    data.layer_delete.layer = 0;

    memcpy(buf, &data, sizeof(data));
}

// Debug helper: single-graphic packet (sub-cmd 0x0101, 21 bytes payload).
// Draws one obvious white rectangle in the central screen region.

void build_test_circle_single_for_robot(uint8_t *buf, hud_operation_t op, uint8_t robot_id)
{
    robot_interaction_data_t data;
    memset(&data, 0, sizeof(data));

    data.data_cmd_id = UI_CMD_GRAPHIC_ONE;
    data.sender_id   = robot_id;
    data.receiver_id = (uint16_t)robot_id + 0x100;

    data.interaction_figure.figure_name[0] = 'r';
    data.interaction_figure.figure_name[1] = '0';
    data.interaction_figure.figure_name[2] = '0';
    data.interaction_figure.operate_type   = op;
    data.interaction_figure.figure_type    = RECTANGLE;
    data.interaction_figure.layer          = 1;
    data.interaction_figure.color          = WHITE;
    data.interaction_figure.details_a      = 0;
    data.interaction_figure.details_b      = 0;
    data.interaction_figure.width          = 3;
    data.interaction_figure.start_x        = 1000;
    data.interaction_figure.start_y        = 500;
    data.interaction_figure.details_c      = 0;
    data.interaction_figure.details_d      = 1500;
    data.interaction_figure.details_e      = 700;

    memcpy(buf, &data, sizeof(data));
}

void build_test_circle_single(uint8_t *buf, hud_operation_t op)
{
    build_test_circle_single_for_robot(buf, op, robot_status.robot_id);
}
