/**
 * ref_structs.h
 *
 * This provides the headers for ref_structs.c
 * Adapted from: https://github.com/RoboMaster/Development-Board-C-Examples
 */
#ifndef REF_DATA_H
#define REF_DATA_H

#include "ref_protocol.h"
#include <stdint.h>

typedef enum
{
    UNKNOWN         = 0,
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    RED_DART        = 8,
    RED_RADAR       = 9,
    RED_OUTPOST     = 10,
    RED_BASE        = 11,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
    BLUE_DART       = 108,
    BLUE_RADAR      = 109,
    BLUE_OUTPOST    = 110,
    BLUE_BASE       = 111,
} robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;

typedef enum
{
    NO_OPERATION    = 0,
    ADD             = 1,
    EDIT            = 2,
    DELETE          = 3
} hud_operation_t;

typedef enum
{
    LINE        = 0,
    RECTANGLE   = 1,
    CIRCLE      = 2,
    ELLIPSE     = 3,
    ARC         = 4,
    FLOAT       = 5,
    INTEGER     = 6,
    CHARACTER   = 7
} hud_shape_t;

typedef enum
{
    TEAM_COLOR  = 0,
    YELLOW      = 1,
    GREEN       = 2,
    ORANGE      = 3,
    MAGENTA     = 4,
    PINK        = 5,
    CYAN        = 6,
    BLACK       = 7,
    WHITE       = 8
} hud_color_t;

#pragma pack(push, 1)
typedef struct //0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t unix_time;
} game_state_t;

typedef struct //0002
{
    uint8_t winner;
} game_result_t;

typedef struct
{
    uint16_t hero_hp;
    uint16_t engineer_hp;
    uint16_t infantry_3_hp;
    uint16_t infantry_4_hp;
    uint16_t reserved;
    uint16_t sentry_hp;
    uint16_t outpost_hp;
    uint16_t base_hp;
} game_robot_HP_t;

typedef struct //0x0101
{
    uint32_t event_type;
} event_data_t;

typedef struct //0x0104
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

typedef struct //0x0105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;

typedef struct //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef struct //0x0202
{
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_heat_17mm;
    uint16_t shooter_heat_42mm;
} power_heat_data_t;

typedef struct //0x0203
{
    float x;
    float y;
    float angle;
} robot_pos_t;

typedef struct //0x0204
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defense_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} buff_t;

typedef struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} robot_hurt_t;

typedef struct //0x0207
{
    uint8_t projectile_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float projectile_speed;
} shoot_data_t;

typedef struct //0x0208
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

typedef struct //0x0209
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} rfid_status_t;

typedef struct //0x020A
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

typedef struct //0x020B
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float infantry_3_x;
    float infantry_3_y;
    float infantry_4_x;
    float infantry_4_y;
    float reserved1;
    float reserved2;
} ground_robot_position_t;

typedef struct //0x020C
{
    uint16_t tracking_progress;
} radar_mark_data_t;

typedef struct //0x020D
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} sentry_info_t;

typedef struct //0x020E
{
    uint8_t radar_info;
} radar_info_t;

typedef struct //0x0301
{
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} interaction_figure_t;

typedef struct //0x0301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    interaction_figure_t interaction_figure;
} robot_interaction_data_t;

#define UI_CMD_DELETE_LAYER  0x0100
#define UI_CMD_GRAPHIC_ONE   0x0101
#define UI_CMD_GRAPHIC_TWO   0x0102
#define UI_CMD_GRAPHIC_FIVE  0x0103
#define UI_CMD_GRAPHIC_SEVEN 0x0104
#define UI_CMD_CHARACTER     0x0110

typedef enum
{
    HUD_TEXT_SPIN      = 0,
    HUD_TEXT_CAP       = 1,
    HUD_TEXT_CLEAR_AIM = 2,
} hud_text_slot_t;

typedef struct // 0x0301 sub-cmd 0x0100
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

typedef struct // 0x0301 sub-cmd 0x0100 with interaction header
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    interaction_layer_delete_t layer_delete;
} client_delete_layer_t;

typedef struct // 0x0301 sub-cmd 0x0102
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    interaction_figure_t interaction_figure[2];
} client_custom_graphic_two_t;

typedef struct // 0x0301 sub-cmd 0x0104
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    interaction_figure_t interaction_figure[7];
} client_custom_graphic_seven_t;

typedef struct // 0x0301 sub-cmd 0x0110
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    interaction_figure_t interaction_figure;
    uint8_t data[30];
} client_custom_character_t;

typedef struct // 0x0302
{
    uint8_t *data;
} custom_robot_data_t;

typedef struct //0x0303
{
    float opponent_position_x;
    float opponent_position_y;
    uint8_t cmd_keyboard;
    uint8_t opponent_robot_id;
    uint16_t source_id;
} map_command_t;

typedef struct //0x0304
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} remote_control_t;

// TODO: update to 2026 protocol format
typedef struct //0x0305
{
    uint16_t hero_location_x;
    uint16_t hero_location_y;
    uint16_t engineer_location_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} map_robot_data_t;

typedef struct //0x0306
{
    uint16_t key_value;
    uint16_t x_location : 12;
    uint16_t mouse_left : 4;
    uint16_t y_location : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} custom_client_data_t;

typedef struct //0x0307
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} map_data_t;

typedef struct //0x0309
{
    uint8_t *data;
} robot_custom_data_1_t;

typedef struct //0x0310
{
    uint8_t *data;
} robot_custom_data_2_t;

#pragma pack(pop)

_Static_assert(sizeof(interaction_figure_t) == 15, "graphic_data_struct_t must be 15 bytes");
_Static_assert(sizeof(client_delete_layer_t) == 8, "delete-layer packet must be 8 bytes");
_Static_assert(sizeof(client_custom_graphic_two_t) == 36, "two-graphic packet must be 36 bytes");
_Static_assert(sizeof(client_custom_graphic_seven_t) == 111, "seven-graphic packet must be 111 bytes");
_Static_assert(sizeof(client_custom_character_t) == 51, "character packet must be 51 bytes");

extern void ref_structs_init(void);
extern void ref_structs_solve(uint8_t *frame);
extern void build_delete_all_for_robot(uint8_t *buf, uint8_t robot_id);
extern void build_hud_data(uint8_t *buf, hud_operation_t op);
extern void build_hud_data_for_robot(uint8_t *buf, hud_operation_t op, uint8_t robot_id);
extern void build_hud_text_for_robot(uint8_t *buf, hud_operation_t op, uint8_t robot_id, hud_text_slot_t slot);
// Debug: builds a single-graphic interaction packet (0x0101 sub-cmd, 21 bytes).
extern void build_test_circle_single(uint8_t *buf, hud_operation_t op);
extern void build_test_circle_single_for_robot(uint8_t *buf, hud_operation_t op, uint8_t robot_id);
extern uint8_t get_robot_id(void);
extern void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat);

extern void hud_state_set_supercap(float voltage_pct, uint8_t mode);
extern void hud_state_set_vision(uint8_t target_state, uint32_t tick_ms);
extern void hud_state_set_opstate(uint8_t spin_mode, uint8_t gimbal_follow, uint8_t aimbot_engaged);

#endif
