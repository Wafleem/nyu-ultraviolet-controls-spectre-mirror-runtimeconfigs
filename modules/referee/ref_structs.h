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
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t *user_data;
} robot_interaction_data_t;

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

extern void ref_structs_init(void);
extern void ref_structs_solve(uint8_t *frame);
extern uint8_t get_robot_id(void);
extern void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat);

#endif
