/**
 * protocol.h
 *
 * Provides constants related to referee system serial protocol.
 * Adapted from: https://github.com/RoboMaster/Development-Board-C-Examples
 */
#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include <stdint.h>

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         150
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + REF_PROTOCOL_CMD_SIZE)
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CMD_SIZE)

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    DART_INFO_CMD_ID                  = 0x0105,
    ROBOT_STATE_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_CMD_ID                       = 0x0204,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    PROJECTILE_ALLOWANCE_CMD_ID       = 0x0208,
    RFID_STATE_CMD_ID                 = 0x0209,
    DART_CLIENT_CMD_ID                = 0x020A,
    GROUND_POSITION_CMD_ID            = 0x020B,
    RADAR_MARK_CMD_ID                 = 0x020C,
    SENTRY_INFO_CMD_ID                = 0x020D,
    RADAR_INFO_CMD_ID                 = 0x020E,
    ROBOT_INTERACTION_DATA_CMD_ID     = 0x0301,
    CUSTOM_ROBOT_DATA_CMD_ID          = 0x0302,
    MAP_COMMAND_CMD_ID                = 0x0303,
    REMOTE_CONTROL_CMD_ID             = 0x0304,
    MAP_ROBOT_CMD_ID                  = 0x0305,
    CUSTOM_CLIENT_CMD_ID              = 0x0306,
    MAP_DATA_CMD_ID                   = 0x0307,
    CUSTOM_INFO_CMD_ID                = 0x0308,
    ROBOT_CUSTOM_DATA_1_CMD_ID        = 0x0309,
    ROBOT_CUSTOM_DATA_2_CMD_ID        = 0x0310,
    IDCustomData,
} referee_cmd_id_t;

typedef struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
