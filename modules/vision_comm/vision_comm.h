#ifndef VISION_COMM_H
#define VISION_COMM_H

#include "main.h"
#include "usart.h"
#include <stdint.h>

// Vision communication UART handle
#define VISION_UART_HANDLE huart5

#define VISION_RECV_SIZE 18u
#define VISION_SEND_SIZE 36u

#pragma pack(1)

// Fire mode
typedef enum
{
    NO_FIRE = 0,
    AUTO_FIRE = 1,
    AUTO_AIM = 2
} Fire_Mode_e;

// Target state
typedef enum
{
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e;

// Target type
typedef enum
{
    NO_TARGET_NUM = 0,
    HERO1 = 1,
    ENGINEER2 = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    OUTPOST = 6,
    SENTRY = 7,
    BASE = 8
} Target_Type_e;

// Vision receive data structure
typedef struct
{
    Fire_Mode_e fire_mode;
    Target_State_e target_state;
    Target_Type_e target_type;
    float pitch;
    float yaw;
    uint8_t updated;
} Vision_Recv_s;

// Enemy color
typedef enum
{
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED = 2,
} Enemy_Color_e;

// Work mode
typedef enum
{
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

// Bullet speed
typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

// Vision send data structure
typedef struct
{
    Enemy_Color_e enemy_color;
    Work_Mode_e work_mode;
    Bullet_Speed_e bullet_speed;
    float yaw;
    float pitch;
    float roll;
} Vision_Send_s;

#pragma pack()

/**
 * @brief Initialize vision communication module (using USART6)
 * @return Pointer to receive data structure
 */
Vision_Recv_s *VisionComm_Init(void);

/**
 * @brief Send vision data
 */
void VisionComm_Send(void);

/**
 * @brief Set vision send flags
 * @param enemy_color Enemy color
 * @param work_mode Work mode
 * @param bullet_speed Bullet speed
 */
void VisionComm_SetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief Set attitude data for sending
 * @param yaw Yaw angle
 * @param pitch Pitch angle
 * @param roll Roll angle
 */
void VisionComm_SetAltitude(float yaw, float pitch, float roll);

/**
 * @brief Get vision receive data
 * @return Pointer to receive data structure
 */
Vision_Recv_s *VisionComm_GetData(void);

/**
 * @brief UART receive callback function (called in UART interrupt)
 * @param buf Receive buffer
 * @param len Receive data length
 */
void VisionComm_RxCallback(uint8_t *buf, uint32_t len);

/**
 * @brief Start UART reception for vision communication
 */
void VisionComm_StartReceive(void);

#endif // VISION_COMM_H

