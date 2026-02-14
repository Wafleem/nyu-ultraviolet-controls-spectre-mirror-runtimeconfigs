/*
 * remote_control.h
 * This file contains all the function prototypes for the remote_control.c file.
 */

#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__


#include "usart.h"
#include <stdbool.h>


#define RC_UART 				(&huart7)
#define RC_HEADER               0x0F
#define RC_FOOTER               0x00

#define RC_FRAME_LENGTH			25u

#define RC_CH_VALUE_MIN         ((uint16_t)240)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1808)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __PACKED_STRUCT
{
        __PACKED_STRUCT
        {
                int16_t ch[6];
                char s[4];
        } rc;
        __PACKED_STRUCT
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __PACKED_STRUCT
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

typedef enum {
    RC_SYNC      = 0,
    RC_SYNCED    = 1
} RC_sync_state_t;

extern volatile RC_sync_state_t RC_sync_state;

/* ----------------------- Internal Data ----------------------------------- */

void remote_control_init(void);
const RC_ctrl_t *get_remote_control_point(void);
uint32_t RC_GetFrameCount(void);
void RC_GetLastFrame(uint8_t out[RC_FRAME_LENGTH]);

void REMOTE_RX_Complete_Handler(UART_HandleTypeDef *huart);
void REMOTE_Error_Handler(UART_HandleTypeDef *huart);


#endif /* __REMOTE_CONTROL_H__ */