/*
 * ibus.h
 * This file contains all the function prototypes for the ibus.c file.
 * Adapted from https://github.com/mokhwasomssi/stm32_hal_ibus/blob/main/stm32f411_fw_ibus/Ibus/ibus.h
 */

#ifndef __IBUS_H__
#define __IBUS_H__


#include "usart.h"
#include <stdbool.h>


/* Defines */
#define RC_UART 				(&huart7)
#define RC_FRAME_LENGTH			0x20	// 32 bytes
#define RC_COMMAND40			0x40	// Header byte
#define RC_NUM_CHANNELS 		6		// 4 stick + knob channels
#define RC_NUM_SWITCHES 		4		// 4 switches

#define RC_CH_VALUE_MIN         ((uint16_t)1000)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1500)
#define RC_CH_VALUE_MAX         ((uint16_t)2000)
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_DOWN              ((uint16_t)2)


/* Structs */
typedef __PACKED_STRUCT
{
    __PACKED_STRUCT
    {
        int16_t ch[6];
        char s[4];
    } rc;
} RC_ctrl_t;

typedef enum {
    RC_SYNC0     = 0,
    RC_SYNC1     = 1,
    RC_SYNCED    = 2
} RC_sync_state_t;

extern volatile RC_sync_state_t RC_sync_state;


/* Functions */
uint32_t RC_GetFrameCount();
void remote_control_init();
void RC_GetLastFrame(uint8_t out[RC_FRAME_LENGTH]);
const RC_ctrl_t *get_remote_control_point();
void REMOTE_RX_Complete_Handler();
void REMOTE_UART_Error_Handler();


#endif /* __IBUS_H__ */