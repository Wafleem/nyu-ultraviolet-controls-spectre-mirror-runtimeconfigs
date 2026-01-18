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
#define RC_COMMAND40			0x40	// Header byte
#define RC_NUM_CHANNELS 		6		// 4 stick + knob channels
#define RC_NUM_SWITCHES 		4		// 4 switches
#define RC_FRAME_LENGTH			0x20	// 32 bytes

#define RC_CH_VALUE_MIN         ((uint16_t)1000)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1500)
#define RC_CH_VALUE_MAX         ((uint16_t)2000)
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_DOWN              ((uint16_t)2)

#define RC_OK                 0
#define RC_NOT_READY          1
#define RC_INVALID_HEADER     2
#define RC_INVALID_CHECKSUM   3


/* Structs */
typedef __PACKED_STRUCT
{
    __PACKED_STRUCT
    {
        int16_t ch[6];
        char s[4];
    } rc;
} RC_ctrl_t;


/* Functions */
void remote_control_init();
void RC_GetBuffer(uint8_t out[RC_FRAME_LENGTH]);
const RC_ctrl_t *get_remote_control_point();
void ibus_handle_complete();
void ibus_handle_error();
int ibus_read();


#endif /* __IBUS_H__ */