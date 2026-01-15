/*
 * ibus.h
 * This file contains all the function prototypes for the ibus.c file.
 * Adapted from https://github.com/mokhwasomssi/stm32_hal_ibus/blob/main/stm32f411_fw_ibus/Ibus/ibus.h
 */

#ifndef __IBUS_H__
#define __IBUS_H__


#include "usart.h"              // header from stm32cubemx code generate
#include <stdbool.h>


/* Defines */
#define IBUS_UART				(&huart7)
#define IBUS_USER_CHANNELS		10		// Use 10 channels
#define IBUS_BUFFER_LENGTH      64      // 64 bytes
#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Header byte
#define IBUS_MAX_CHANNLES		14


void ibus_init();
int ibus_read(uint16_t* ibus_data);
bool ibus_is_valid();
bool ibus_checksum();
void ibus_update(uint16_t* ibus_data);
void ibus_soft_failsafe(uint16_t* ibus_data, uint8_t fail_safe_max);
void ibus_reset_failsafe();
void ibus_set_ready();
uint8_t* ibus_get_buffer();

#endif /* __IBUS_H__ */