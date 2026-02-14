/*
 * remote_control.c
 * This file reads channel data from FlySky Radio Receivers (FS-IA6B)
 * using normal mode DMA. It assumes the receiver's servo pins are connected
 * to the devboard's UART and the UART is configured as follows:
 *
 * Mode: Asynchronous
 * Baudrate: 100000
 * Format: 8E2 (9 bits including parity)
 * RX Inversion: Enabled
 */

#include "remote_control.h"
#include "message_center.h"
#include <string.h>

// Load buffer into a custom RAM section instead of default DTCMRAM, so that
// DMA can access it (required on STM32H7)
__attribute__((__section__(".dma_bss"))) static uint8_t buffer[RC_FRAME_LENGTH] = {0};

RC_ctrl_t rc_ctrl;
static volatile uint32_t rc_frame_count = 0;
volatile RC_sync_state_t RC_sync_state = RC_SYNC;

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

/* Public functions */

// Put this where you need frame count
uint32_t RC_GetFrameCount(void) {
  return rc_frame_count;
}

// Put this in your main.c initialization.
void remote_control_init() {
	HAL_UART_Receive_DMA(RC_UART, (uint8_t*) &buffer[0], 1);
}

// Put this where you need raw buffer data
void RC_GetLastFrame(uint8_t out[RC_FRAME_LENGTH]) {
    if (!out) return;
    memcpy(out, buffer, RC_FRAME_LENGTH);
}

// Put this where you need remote control data
const RC_ctrl_t *get_remote_control_point() {
	return &rc_ctrl;
}

// Put this in HAL_UART_RxCpltCallback
void REMOTE_RX_Complete_Handler(UART_HandleTypeDef *huart) {
	switch (RC_sync_state) {
		case RC_SYNC:
			if (buffer[0] == RC_HEADER) {
				RC_sync_state = RC_SYNCED;
				HAL_UART_Receive_DMA(RC_UART, (uint8_t*) &buffer[1], RC_FRAME_LENGTH - 1);
			} else {
				HAL_UART_Receive_DMA(RC_UART, (uint8_t*) &buffer[0], 1);
			}
			break;
		case RC_SYNCED:
			if (buffer[0] != RC_HEADER && buffer[24] != RC_FOOTER) {
				RC_sync_state = RC_SYNC;
				HAL_UART_Receive_DMA(RC_UART, (uint8_t*) &buffer[0], 1);
				break;
			}
			rc_frame_count++;
			sbus_to_rc(buffer, &rc_ctrl);

			/* Publish RC data to message center from ISR context */
			MsgCenter_PublishFromISR(TOPIC_RC_UPDATE, &rc_ctrl, sizeof(rc_ctrl));

			HAL_UART_Receive_DMA(RC_UART, (uint8_t*) &buffer[0], RC_FRAME_LENGTH);
			break;
	}
}

// Put this in HAL_UART_ErrorCallback
void REMOTE_Error_Handler(UART_HandleTypeDef *huart) {
	// Clear overrun error
    __HAL_UART_CLEAR_OREFLAG(RC_UART);
    // Clear error code
    RC_UART->ErrorCode = HAL_UART_ERROR_NONE;
	// Reset state machine
	RC_sync_state = RC_SYNC;
    
    // Abort and restart reception
    HAL_UART_AbortReceive(RC_UART);
    HAL_UART_Receive_DMA(RC_UART, (uint8_t*) &buffer, 1);
}

/* Helper Functions */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    int bit_index = 0;
    for (int ch = 0; ch < 10; ch++) {
        // Calculate the current byte and offset from channel number
        int byte_index = bit_index / 8 + 1;  // +1 to skip SOF
        int bit_offset = bit_index % 8;

        // Build 11-bit channel from 2 to 3 contiguous bytes
        uint16_t value = sbus_buf[byte_index] >> bit_offset;
        value |= ((uint16_t)sbus_buf[byte_index + 1]) << (8 - bit_offset);
        if (bit_offset > 5) {
            value |= ((uint16_t)sbus_buf[byte_index + 2]) << (16 - bit_offset);
        }

        // Mask the channel with 11 ones (equal to 0x07FF)
        value &= 0x07FF;

        // Set the RC_ctrl component based on the channel number
        if (0 <= ch && ch < 4) {
            rc_ctrl->rc.ch[ch] = value - RC_CH_VALUE_OFFSET;
        } else if (4 <= ch && ch < 8) {
            // Up is ~240, down is ~1808, mid is ~1024
            char switch_pos = 1;
            if (value < 800) {
                switch_pos = 1;
            } else if (1200 < value) {
                switch_pos = 2;
            } else if (800 < value && value < 1200) {
                switch_pos = 3;
            }
            rc_ctrl->rc.s[ch - 4] = switch_pos;
        } else if (8 <= ch && ch < 10) {
            rc_ctrl->rc.ch[ch - 4] = value - RC_CH_VALUE_OFFSET;
        }

        // Go to the next 11-bit channel
        bit_index += 11;
    }
}
