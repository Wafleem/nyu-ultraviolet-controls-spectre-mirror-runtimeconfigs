/*
 * ibus.c
 * This file reads IBUS channel data from FlySky Radio Receivers (FS-IA6B)
 * using UART interrupts. It replaces bsp_rc.c and remote_control.c from the
 * DT7 code. It assumes the receiver's IBUS servo pins are connected to the
 * devboard's UART and the UART is configured as follows:
 *
 * Mode: Asynchronous
 * Baudrate: 115200
 * Format: 8N1
 *
 * Adapted from https://github.com/mokhwasomssi/stm32_hal_ibus/blob/main/stm32f411_fw_ibus/Ibus/ibus.c
 */

#include "ibus.h"
#include <string.h>

RC_ctrl_t rc_ctrl;
static uint8_t buffer[RC_FRAME_LENGTH] = {0};
static volatile uint32_t rc_frame_count = 0;
volatile RC_sync_state_t RC_sync_state = RC_SYNC0;

static void ibus_to_rc(volatile const uint8_t *ibus_buf, RC_ctrl_t *rc_ctrl);
static bool ibus_checksum(volatile const uint8_t *ibus_buf);

/* Public functions */

// Put this where you need IBUS frame count
uint32_t RC_GetFrameCount(void) {
  return rc_frame_count;
}

// Put this in your main.c initialization.
void remote_control_init() {
	HAL_UART_Receive_IT(RC_UART, (uint8_t*) &buffer[0], 1);
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
		case RC_SYNC0:
			if (buffer[0] == RC_FRAME_LENGTH) {
				RC_sync_state = RC_SYNC1;
				HAL_UART_Receive_IT(RC_UART, (uint8_t*) &buffer[1], 1);
			} else {
				HAL_UART_Receive_IT(RC_UART, (uint8_t*) &buffer[0], 1);
			}
			break;
		case RC_SYNC1:
			if (buffer[1] == RC_COMMAND40) {
				RC_sync_state = RC_SYNCED;
				HAL_UART_Receive_IT(RC_UART, (uint8_t*) &buffer[2], RC_FRAME_LENGTH - 2);
			} else {
				RC_sync_state = RC_SYNC0;
				HAL_UART_Receive_IT(RC_UART, (uint8_t*) &buffer[0], 1);
			}
			break;
		case RC_SYNCED:
			if (buffer[0] != RC_FRAME_LENGTH || buffer[1] != RC_COMMAND40 || !ibus_checksum(buffer)) {
				RC_sync_state = RC_SYNC0;
				HAL_UART_Receive_IT(RC_UART, (uint8_t*) &buffer[0], 1);
				break;
			}
			rc_frame_count++;
			ibus_to_rc(buffer, &rc_ctrl);
			HAL_UART_Receive_IT(RC_UART, (uint8_t*) &buffer[0], RC_FRAME_LENGTH);
			break;
	}
}

// Put this in HAL_UART_ErrorCallback
void REMOTE_UART_Error_Handler(UART_HandleTypeDef *huart) {
	// Clear overrun error
    __HAL_UART_CLEAR_OREFLAG(RC_UART);
    // Clear error code
    RC_UART->ErrorCode = HAL_UART_ERROR_NONE;
	// Reset state machine
	RC_sync_state = RC_SYNC0;
    
    // Abort and restart reception
    HAL_UART_AbortReceive(RC_UART);
    HAL_UART_Receive_IT(RC_UART, (uint8_t*) &buffer, 1);
}

/* Helper Functions */
static void ibus_to_rc(volatile const uint8_t *ibus_buf, RC_ctrl_t *rc_ctrl) {
	// Joy-stick channels: right horizontal (0), right vertical (1), left vertical (2), left horizontal (3)
	int bf_index = 2;
	for(int ch_index = 0; ch_index < 4; ch_index++) {
		uint16_t raw_channel = ibus_buf[bf_index + 1] << 8 | ibus_buf[bf_index];
		// Shift range to be around 0, just like the DT7 code
		rc_ctrl->rc.ch[ch_index] = raw_channel - RC_CH_VALUE_OFFSET;
		bf_index += 2;
	}

	// Switches: far left (0), middle left (1), middle right (2), far right (3)
	for(int sw_index = 0; sw_index < RC_NUM_SWITCHES; sw_index++) {
		uint16_t raw_channel = ibus_buf[bf_index + 1] << 8 | ibus_buf[bf_index];
		// Change switch range to be 1 or 2, just like the DT7 code
		rc_ctrl->rc.s[sw_index] = (raw_channel < 1500) ? 1 : 2;
		bf_index += 2;
	}
	
	// Knob channels: left (4), right (5)
	for(int ch_index = 4; ch_index < RC_NUM_CHANNELS; ch_index++) {
		uint16_t raw_channel = ibus_buf[bf_index + 1] << 8 | ibus_buf[bf_index];
		// Shift range to be around 0, just like the DT7 code
		rc_ctrl->rc.ch[ch_index] = raw_channel - RC_CH_VALUE_OFFSET;
		bf_index += 2;
	}
}

static bool ibus_checksum(volatile const uint8_t *ibus_buf) {
 	uint16_t checksum_cal = 0xffff;
	uint16_t checksum_ibus;

	for(int i = 0; i < 30; i++) {
		checksum_cal -= ibus_buf[i];
	}

	checksum_ibus = ibus_buf[31] << 8 | ibus_buf[30];
	return (checksum_ibus == checksum_cal);
}
