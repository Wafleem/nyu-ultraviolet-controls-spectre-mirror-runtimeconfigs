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
 * Note: this implementation requires your IBUS reads to keep up with the
 * transmit rate (approximately every 7ms), so don't put too many delays in
 * your main infinite loop.
 *
 * Adapted from https://github.com/mokhwasomssi/stm32_hal_ibus/blob/main/stm32f411_fw_ibus/Ibus/ibus.c
 */

#include "ibus.h"
#include <string.h>

RC_ctrl_t rc_ctrl;
static uint8_t buffer[RC_FRAME_LENGTH] = {0};
static volatile bool ibus_ready = false;

static int ibus_to_rc(volatile const uint8_t *ibus_buf, RC_ctrl_t *rc_ctrl);
static bool ibus_checksum(volatile const uint8_t *ibus_buf);

/* Public functions */

// Put this in your main.c initialization.
void remote_control_init() {
	HAL_UART_Receive_IT(RC_UART, buffer, RC_FRAME_LENGTH);
}

// Put this where you need to debug buffer data
void RC_GetBuffer(uint8_t out[RC_FRAME_LENGTH]) {
    if (!out) return;
    memcpy(out, buffer, RC_FRAME_LENGTH);
}

// Put this where you need remote control data
const RC_ctrl_t *get_remote_control_point() {
	return &rc_ctrl;
}

// Put this in HAL_UART_RxCpltCallback
void ibus_handle_complete(UART_HandleTypeDef *huart) {
	ibus_ready = true;
}

// Put this in HAL_UART_ErrorCallback
void ibus_handle_error(UART_HandleTypeDef *huart) {
	// Clear overrun error
    __HAL_UART_CLEAR_OREFLAG(RC_UART);
    // Clear error code
    RC_UART->ErrorCode = HAL_UART_ERROR_NONE;
    
    // Abort and restart reception
    HAL_UART_AbortReceive(RC_UART);
    HAL_UART_Receive_IT(RC_UART, buffer, RC_FRAME_LENGTH);
}

// Put this in your main infinite loop
int ibus_read() {
	// Check if IBUS frame is complete
	if (!ibus_ready) {
		return RC_NOT_READY;
	}
	ibus_ready = false;

	// Read IBUS data into provided buffer
	int status = ibus_to_rc(buffer, &rc_ctrl);
	HAL_UART_Receive_IT(RC_UART, buffer, RC_FRAME_LENGTH);
	return status;
}

/* Helper Functions */
static int ibus_to_rc(volatile const uint8_t *ibus_buf, RC_ctrl_t *rc_ctrl) {
	// Stop if header bytes are wrong
	if(ibus_buf[0] != RC_FRAME_LENGTH || ibus_buf[1] != RC_COMMAND40) {
		return RC_INVALID_HEADER;
	}

	// Stop if checksum is wrong
	if(!ibus_checksum(ibus_buf)) {
		return RC_INVALID_CHECKSUM;
	}

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
	return RC_OK;
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
