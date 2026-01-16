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

RC_ctrl_t rc_ctrl;
static uint8_t buffer[IBUS_LENGTH] = {0};
static volatile bool ibus_ready = false;

/* Helper functions */

static bool ibus_checksum(uint8_t* frame) {
 	uint16_t checksum_cal = 0xffff;
	uint16_t checksum_ibus;

	for(int i = 0; i < 30; i++) {
		checksum_cal -= frame[i];
	}

	checksum_ibus = frame[31] << 8 | frame[30];
	return (checksum_ibus == checksum_cal);
}

static int ibus_process_frame(uint8_t* frame) {
	// Stop if header bytes are wrong
	if(frame[0] != IBUS_LENGTH || frame[1] != IBUS_COMMAND40) {
		return IBUS_INVALID_HEADER;
	}

	// Stop if checksum is wrong
	if(!ibus_checksum(frame)) {
		return IBUS_INVALID_CHECKSUM;
	}

	// Joy-stick channels: right horizontal (0), right vertical (1), left vertical (2), left horizontal (3)
	int bf_index = 2;
	for(int ch_index = 0; ch_index < 4; ch_index++) {
		uint16_t raw_channel = frame[bf_index + 1] << 8 | frame[bf_index];
		// Shift range to be around 0, just like the DT7 code
		rc_ctrl.rc.ch[ch_index] = raw_channel - RC_CH_VALUE_OFFSET;
		bf_index += 2;
	}

	// Switches: far left (0), middle left (1), middle right (2), far right (3)
	for(int sw_index = 0; sw_index < IBUS_NUM_SWITCHES; sw_index++) {
		uint16_t raw_channel = frame[bf_index + 1] << 8 | frame[bf_index];
		// Change switch range to be 1 or 2, just like the DT7 code
		rc_ctrl.rc.s[sw_index] = (raw_channel < 1500) ? 1 : 2;
		bf_index += 2;
	}
	
	// Knob channels: left (4), right (5)
	for(int ch_index = 4; ch_index < IBUS_NUM_CHANNELS; ch_index++) {
		uint16_t raw_channel = frame[bf_index + 1] << 8 | frame[bf_index];
		// Shift range to be around 0, just like the DT7 code
		rc_ctrl.rc.ch[ch_index] = raw_channel - RC_CH_VALUE_OFFSET;
		bf_index += 2;
	}
	return IBUS_OK;
}

/* Public functions */

// Put this in your main.c initialization.
void ibus_init() {
	HAL_UART_Receive_IT(IBUS_UART, buffer, IBUS_LENGTH);
}

// Put this in HAL_UART_RxCpltCallback
void ibus_handle_complete(UART_HandleTypeDef *huart) {
	ibus_ready = true;
}

// Put this in HAL_UART_ErrorCallback
void ibus_handle_error(UART_HandleTypeDef *huart) {
	// Clear overrun error
    __HAL_UART_CLEAR_OREFLAG(IBUS_UART);
    // Clear error code
    IBUS_UART->ErrorCode = HAL_UART_ERROR_NONE;
    
    // Abort and restart reception
    HAL_UART_AbortReceive(IBUS_UART);
    HAL_UART_Receive_IT(IBUS_UART, ibus_get_buffer(), IBUS_LENGTH);
}

// Put this in your main infinite loop
int ibus_read() {
	// Check if IBUS frame is complete
	if (!ibus_ready) {
		return IBUS_NOT_READY;
	}
	ibus_ready = false;

	// Read IBUS data into provided buffer
	int status = ibus_process_frame(buffer);
	HAL_UART_Receive_IT(IBUS_UART, buffer, IBUS_LENGTH);
	return status;
}

// Put this where you need remote control data
const RC_ctrl_t *get_remote_control_point() {
	return &rc_ctrl;
}

// Put this where you need to debug buffer data
uint8_t* ibus_get_buffer() {
    return buffer;
}
