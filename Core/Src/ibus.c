/*
 * ibus.c
 * This file reads IBUS channel data from FlySky Radio Receivers (FS-IA6B),
 * assuming the receiver's IBUS servo pin is connected to the devboard's UART RX.
 *
 * This uses an interrupt-based implementation.
 *
 * Adapted from https://github.com/mokhwasomssi/stm32_hal_ibus/blob/main/stm32f411_fw_ibus/Ibus/ibus.c
 */

#include "ibus.h"


static uint8_t uart_rx_buffer[IBUS_LENGTH] = {0};
static uint8_t buffer[IBUS_BUFFER_LENGTH];
static uint8_t fail_safe_flag = 0;
volatile bool ibus_ready = false;

void ibus_init() {
	HAL_UART_Receive_IT(IBUS_UART, uart_rx_buffer, IBUS_LENGTH);
}

// If the current ibus frame is valid, read it into ibus_data
int ibus_read(uint16_t* ibus_data) {
	// Handle UART error
	if (IBUS_UART->ErrorCode != HAL_UART_ERROR_NONE) {
        __HAL_UART_CLEAR_OREFLAG(&huart7);
        IBUS_UART->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_AbortReceive(&huart7);
        HAL_UART_Receive_IT(&huart7, uart_rx_buffer, IBUS_LENGTH);
        return 3;
    }

	// Check if IBUS frame is complete
	if (!ibus_ready) {
		return 2;
	}
	ibus_ready = false;

	// Check if header and checksums are valid
	if(!ibus_is_valid() || !ibus_checksum()) {
		HAL_UART_Receive_IT(IBUS_UART, uart_rx_buffer, IBUS_LENGTH);
		return 1;
	}

	// Read IBUS data into provided buffer
	ibus_update(ibus_data);
	HAL_UART_Receive_IT(IBUS_UART, uart_rx_buffer, IBUS_LENGTH);
	return 0;
}

// Check if length and command byte are valid
bool ibus_is_valid() {
	return (uart_rx_buffer[0] == IBUS_LENGTH && uart_rx_buffer[1] == IBUS_COMMAND40);
}

// Check if read and calculated checksums match
bool ibus_checksum() {
 	uint16_t checksum_cal = 0xffff;
	uint16_t checksum_ibus;

	for(int i = 0; i < 30; i++)
	{
		checksum_cal -= uart_rx_buffer[i];
	}

	checksum_ibus = uart_rx_buffer[31] << 8 | uart_rx_buffer[30]; // checksum value from ibus
	return (checksum_ibus == checksum_cal);
}

// Read every 2 bytes into each slot in ibus_data
void ibus_update(uint16_t* ibus_data) {
	for(int ch_index = 0, bf_index = 2; ch_index < IBUS_USER_CHANNELS; ch_index++, bf_index += 2)
	{
		ibus_data[ch_index] = uart_rx_buffer[bf_index + 1] << 8 | uart_rx_buffer[bf_index];
	}
}

// FS-A8S don't have fail safe feature, So make software fail-safe.
void ibus_soft_failsafe(uint16_t* ibus_data, uint8_t fail_safe_max) {	
	fail_safe_flag++;

	if(fail_safe_max > fail_safe_flag)
		return;

	// Clear ibus data
	for(int i = 0; i < IBUS_USER_CHANNELS; i++)
		ibus_data[i] = 0;

	// Clear ibus buffer
	for(int j = 0; j < IBUS_LENGTH; j++)
		uart_rx_buffer[j] = 0;

	fail_safe_flag = 0;
	return;
}

void ibus_reset_failsafe() {
	fail_safe_flag = 0; // flag reset
}

void ibus_set_ready() {
	ibus_ready = true;
}

uint8_t* ibus_get_buffer() {
    return uart_rx_buffer;
}

