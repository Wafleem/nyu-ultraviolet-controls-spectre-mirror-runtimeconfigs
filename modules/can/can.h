/*
 * can.h
 * CAN module public interface and shared variables
 */
#ifndef CORE_INC_CAN_H
#define CORE_INC_CAN_H

#include <stdint.h>
#include "fdcan.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Public CAN variables (defined in can.c)
extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern volatile uint8_t can_rx_flag;
extern volatile uint32_t received_number;

// Initialize and configure CAN peripherals
void CAN_Config(void);

// Send a 32-bit counter as an 8-byte CAN frame (packed into bytes 0..3)
// Returns true if the HAL transmit request was queued successfully.
bool can_send_number(uint32_t sent_number);

// Check whether a CAN message was received (clears the internal flag).
// If true, writes the received 32-bit value to *out and returns true.
bool can_check_received(uint32_t *out);

// Generic CAN send: send `len` bytes from `data` with `id` (standard ID by default).
// Returns true if the message was queued for transmit.
bool can_send(uint32_t id, const uint8_t *data, uint8_t len);

// Generic CAN receive: if a frame is pending, copies `id` into *out_id, up to
// `max_len` bytes into `out_data`, stores received length in *out_len, clears
// the pending flag and returns true. Otherwise returns false.
bool can_receive(uint32_t *out_id, uint8_t *out_data, uint8_t max_len, uint8_t *out_len);

#ifdef __cplusplus
}
#endif

#endif // CORE_INC_CAN_H

