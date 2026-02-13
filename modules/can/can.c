/*
 * can.c
 * CAN configuration and interrupt callback implementation for current test setup
 */

#include "can.h"
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

// Define the shared CAN variables
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
volatile uint8_t can_rx_flag = 0;
volatile uint32_t received_number = 0;

void CAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // Configure FDCAN2 filter to accept all standard ID messages into FIFO0
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Reject non-matching frames on FDCAN2
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    // RX interrupt on FDCAN2 for receiver
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    // Start CAN peripherals
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    // TX header for FDCAN1
    TxHeader.Identifier = 0x123;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
}

/*
 * NOTE: HAL_FDCAN_RxFifo0Callback has been moved to can_manager.c
 * which dispatches received frames through the CAN Manager system.
 * The old test-level callback that only handled FDCAN2 is no longer needed.
 */
/* EOF */

bool can_send_number(uint32_t sent_number)
{
    // Pack counter into TxData
    TxData[0] = (sent_number >> 24) & 0xFF;
    TxData[1] = (sent_number >> 16) & 0xFF;
    TxData[2] = (sent_number >> 8) & 0xFF;
    TxData[3] = sent_number & 0xFF;
    TxData[4] = 0x00; TxData[5] = 0x00; TxData[6] = 0x00; TxData[7] = 0x00;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) == HAL_OK) {
        return true;
    }
    return false;
}

bool can_check_received(uint32_t *out)
{
    if (out == NULL) return false;
    if (can_rx_flag) {
        *out = received_number;
        can_rx_flag = 0;
        return true;
    }
    return false;
}

static uint8_t dlc_to_bytes(uint32_t dlc)
{
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
#ifdef FDCAN_DLC_BYTES_12
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
#endif
        default: return 0;
    }
}

static uint32_t bytes_to_dlc(uint8_t len)
{
    switch (len) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

bool can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len > 8) return false;

    TxHeader.Identifier = id;
    TxHeader.DataLength = bytes_to_dlc(len);

    // copy payload (pad remaining bytes with 0)
    memset(TxData, 0, sizeof(TxData));
    memcpy(TxData, data, len);

    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) == HAL_OK);
}

bool can_receive(uint32_t *out_id, uint8_t *out_data, uint8_t max_len, uint8_t *out_len)
{
    if (out_id == NULL || out_data == NULL || out_len == NULL) return false;
    if (!can_rx_flag) return false;

    // Copy ID
    *out_id = RxHeader.Identifier;

    // Determine received byte count from DLC
    uint8_t n = dlc_to_bytes(RxHeader.DataLength);
    if (n > max_len) n = max_len;

    memcpy(out_data, RxData, n);
    *out_len = n;

    // clear pending
    can_rx_flag = 0;
    return true;
}

