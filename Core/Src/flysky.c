/*
 * flysky.c
 * This file reads iBus channel data from FlySky Radio Receivers (FS-IA6B),
 * assuming the receiver's iBus servo pin is connected to the devboard's UART RX.
 * 
 * Modified to use standard asynchronous UART (RX-only) instead of half-duplex in order to use with Spectre2 board.
 *
 * Adapted from https://github.com/AtaberkOKLU/STM32-FlySky-IBus
 */

#include "main.h"
#include "flysky.h"

// Receiver Variables
FlyskyServoStruct ServoList;
volatile uint8_t Transiever_TX_Buffer[SERVO_BUFFER_SIZE];
volatile struct __FLAGS FLAGS;


void Servo_UART_Flysky_Init(UART_HandleTypeDef *huart) {
    // Register UART callbacks to receive Flysky servo frames
    HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, Servo_UART_RxComplete_Callback);
    HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, Servo_UART_Error_Callback);
    
    // No half-duplex needed we're only receiving
    // HAL_HalfDuplex_EnableReceiver(huart);
    
    FLAGS.FLYSKY_SYNC_STATES = FLYSKY_SYNC_SYNC0;
    HAL_UART_Receive_DMA(huart, (uint8_t*) &Transiever_TX_Buffer[0], 1);
}


void Servo_UART_Error_Callback(UART_HandleTypeDef *huart){
    // Clear all error flags
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    HAL_UART_DMAStop(huart);

    FLAGS.FLYSKY_SYNC_STATES = FLYSKY_SYNC_SYNC0;

    // Restart DMA - non-blocking (if it fails, we'll retry on next error)
    HAL_UART_Receive_DMA(huart, (uint8_t*) &Transiever_TX_Buffer[0], 1);
}


void Servo_UART_RxComplete_Callback(UART_HandleTypeDef *huart){
    uint16_t t, chksum = 0;

    switch (FLAGS.FLYSKY_SYNC_STATES) {
        // Check if the block's first header byte is valid
        case FLYSKY_SYNC_SYNC0:
            if(Transiever_TX_Buffer[0] == SERVO_HEADER_1) {
                FLAGS.FLYSKY_SYNC_STATES = FLYSKY_SYNC_SYNC1;
            }
            HAL_UART_Receive_DMA(huart, (uint8_t *) &Transiever_TX_Buffer[1], 1);
            break;
        // Check if the block's second header byte is valid
        case FLYSKY_SYNC_SYNC1:
            if(Transiever_TX_Buffer[1] == SERVO_HEADER_2) {
                FLAGS.FLYSKY_SYNC_STATES = FLYSKY_SYNC_SYNCED;
                HAL_UART_Receive_DMA(huart, (uint8_t *) &Transiever_TX_Buffer[2], SERVO_BUFFER_SIZE-2);
            } else {
                FLAGS.FLYSKY_SYNC_STATES = FLYSKY_SYNC_SYNC0;
                HAL_UART_Receive_DMA(huart, (uint8_t *) &Transiever_TX_Buffer[0], 1);
            }
            break;
        // Check if header bytes are still valid on subsequent byte blocks
        case FLYSKY_SYNC_VERIFIED:
            if((Transiever_TX_Buffer[0] != SERVO_HEADER_1) || (Transiever_TX_Buffer[1] != SERVO_HEADER_2)) {
                FLAGS.FLYSKY_SYNC_STATES = FLYSKY_SYNC_SYNC0;
                HAL_UART_Receive_DMA(huart, (uint8_t *) &Transiever_TX_Buffer[0], 1);
                break;
            }
            // Fall through to SYNCED case
            __attribute__((fallthrough));
        case FLYSKY_SYNC_SYNCED:
            // Calculate checksums
            t = 0;
            for(uint8_t i = 0; i < SERVO_BUFFER_SIZE-2; i++)
                t += Transiever_TX_Buffer[i];
            t = TELM_CHECKSUM_CONST-t;
            chksum = (Transiever_TX_Buffer[31]<<8 | Transiever_TX_Buffer[30]);

            // If checksums match, read the rest of the block into a struct
            if(t == chksum) {
                memcpy((void*)&ServoList, (uint8_t *) Transiever_TX_Buffer, SERVO_BUFFER_SIZE);
                FLAGS.FAIL_SAFE = (ServoList.Channel_11 > 1975);
                FLAGS.FLYSKY_SYNC_STATES = FLYSKY_SYNC_VERIFIED;
                HAL_UART_Receive_DMA(huart, (uint8_t *) &Transiever_TX_Buffer[0], SERVO_BUFFER_SIZE);
            // If checksums don't match, re-sync the byte block
            } else {
                FLAGS.FLYSKY_SYNC_STATES = FLYSKY_SYNC_SYNC0;
                HAL_UART_Receive_DMA(huart, (uint8_t *) &Transiever_TX_Buffer[0], 1);
            }
            break;
    }
}