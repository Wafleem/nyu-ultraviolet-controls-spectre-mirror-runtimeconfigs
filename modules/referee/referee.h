#ifndef REFEREE_H
#define REFEREE_H

#include "usart.h"
#include "ref_protocol.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

#define REFEREE_UART_HANDLE (&huart1)
#define REFEREE_USART USART1
#define REFEREE_DMA_RX hdma_usart1_rx
#define REFEREE_RX_BUF_LENGTH 512
#define REFEREE_FIFO_BUF_LENGTH 1024
#define REFEREE_TX_BUF_LENGTH 127

extern void get_referee_buffer(uint8_t out[REFEREE_RX_BUF_LENGTH]);
extern void referee_init(void);
extern void referee_unpack_fifo_data(void);
extern void referee_send_data(void);
extern void referee_IDLE_Handler(UART_HandleTypeDef *huart, uint16_t size);
void referee_Error_Handler(UART_HandleTypeDef *huart);
int get_temp();

#endif
