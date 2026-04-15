#include "referee.h"
#include "ref_structs.h"
#include "ref_crc.h"
#include "fifo.h"

uint8_t referee_rx_buf[REFEREE_RX_BUF_LENGTH] DMA_SECTION;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
uint8_t referee_tx_buf[REFEREE_TX_BUF_LENGTH];
fifo_s_t referee_fifo;
unpack_data_t referee_unpack_obj;
robot_interaction_data_t hud_data;
uint8_t referee_send_seq;

// Get the referee buffer for debugging purposes
void get_referee_buffer(uint8_t out[REFEREE_RX_BUF_LENGTH]) {
    if (!out) return;
    memcpy(out, referee_rx_buf, REFEREE_RX_BUF_LENGTH);
}

// Parse referee frames from the referee FIFO
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  // Check if there is any new referee data
  while ( fifo_s_used(&referee_fifo) )
  {
    // Grab the first byte from the referee FIFO
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      // If this byte starts a new referee frame, go to the next step
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      // Start reading the frame length and go next
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      // Finish reading the frame length. If it is valid, go next
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;

      // Read the frame sequence number and go next
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      // Read the CRC8 checksum. If it is valid, go next
      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      // Read the CRC16 checksum. If it is valid, read the frame data
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            ref_structs_solve(p_obj->protocol_packet);
          }
        }
      }break;

      // If current step is unknown, reset the state machine
      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

void referee_send_data(void)
{
  uint16_t data_len = sizeof(robot_interaction_data_t);
  uint16_t cmd_id = 0x0301;

  referee_tx_buf[0] = 0xA5;
  referee_tx_buf[1] = data_len & 0xFF;
  referee_tx_buf[2] = (data_len >> 8) & 0xFF;
  referee_tx_buf[3] = referee_send_seq;
  append_CRC8_check_sum(referee_tx_buf, 5);
  referee_tx_buf[5] = cmd_id & 0xFF;
  referee_tx_buf[6] = (cmd_id >> 8) & 0xFF;
  build_hud_data(&hud_data);
  memcpy(&hud_data, referee_tx_buf + 7, data_len);
  append_CRC16_check_sum(referee_tx_buf, 7 + data_len + 2);

  HAL_UART_Transmit(REFEREE_UART_HANDLE, referee_tx_buf, REFEREE_TX_BUF_LENGTH, HAL_MAX_DELAY);
  referee_send_seq++;
}

// Initialize referee interpreter
void referee_init(void)
{
  // Initialize referee structs
  ref_structs_init();
  memset(&hud_data, 0, sizeof(robot_interaction_data_t));
  referee_send_seq = 0;

  // Initialize FIFO
  fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);

  // Start DMA reception with IDLE detection
  HAL_UARTEx_ReceiveToIdle_DMA(REFEREE_UART_HANDLE, referee_rx_buf, REFEREE_RX_BUF_LENGTH);
}

// Put this in HAL_UARTEx_RxEventCallback
void referee_IDLE_Handler(UART_HandleTypeDef *huart)
{
  // Add received bytes to FIFO
  uint16_t rx_len = REFEREE_RX_BUF_LENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);
  fifo_s_puts(&referee_fifo, (char*)referee_rx_buf, rx_len);

  // Restart DMA reception
  HAL_UARTEx_ReceiveToIdle_DMA(REFEREE_UART_HANDLE, referee_rx_buf, REFEREE_RX_BUF_LENGTH);
}

// Put this in HAL_UART_ErrorCallback
void referee_Error_Handler(UART_HandleTypeDef *huart) {
	// Clear overrun error
    __HAL_UART_CLEAR_OREFLAG(REFEREE_UART_HANDLE);
    
  // Abort and restart reception
  HAL_UART_AbortReceive(REFEREE_UART_HANDLE);
  HAL_UARTEx_ReceiveToIdle_DMA(REFEREE_UART_HANDLE, referee_rx_buf, REFEREE_RX_BUF_LENGTH);
}
