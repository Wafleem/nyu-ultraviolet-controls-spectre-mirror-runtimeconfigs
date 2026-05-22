#include "referee.h"
#include "ref_structs.h"
#include "ref_crc.h"
#include "fifo.h"
#include "printing.h"
#include "logger.h"
#include "message_center.h"
#include "can_comm.h"
#include "vision_comm.h"

uint8_t referee_rx_buf[REFEREE_RX_BUF_LENGTH] DMA_SECTION;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
uint8_t referee_tx_buf[REFEREE_TX_BUF_LENGTH];
char out[REFEREE_TX_BUF_LENGTH * 3 + 1];
char *ptr = out;
fifo_s_t referee_fifo;
unpack_data_t referee_unpack_obj;
uint8_t referee_send_seq;

volatile uint32_t referee_rx_byte_count = 0;
volatile uint32_t referee_rx_callback_count = 0;
static uint8_t hud_phase;
static uint8_t hud_robot_id;

#define HUD_PHASE_DELETE          0u
#define HUD_PHASE_DELETE_GRAPHICS 1u
#define HUD_PHASE_DELETE_AIM_TXT  2u
#define HUD_PHASE_ADD_AIM         3u
#define HUD_PHASE_ADD_SPIN_TXT    4u
#define HUD_PHASE_ADD_CAP_TXT     5u
#define HUD_PHASE_EDIT_AIM        6u
#define HUD_PHASE_EDIT_SPIN_TXT   7u
#define HUD_PHASE_EDIT_CAP_TXT    8u

static void on_hud_supercap(const MsgEvent *ev, void *user_data) {
  (void)user_data;
  if (ev->size != sizeof(SupercapFeedbackEvent)) return;
  const SupercapFeedbackEvent *sc = (const SupercapFeedbackEvent *)ev->data;
  hud_state_set_supercap(sc->voltage_pct, sc->mode);
}

static void on_hud_vision(const MsgEvent *ev, void *user_data) {
  (void)user_data;
  if (ev->size != sizeof(Vision_Recv_s)) return;
  const Vision_Recv_s *v = (const Vision_Recv_s *)ev->data;
  hud_state_set_vision((uint8_t)v->target_state, HAL_GetTick());
}

static void on_hud_opstate(const MsgEvent *ev, void *user_data) {
  (void)user_data;
  if (ev->size != sizeof(HudOpStateEvent)) return;
  const HudOpStateEvent *op = (const HudOpStateEvent *)ev->data;
  hud_state_set_opstate(op->spin_mode, op->gimbal_follow, op->aimbot_engaged);
}

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
  LOG_INFO(LOG_TAG_REF, "rx_bytes=%lu callbacks=%lu",
           (unsigned long)referee_rx_byte_count,
           (unsigned long)referee_rx_callback_count);

  uint8_t rid = get_robot_id();
  if (rid == 0) {
    LOG_WARN(LOG_TAG_HUD, "waiting for robot_id (no 0x0201 received yet)");
    return;
  }

  if (hud_robot_id != rid) {
    hud_robot_id = rid;
    hud_phase = HUD_PHASE_DELETE;
  }

  uint16_t data_len;
  hud_operation_t op = ADD;
  const char *op_name;
  hud_text_slot_t text_slot = HUD_TEXT_SPIN;
  uint8_t send_text = 0;

  if (hud_phase == HUD_PHASE_DELETE) {
    data_len = sizeof(client_delete_layer_t);
    op_name = "DELETE_ALL";
  } else {
    op = (hud_phase < HUD_PHASE_EDIT_AIM) ? ADD : EDIT;
    op_name = (op == ADD) ? "ADD" : "EDIT";
    switch (hud_phase) {
      case HUD_PHASE_DELETE_GRAPHICS:
        data_len = sizeof(client_custom_graphic_seven_t);
        op = DELETE;
        op_name = "DELETE_GRAPHICS";
        break;
      case HUD_PHASE_ADD_AIM:
      case HUD_PHASE_EDIT_AIM:
        data_len = sizeof(client_custom_graphic_seven_t);
        break;
      case HUD_PHASE_DELETE_AIM_TXT:
        data_len = sizeof(client_custom_character_t);
        text_slot = HUD_TEXT_CLEAR_AIM;
        send_text = 1;
        op = DELETE;
        op_name = "DELETE_AIM_TEXT";
        break;
      case HUD_PHASE_ADD_SPIN_TXT:
      case HUD_PHASE_EDIT_SPIN_TXT:
        data_len = sizeof(client_custom_character_t);
        text_slot = HUD_TEXT_SPIN;
        send_text = 1;
        break;
      case HUD_PHASE_ADD_CAP_TXT:
      case HUD_PHASE_EDIT_CAP_TXT:
        data_len = sizeof(client_custom_character_t);
        text_slot = HUD_TEXT_CAP;
        send_text = 1;
        break;
      default:
        data_len = sizeof(client_custom_character_t);
        text_slot = HUD_TEXT_SPIN;
        send_text = 1;
        break;
    }
  }

  uint16_t cmd_id    = 0x0301;
  uint16_t frame_len = 7 + data_len + 2;

  memset(referee_tx_buf, 0, frame_len);
  referee_tx_buf[0] = 0xA5;
  referee_tx_buf[1] = data_len & 0xFF;
  referee_tx_buf[2] = (data_len >> 8) & 0xFF;
  referee_tx_buf[3] = referee_send_seq;
  append_CRC8_check_sum(referee_tx_buf, 5);
  referee_tx_buf[5] = cmd_id & 0xFF;
  referee_tx_buf[6] = (cmd_id >> 8) & 0xFF;
  if (hud_phase == HUD_PHASE_DELETE) {
    build_delete_all_for_robot(referee_tx_buf + 7, rid);
  } else if (send_text) {
    build_hud_text_for_robot(referee_tx_buf + 7, op, rid, text_slot);
  } else {
    build_hud_data_for_robot(referee_tx_buf + 7, op, rid);
  }
  append_CRC16_check_sum(referee_tx_buf, frame_len);

  ptr = out;
  for (int i = 0; i < frame_len; i++) {
    ptr += sprintf(ptr, "%02X ", referee_tx_buf[i]);
  }

  HAL_StatusTypeDef tx_status = HAL_UART_Transmit(REFEREE_UART_HANDLE,
                                                   referee_tx_buf, frame_len, 50);
  if (tx_status != HAL_OK) {
    LOG_ERROR(LOG_TAG_HUD, "HUD TX ERR status=%u seq=%u", (unsigned)tx_status, referee_send_seq);
  } else {
    LOG_INFO(LOG_TAG_HUD,
             "HUD %s phase=%u rid=%u recv=0x%X len=%u seq=%u\r\n%s",
             op_name, (unsigned)hud_phase, rid, (unsigned)((uint16_t)rid + 0x100),
             frame_len, referee_send_seq, out);
    if (hud_phase < HUD_PHASE_EDIT_CAP_TXT) {
      hud_phase++;
    } else {
      hud_phase = HUD_PHASE_EDIT_AIM;
    }
  }

  referee_send_seq++;
}

// Initialize referee interpreter
void referee_init(void)
{
  // Initialize referee structs
  ref_structs_init();
  memset(referee_tx_buf, 0, sizeof(referee_tx_buf));
  referee_send_seq = 0;
  hud_phase = HUD_PHASE_DELETE;
  hud_robot_id = 0;

  // Initialize FIFO
  fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);

  // Disable rate limiting on REF/HUD logger tags so every print comes through
  Logger_SetRate(LOG_TAG_REF, 0);
  Logger_SetRate(LOG_TAG_HUD, 0);
  LOG_INFO(LOG_TAG_HUD, "HUD sender mode=SPIN_SUPERCAP_ONLY");

  (void)MsgCenter_Subscribe(TOPIC_SUPERCAP_FEEDBACK, on_hud_supercap, NULL);
  (void)MsgCenter_Subscribe(TOPIC_VISION_DATA,       on_hud_vision,   NULL);
  (void)MsgCenter_Subscribe(TOPIC_HUD_OPSTATE,       on_hud_opstate,  NULL);

  // Start DMA reception with IDLE detection
  HAL_UARTEx_ReceiveToIdle_DMA(REFEREE_UART_HANDLE, referee_rx_buf, REFEREE_RX_BUF_LENGTH);
}

// Put this in HAL_UARTEx_RxEventCallback
void referee_IDLE_Handler(UART_HandleTypeDef *huart, uint16_t size)
{
  // Add received bytes to FIFO
  uint16_t rx_len = size;
  referee_rx_byte_count += rx_len;
  referee_rx_callback_count++;
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
