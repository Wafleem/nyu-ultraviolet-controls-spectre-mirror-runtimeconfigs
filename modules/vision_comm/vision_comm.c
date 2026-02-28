/**
 * @file vision_comm.c
 * @brief Vision communication module - using USART6
 */

#include "vision_comm.h"
#include "seasky_protocol.h"
#include "message_center.h"
#include "gyro_data.h"
#include "stm32h7xx_hal.h"
#include "printing.h"
#include <string.h>
#include <stdio.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;

// UART receive buffer (double buffer for DMA/IT mode)
uint8_t uart_recv_buff[VISION_RECV_SIZE];  // Made non-static for access in HAL callback
static uint8_t uart_recv_processing[VISION_RECV_SIZE];

// Vision send frequency control (100Hz = 10ms interval)
#define VISION_SEND_INTERVAL_MS 10
static uint32_t last_send_time = 0;

// Debug counters (ISR-safe, incremented in callback, printed from task context)
volatile uint32_t dbg_rx_count      = 0;  // total UART callbacks fired
volatile uint32_t dbg_parse_ok      = 0;  // packets that passed CRC + cmd_id check
volatile uint32_t dbg_parse_fail    = 0;  // packets that failed CRC or wrong cmd_id
volatile uint32_t dbg_len_fail      = 0;  // callbacks where len was out of range
static volatile uint32_t dbg_attitude_sent = 0;  // attitude frames sent to Jetson

/**
 * @brief UART receive callback function (called in UART interrupt)
 */
void VisionComm_RxCallback(uint8_t *buf, uint32_t len)
{
    uint16_t flag_register;
    dbg_rx_count++;

    // Copy data to processing buffer
    if (len >= 18 && len <= VISION_RECV_SIZE) {
        memcpy(uart_recv_processing, buf, len);

        // Try CRC-validated parse first
        uint16_t cmd_id = get_protocol_info(uart_recv_processing,
                                            &flag_register,
                                            (uint8_t *)&recv_data.pitch);

        // Fallback: if CRC fails, manually parse assuming valid Seasky frame
        if (cmd_id != 0x0001) {
            cmd_id = (uint16_t)uart_recv_processing[4] | ((uint16_t)uart_recv_processing[5] << 8);
            if (cmd_id == 0x0001) {
                flag_register = (uint16_t)uart_recv_processing[6] | ((uint16_t)uart_recv_processing[7] << 8);
                memcpy((uint8_t *)&recv_data.pitch, uart_recv_processing + 8, 8); // 2 floats
            }
        }

        if (cmd_id == 0x0001) {
            recv_data.fire_mode = (Fire_Mode_e)(flag_register & 0x03);
            recv_data.target_state = (Target_State_e)((flag_register >> 2) & 0x03);
            recv_data.target_type = (Target_Type_e)((flag_register >> 4) & 0x0F);

            // Mark data as updated
            recv_data.updated = 1;
            dbg_parse_ok++;

            // Publish vision data to message center (from ISR context!)
            (void)MsgCenter_PublishFromISR(TOPIC_VISION_DATA, &recv_data, sizeof(Vision_Recv_s));
        } else {
            // CRC failed or unexpected cmd_id
            dbg_parse_fail++;
        }
    } else {
        // Wrong packet length - likely framing error or partial packet
        dbg_len_fail++;
    }

    // Restart reception
    VisionComm_StartReceive();
}

/**
 * @brief Start UART reception for vision communication
 */
void VisionComm_StartReceive(void)
{
    // Use HAL_UARTEx_ReceiveToIdle_IT for variable length reception
    HAL_UARTEx_ReceiveToIdle_IT(&VISION_UART_HANDLE, uart_recv_buff, VISION_RECV_SIZE);
}

/**
 * @brief IMU update callback - sends vision data at controlled frequency
 */
static void on_imu_update(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    
    if (ev->size == sizeof(SensorData)) {
        const SensorData *sensor_data = (const SensorData *)ev->data;
        uint32_t current_time = HAL_GetTick();
        
        // Control send frequency (100Hz)
        if (current_time - last_send_time >= VISION_SEND_INTERVAL_MS) {
            // Set attitude data from gimbal IMU (EKF Euler angles in degrees).
            // NOTE: Previously this was g_gz/g_gx/g_gy which are gyro angular
            // velocities (rad/s), NOT orientation. The Jetson needs actual
            // orientation angles for auto-aim prediction to work.
            VisionComm_SetAltitude(sensor_data->yaw, sensor_data->pitch, sensor_data->roll);
            VisionComm_SetFlag(COLOR_BLUE, VISION_MODE_AIM, SMALL_AMU_15);
            VisionComm_Send();
            USB_CDC_Printf("[VISION] Sent attitude: yaw=%.2f, pitch=%.2f, roll=%.2f\r\n",
                           sensor_data->yaw, sensor_data->pitch, sensor_data->roll);
            dbg_attitude_sent++;

            /* VISDBG disabled for clean vision testing
            static uint32_t last_dbg_print = 0;
            if (current_time - last_dbg_print >= 1000) {
                last_dbg_print = current_time;
                USB_CDC_Printf("VISDBG: rx=%lu ok=%lu fail=%lu len_err=%lu att_sent=%lu | last: fm=%d ts=%d pt=%.4f yw=%.4f\r\n",
                               dbg_rx_count, dbg_parse_ok, dbg_parse_fail, dbg_len_fail, dbg_attitude_sent,
                               recv_data.fire_mode, recv_data.target_state,
                               recv_data.pitch, recv_data.yaw);
            } */

            last_send_time = current_time;
        }
    }
}

/**
 * @brief Initialize vision communication
 */
Vision_Recv_s *VisionComm_Init(void)
{
    // Clear receive and send data
    memset(&recv_data, 0, sizeof(Vision_Recv_s));
    memset(&send_data, 0, sizeof(Vision_Send_s));
    memset(uart_recv_buff, 0, sizeof(uart_recv_buff));
    memset(uart_recv_processing, 0, sizeof(uart_recv_processing));

    // Subscribe to IMU updates for sending vision data
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

    // Start UART reception
    VisionComm_StartReceive();

    return &recv_data;
}

/**
 * @brief Set flags
 */
void VisionComm_SetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

/**
 * @brief Set attitude data
 */
void VisionComm_SetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief Send vision data
 */
void VisionComm_Send(void)
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;

    // Pack flag register from send_data fields set by VisionComm_SetFlag().
    // Previously hardcoded to (30 << 8 | 0x01) which always sent
    // bullet_speed=30 and enemy_color=BLUE regardless of SetFlag calls.
    flag_register = ((uint16_t)send_data.bullet_speed << 8)
                  | ((uint16_t)send_data.work_mode << 1)
                  | ((uint16_t)send_data.enemy_color & 0x01);

    // Convert data to seasky protocol packet
    get_protocol_send_data(0x02,                // cmd_id = 0x0002 (attitude data)
                          flag_register,
                          &send_data.yaw,       // 3 floats: yaw, pitch, roll
                          3,
                          send_buff,
                          &tx_len);

    // Send via USART6
    HAL_UART_Transmit(&VISION_UART_HANDLE, send_buff, tx_len, HAL_MAX_DELAY);
}

/**
 * @brief Get receive data
 */
Vision_Recv_s *VisionComm_GetData(void)
{
    return &recv_data;
}

