/**
 * @file can_manager.c
 * @brief FDCAN manager for STM32H7 - ported from NYUSH bxCAN version
 *
 * Preserves the same external API (CAN_Manager_Init/Start/SendMotorCurrent/FlushTx/etc.)
 * but all internal HAL calls use the STM32H7 FDCAN peripheral instead of bxCAN.
 *
 * Key API changes from bxCAN -> FDCAN:
 *   CAN_HandleTypeDef  ->FDCAN_HandleTypeDef
 *   CAN_FilterTypeDef  -> FDCAN_FilterTypeDef (no bank numbering)
 *   CAN_TxHeaderTypeDef  -> FDCAN_TxHeaderTypeDef (more fields)
 *   CAN_RxHeaderTypeDef  -> FDCAN_RxHeaderTypeDef
 *   HAL_CAN_*  -> HAL_FDCAN_*
 *   rx.StdId -> rx.Identifier
 *   rx.DLC (raw uint8) -> rx.DataLength (FDCAN_DLC_BYTES_* enum)
 *   rx.IDE == CAN_ID_STD -> rx.IdType == FDCAN_STANDARD_ID
 */
#include "can_manager.h"
#include "motor_registry.h"
#include "robot_config.h"
#include "message_center.h"
#include "can_comm.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

static ChassisCalibEvent s_calib_buf = {};
static uint8_t s_calib_received = 0;

/*DLC conversion helpers (from existing can.c)*/

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

/* build a classic CAN FDCAN TX header*/

static void build_fdcan_tx_header(FDCAN_TxHeaderTypeDef *tx, uint16_t std_id)
{
    memset(tx, 0, sizeof(*tx));
    tx->Identifier = std_id;
    tx->IdType = FDCAN_STANDARD_ID;
    tx->TxFrameType = FDCAN_DATA_FRAME;
    tx->DataLength = FDCAN_DLC_BYTES_8;
    tx->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx->BitRateSwitch = FDCAN_BRS_OFF;
    tx->FDFormat = FDCAN_CLASSIC_CAN;
    tx->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx->MessageMarker = 0;
}

/* ================================================================== */
/*                        PUBLIC API                                   */
/* ================================================================== */

/*store FDCAN handle, no filter_bank (FDCAN vs CAN really is like Switch 2 vs Switch !) */
HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager,
                                  CAN_Channel_t channel,
                                  FDCAN_HandleTypeDef *hfdcan,
                                  const RobotConfig_t *robot_config,
                                  MotorRegistry_t *registry_storage)
{
    if (manager == NULL || hfdcan == NULL || robot_config == NULL || registry_storage == NULL) {
        return HAL_ERROR;
    }

    memset(manager, 0, sizeof(CAN_Manager_t));
    manager->hfdcan = hfdcan;
    manager->channel = channel;

    /* Initialize motor registry from config */
    manager->registry = registry_storage;
    MotorRegistry_Init(manager->registry, robot_config, channel);

    /* Initialize TX frame buffers (same IDs as main codebase) */
    manager->tx_frames[0].std_id = 0x200;
    manager->tx_frames[1].std_id = 0x1FF;
    manager->tx_frames[2].std_id = 0x2FF;

    manager->initialized = 1;
    return HAL_OK;
}

/* FDCAN filter + start + RX notification */
HAL_StatusTypeDef CAN_Manager_Start(CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return HAL_ERROR;

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;   /* Accept all */
    filter.FilterID2 = 0x000;   /* Mask = 0 = don't care */

    HAL_StatusTypeDef status;

    status = HAL_FDCAN_ConfigFilter(manager->hfdcan, &filter);
    if (status != HAL_OK) return status;

    status = HAL_FDCAN_ConfigGlobalFilter(manager->hfdcan,
                FDCAN_REJECT, FDCAN_REJECT,
                FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) return status;

    status = HAL_FDCAN_Start(manager->hfdcan);
    if (status != HAL_OK) return status;

    status = HAL_FDCAN_ActivateNotification(manager->hfdcan,
                FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

bool CAN_Manager_IsInitialized(const CAN_Manager_t *manager)
{
    if (manager == NULL) return false;
    return manager->initialized;
}

FDCAN_HandleTypeDef* CAN_Manager_GetHandle(const CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return NULL;
    return manager->hfdcan;
}

/*FDCAN RX message parsing */
void CAN_Manager_ProcessCallback(CAN_Manager_t *manager, FDCAN_HandleTypeDef *hfdcan)
{
    if (manager == NULL || !manager->initialized || hfdcan != manager->hfdcan) return;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8];
    uint32_t current_tick = HAL_GetTick();

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) != HAL_OK) return;

    manager->rx_frames++;
    manager->last_rx_id = rx.Identifier;    /* was rx.StdId */
    manager->last_rx_time = current_tick;

    /* Convert FDCAN DLC enum to byte count */
    uint8_t dlc = dlc_to_bytes(rx.DataLength);

    /* Publish raw CAN RX frame to message center (from ISR context!) */
    CanRxFrame f = { (uint16_t)rx.Identifier, dlc, {0} };
    if (dlc <= 8) { memcpy(f.data, d, dlc); }
    (void)MsgCenter_PublishFromISR(TOPIC_CAN_RX, &f, sizeof(f));

    /* Power reading at CAN ID 0x404 */
    if (rx.Identifier == 0x404) {
        uint32_t power_bytes = ((uint32_t)d[0] << 24)
                             | ((uint32_t)d[1] << 16)
                             | ((uint32_t)d[2] << 8)
                             | ((uint32_t)d[3]);
        float power_value;
        memcpy(&power_value, &power_bytes, sizeof(power_value));

        PowerFeedbackEvent pev = {
            .power = power_value
        };
        (void)MsgCenter_PublishFromISR(TOPIC_CHASSIS_POWER, &pev, sizeof(pev));
    }
    /* Supercap (Wraith) telemetry. Per Controls_Supercap CAN_PROTOCOL.md:
     *   0x405 (DLC 8): bytes 0-3 = pmm_w float LE, bytes 4-7 = chassis_w float LE
     *   0x406 (DLC 5): bytes 0-3 = voltage_pct float LE, byte 4 = mode uint8
     * Both are little-endian. Parsed BEFORE the dlc!=8 early return because
     * 0x406 has DLC 5. Latest values are merged into a static struct so each
     * frame republishes a complete snapshot. */
    if (rx.IdType == FDCAN_STANDARD_ID &&
        (rx.Identifier == 0x405 || rx.Identifier == 0x406)) {
        static SupercapFeedbackEvent s_sc = {0};
        if (rx.Identifier == 0x405 && dlc >= 8) {
            memcpy(&s_sc.pmm_w,     &d[0], sizeof(float));
            memcpy(&s_sc.chassis_w, &d[4], sizeof(float));
        } else if (rx.Identifier == 0x406 && dlc >= 5) {
            memcpy(&s_sc.voltage_pct, &d[0], sizeof(float));
            s_sc.mode = d[4];
        }
        s_sc.tick_ms = current_tick;
        (void)MsgCenter_PublishFromISR(TOPIC_SUPERCAP_FEEDBACK, &s_sc, sizeof(s_sc));
    }

    /* Only process standard ID frames with 8 bytes */
    if (rx.IdType != FDCAN_STANDARD_ID || dlc != 8) {
        return;
    }
    // Receive chassis IMU calibration values
    if (0x42E <= rx.Identifier  && rx.Identifier <= 0x430) {
        int index = 0x430 - rx.Identifier;
        uint32_t offset_bytes = ((uint32_t)d[0] << 24)
                              | ((uint32_t)d[1] << 16)
                              | ((uint32_t)d[2] << 8)
                              | ((uint32_t)d[3]);
        float offset_value;
        memcpy(&offset_value, &offset_bytes, sizeof(float));
        s_calib_buf.offset[index] = offset_value;

        uint32_t normal_bytes = ((uint32_t)d[4] << 24)
                              | ((uint32_t)d[5] << 16)
                              | ((uint32_t)d[6] << 8)
                              | ((uint32_t)d[7]);
        float normal_value;
        memcpy(&normal_value, &normal_bytes, sizeof(float));
        s_calib_buf.normal[index] = normal_value;
        s_calib_received |= (1 << index);
    }
    if (s_calib_received == 0x07) {
        s_calib_received = 0;
        (void)MsgCenter_PublishFromISR(TOPIC_CHASSIS_CALIB, &s_calib_buf, sizeof(s_calib_buf));
    }
    // Receive chassis IMU attitude data
    if (rx.Identifier == 0x434) {
        int16_t raw_roll  = (int16_t)((d[0] << 8) | d[1]);
        int16_t raw_pitch = (int16_t)((d[2] << 8) | d[3]);
        int16_t raw_yaw   = (int16_t)((d[4] << 8) | d[5]);

        ChassisIMUFeedbackEvent yev;
        memcpy(&yev.roll,  &raw_roll,  sizeof(raw_roll));
        memcpy(&yev.pitch, &raw_pitch, sizeof(raw_pitch));
        memcpy(&yev.yaw,   &raw_yaw,   sizeof(raw_yaw));

        // Move angle readings from [0, 3600] to [0, 360]
        yev.roll = yev.roll / 10;
        yev.pitch = yev.pitch / 10;
        yev.yaw = yev.yaw / 10;
        (void)MsgCenter_PublishFromISR(TOPIC_CHASSIS_IMU, &yev, sizeof(yev));
    }
    /* Dynamic motor feedback processing using registry */
    const MotorConfig_t *motor = MotorRegistry_FindByRxId(manager->registry, rx.Identifier);
    if (motor == NULL) {
        return;  /* Not a registered motor */
    }

    if (motor->type == MOTOR_TYPE_M3508 || motor->type == MOTOR_TYPE_M2006) {
        uint16_t angle   = (uint16_t)((d[0] << 8) | d[1]);
        int16_t  speed   = (int16_t)((d[2] << 8) | d[3]);
        int16_t  current = (int16_t)((d[4] << 8) | d[5]);
        uint8_t  temp    = d[6];

        MotorFeedbackEvent ev = {
            .id = motor->motor_id,
            .angle = angle,
            .speed = speed,
            .current = current,
            .temp = temp,
            .tick_ms = current_tick
        };
        (void)MsgCenter_PublishFromISR(TOPIC_MOTOR_FEEDBACK, &ev, sizeof(ev));
    }
    else if (motor->type == MOTOR_TYPE_GM6020) {
        uint16_t angle_raw = (uint16_t)((d[0] << 8) | d[1]);
        int16_t  speed_rpm = (int16_t)((d[2] << 8) | d[3]);
        int16_t  current   = (int16_t)((d[4] << 8) | d[5]);

        GM6020FeedbackEvent gev = {
            .id = motor->motor_id,
            .angle = angle_raw,
            .speed = speed_rpm,
            .tick_ms = current_tick,
            .current = current
        };
        (void)MsgCenter_PublishFromISR(TOPIC_GM6020_FEEDBACK, &gev, sizeof(gev));
    }
}

/*Global callback - dispatches to correct manager */
extern CAN_Manager_t can1_manager;
extern CAN_Manager_t can2_manager;

void CAN_Manager_GlobalCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == CAN_Manager_GetHandle(&can1_manager)) {
        CAN_Manager_ProcessCallback(&can1_manager, hfdcan);
    } else if (hfdcan == CAN_Manager_GetHandle(&can2_manager)) {
        CAN_Manager_ProcessCallback(&can2_manager, hfdcan);
    }
}

/* HAL FDCAN RX callback - wired to global dispatcher */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        CAN_Manager_GlobalCallback(hfdcan);
    }
}

/* TX functions - FDCAN TX header construction */

HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(FDCAN_HandleTypeDef *hfdcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    if (hfdcan == NULL) return HAL_ERROR;

    static uint32_t last_tx_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tx_tick < 4) {
        return HAL_OK;
    }
    last_tx_tick = now;

    FDCAN_TxHeaderTypeDef tx;
    uint8_t d[8];

    build_fdcan_tx_header(&tx, std_id);

    d[0] = (uint8_t)(i1 >> 8); d[1] = (uint8_t)i1;
    d[2] = (uint8_t)(i2 >> 8); d[3] = (uint8_t)i2;
    d[4] = (uint8_t)(i3 >> 8); d[5] = (uint8_t)i3;
    d[6] = (uint8_t)(i4 >> 8); d[7] = (uint8_t)i4;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, d);

    CAN_Manager_t *m = NULL;
    if (hfdcan == can1_manager.hfdcan) m = &can1_manager;
    else if (hfdcan == can2_manager.hfdcan) m = &can2_manager;
    if (m) {
        if (st == HAL_OK) m->tx_ok++; else m->tx_err++;
        m->last_tx_time = HAL_GetTick();
    }
    return st;
}

HAL_StatusTypeDef CAN_Manager_SendGM6020Current(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int16_t current)
{
    if (hfdcan == NULL) return HAL_ERROR;

    static uint32_t last_tx_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tx_tick < 2) {
        return HAL_OK;
    }
    last_tx_tick = now;

    if (motor_id < 1 || motor_id > 7) return HAL_ERROR;
    if (current >  25000) current =  25000;
    if (current < -25000) current = -25000;

    uint16_t stdId = (motor_id <= 4) ? 0x1FF : 0x2FF;
    uint8_t  slot  = (motor_id <= 4) ? (uint8_t)(motor_id - 1) : (uint8_t)(motor_id - 5);

    FDCAN_TxHeaderTypeDef tx;
    uint8_t d[8] = {0};

    build_fdcan_tx_header(&tx, stdId);

    d[slot*2 + 0] = (uint8_t)((current >> 8) & 0xFF);
    d[slot*2 + 1] = (uint8_t)( current       & 0xFF);

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, d);

    CAN_Manager_t *m = NULL;
    if (hfdcan == can1_manager.hfdcan) m = &can1_manager;
    else if (hfdcan == can2_manager.hfdcan) m = &can2_manager;
    if (m) {
        if (st == HAL_OK) m->tx_ok++; else m->tx_err++;
        m->last_tx_time = HAL_GetTick();
    }
    return st;
}

HAL_StatusTypeDef CAN_Manager_SendMotorCurrent(CAN_Manager_t *manager,
                                              uint8_t motor_id,
                                              int16_t current)
{
    if (manager == NULL || !manager->initialized || manager->registry == NULL) {
        return HAL_ERROR;
    }

    const MotorConfig_t *motor = MotorRegistry_FindByMotorId(manager->registry, motor_id);
    if (motor == NULL) {
        return HAL_ERROR;
    }

    /* Clamp current based on motor type */
    if (motor->type == MOTOR_TYPE_GM6020) {
        if (current >  25000) current =  25000;
        if (current < -25000) current = -25000;
    } else if (motor->type == MOTOR_TYPE_M2006) {
        if (current >  10000) current =  10000;
        if (current < -10000) current = -10000;
    } else {
        if (current >  16384) current =  16384;
        if (current < -16384) current = -16384;
    }

    /* Find appropriate TX frame */
    CANTxFrame_t *tx_frame = NULL;
    for (uint8_t i = 0; i < CAN_TX_FRAME_COUNT; i++) {
        if (manager->tx_frames[i].std_id == motor->can_tx_id) {
            tx_frame = &manager->tx_frames[i];
            break;
        }
    }

    if (tx_frame == NULL) {
        return HAL_ERROR;
    }

    if (motor->tx_slot < 4) {
        tx_frame->currents[motor->tx_slot] = current;
        tx_frame->pending = 1;
    } else {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef CAN_Manager_FlushTx(CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef result = HAL_OK;

    for (uint8_t i = 0; i < CAN_TX_FRAME_COUNT; i++) {
        CANTxFrame_t *tx_frame = &manager->tx_frames[i];

        if (tx_frame->pending) {
            FDCAN_TxHeaderTypeDef tx_header;
            uint8_t data[8] = {0};

            build_fdcan_tx_header(&tx_header, tx_frame->std_id);

            /* Pack currents big-endian */
            for (uint8_t slot = 0; slot < 4; slot++) {
                data[slot * 2 + 0] = (uint8_t)((tx_frame->currents[slot] >> 8) & 0xFF);
                data[slot * 2 + 1] = (uint8_t)(tx_frame->currents[slot] & 0xFF);
            }

            HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(manager->hfdcan, &tx_header, data);
            if (status == HAL_OK) {
                manager->tx_ok++;
            } else {
                manager->tx_err++;
                result = HAL_ERROR;
            }
            manager->last_tx_time = HAL_GetTick();

            // Not clearing currents in case multiple apps write and flush the same CAN frame
            // memset(tx_frame->currents, 0, sizeof(tx_frame->currents));
            tx_frame->pending = 0;
        }
    }

    return result;
}

/* Send Wraith (supercap) discharge command on CAN1.
 * Standard ID 0x407, DLC 1. Per Controls_Supercap CAN_PROTOCOL.md. */
HAL_StatusTypeDef CAN_Manager_SendSupercapDischarge(bool enable)
{
    if (!can1_manager.initialized || can1_manager.hfdcan == NULL) {
        return HAL_ERROR;
    }

    FDCAN_TxHeaderTypeDef tx;
    memset(&tx, 0, sizeof(tx));
    tx.Identifier          = 0x407;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_1;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;

    uint8_t d[1] = { enable ? 0x01u : 0x00u };

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(can1_manager.hfdcan, &tx, d);
    if (st == HAL_OK) can1_manager.tx_ok++;
    else              can1_manager.tx_err++;
    can1_manager.last_tx_time = HAL_GetTick();
    return st;
}

/* Send Wraith (supercap) charging power ceiling on CAN1.
 * Standard ID 0x408, DLC 4, float32 LE watts. Per Controls_Supercap
 * CAN_PROTOCOL.md. Clamping is handled on the Wraith side. */
HAL_StatusTypeDef CAN_Manager_SendSupercapChargeLimit(float watts)
{
    if (!can1_manager.initialized || can1_manager.hfdcan == NULL) {
        return HAL_ERROR;
    }

    FDCAN_TxHeaderTypeDef tx;
    memset(&tx, 0, sizeof(tx));
    tx.Identifier          = 0x408;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_4;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;

    uint8_t d[4];
    memcpy(d, &watts, sizeof(float));

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(can1_manager.hfdcan, &tx, d);
    if (st == HAL_OK) can1_manager.tx_ok++;
    else              can1_manager.tx_err++;
    can1_manager.last_tx_time = HAL_GetTick();

    // USB_CDC_Printf("[0x408] Wraith charge limit -> %.1fW (tx=%d)\r\n", (double)watts, (int)st);

    return st;
}

CAN_Manager_t* CAN_Manager_FromHandle(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == can1_manager.hfdcan) {
        return &can1_manager;
    } else if (hfdcan == can2_manager.hfdcan) {
        return &can2_manager;
    }
    return NULL;
}

/* Debug counters */
uint32_t CAN_Manager_GetTxOk(const CAN_Manager_t *m){ return m?m->tx_ok:0; }
uint32_t CAN_Manager_GetTxErr(const CAN_Manager_t *m){ return m?m->tx_err:0; }
uint32_t CAN_Manager_GetRxFrames(const CAN_Manager_t *m){ return m?m->rx_frames:0; }
uint32_t CAN_Manager_GetLastRxId(const CAN_Manager_t *m){ return m?m->last_rx_id:0; }
uint32_t CAN_Manager_GetLastTxTime(const CAN_Manager_t *m){ return m?m->last_tx_time:0; }
uint32_t CAN_Manager_GetLastRxTime(const CAN_Manager_t *m){ return m?m->last_rx_time:0; }
