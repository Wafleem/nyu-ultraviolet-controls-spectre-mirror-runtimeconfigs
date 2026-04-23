#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "fdcan.h"
#include "config_types.h"
#include "motor_registry.h"

// TX frame structure for aggregating motor commands
#define CAN_TX_FRAME_COUNT 3  // Support 0x200, 0x1FF, 0x2FF
typedef struct {
    uint16_t std_id;           // Standard ID (0x200, 0x1FF, or 0x2FF)
    int16_t currents[4];       // Currents for 4 motor slots
    uint8_t pending;           // true if frame needs to be sent
} CANTxFrame_t;

// CAN manager structure (FDCAN version for STM32H7)
typedef struct {
    FDCAN_HandleTypeDef *hfdcan;   // FDCAN handle (was CAN_HandleTypeDef *hcan)
    CAN_Channel_t channel;
    uint8_t initialized;

    // Motor registry for dynamic motor lookup
    MotorRegistry_t *registry;

    // TX frame aggregation buffers
    CANTxFrame_t tx_frames[CAN_TX_FRAME_COUNT];

    // Debug counters
    uint32_t tx_ok;
    uint32_t tx_err;
    uint32_t rx_frames;
    uint32_t last_rx_id;
    uint32_t last_tx_time;
    uint32_t last_rx_time;
} CAN_Manager_t;

/**
 * @brief Initialize CAN manager with robot configuration
 * @param manager CAN manager pointer
 * @param channel CAN channel (CAN_CHANNEL_1 or CAN_CHANNEL_2)
 * @param hfdcan FDCAN handle pointer
 * @param robot_config Robot configuration (for motor registry initialization)
 * @param registry_storage Pointer to registry storage (allocated by caller)
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager,
                                  CAN_Channel_t channel,
                                  FDCAN_HandleTypeDef *hfdcan,
                                  const RobotConfig_t *robot_config,
                                  MotorRegistry_t *registry_storage);

/**
 * @brief Start CAN communication (configures FDCAN filters and enables RX interrupt)
 * @param manager CAN manager pointer
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_Start(CAN_Manager_t *manager);

/**
 * @brief Check if CAN is initialized
 */
bool CAN_Manager_IsInitialized(const CAN_Manager_t *manager);

/**
 * @brief Get FDCAN handle
 */
FDCAN_HandleTypeDef* CAN_Manager_GetHandle(const CAN_Manager_t *manager);

/**
 * @brief Send 4 motor currents in one CAN frame
 */
HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(FDCAN_HandleTypeDef *hfdcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4);

/**
 * @brief Send GM6020 current by motor id (1..7)
 */
HAL_StatusTypeDef CAN_Manager_SendGM6020Current(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int16_t current);

/**
 * @brief Send motor current by motor ID (aggregated API)
 * Call CAN_Manager_FlushTx() to actually send frames.
 */
HAL_StatusTypeDef CAN_Manager_SendMotorCurrent(CAN_Manager_t *manager,
                                              uint8_t motor_id,
                                              int16_t current);

/**
 * @brief Flush all pending TX frames
 */
HAL_StatusTypeDef CAN_Manager_FlushTx(CAN_Manager_t *manager);

/**
 * @brief Send Wraith (supercap) discharge command on CAN1.
 *        Frame: standard ID 0x404, DLC 1.
 *        byte 0 = 0x01 -> start discharging
 *        byte 0 = 0x00 -> stop discharging (resume charging)
 *        Per Controls_Supercap CAN_PROTOCOL.md.
 */
HAL_StatusTypeDef CAN_Manager_SendSupercapDischarge(bool enable);

/**
 * @brief Send Wraith (supercap) per-robot charging power ceiling on CAN1.
 *        Frame: standard ID 0x408, DLC 4, bytes 0-3 = float32 LE (watts).
 *        Per Controls_Supercap CAN_PROTOCOL.md. Intended to be sent when
 *        the referee system reveals (or changes) the robot ID.
 */
HAL_StatusTypeDef CAN_Manager_SendSupercapChargeLimit(float watts);

/**
 * @brief Get CAN manager from FDCAN handle (reverse lookup)
 */
CAN_Manager_t* CAN_Manager_FromHandle(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Process CAN receive callback
 * @param manager CAN manager pointer
 * @param hfdcan FDCAN handle that triggered the callback
 */
void CAN_Manager_ProcessCallback(CAN_Manager_t *manager, FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Global CAN callback (called from HAL_FDCAN_RxFifo0Callback)
 */
void CAN_Manager_GlobalCallback(FDCAN_HandleTypeDef *hfdcan);

// Debug accessor helpers
uint32_t CAN_Manager_GetTxOk(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetTxErr(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetRxFrames(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastRxId(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastTxTime(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastRxTime(const CAN_Manager_t *m);

#endif // CAN_MANAGER_H
