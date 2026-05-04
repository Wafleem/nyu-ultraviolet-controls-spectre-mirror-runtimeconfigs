#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 8-bit I2C address (7-bit 0x29 << 1) */
#define TOF_I2C_ADDR 0x52
#define TOF_RESET_THRESHOLD_MM 25

/* ToF measurement data published via TOPIC_TOF_UPDATE */
typedef struct {
  uint16_t distance_mm;  /* Distance in millimeters (0 if invalid) */
  uint8_t range_status;  /* 0 = valid measurement, >0 = error code */
  uint16_t signal_rate;  /* Signal rate in kcps/SPAD */
  uint16_t ambient_rate; /* Ambient rate in kcps/SPAD */
  uint8_t valid;         /* 1 = data is fresh and valid, 0 = stale/error */
} ToF_Data_t;

extern volatile uint8_t tof_initialized;
extern ToF_Data_t ToF_Data;

/**
 * @brief Initialize the VL53L1CX sensor over I2C3.
 *        Verifies sensor ID, loads default config, sets long-range mode,
 *        configures 50ms timing budget, and starts continuous ranging.
 * @return 0 on success, -1 on failure
 */
int8_t ToF_Init(void);

/**
 * @brief Poll sensor for new data and publish via message center.
 *        Call this periodically from the ToF FreeRTOS task.
 */
void ToF_Task(void);

/**
 * @brief Set the robot config
 */
void ToF_SetRobotConfig(const RobotConfig_t *robot_config);

#ifdef __cplusplus
}
#endif

#endif /* TOF_SENSOR_H */
