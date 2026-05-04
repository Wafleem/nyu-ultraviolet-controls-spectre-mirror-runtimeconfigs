/**
 * ToF Sensor Module — VL53L1CX over I2C3
 *
 * Initializes the sensor, polls for ranging data, and publishes
 * distance measurements to TOPIC_TOF_UPDATE via the message center.
 */

#include "tof_sensor.h"
#include "VL53L1X_api.h"
#include "imu.h"
#include "logger.h"
#include "message_center.h"
#include <string.h>

#ifdef USE_FREERTOS
#include "cmsis_os.h"
#define TOF_DELAY(ms) osDelay(ms)
#else
#include "main.h"
#define TOF_DELAY(ms) HAL_Delay(ms)
#endif



/* Published data */
ToF_Data_t ToF_Data;
const RobotConfig_t *s_robot_config = NULL;
volatile uint8_t tof_initialized = 0;

int8_t ToF_Init(void) {
  VL53L1X_ERROR status;
  uint16_t sensor_id;
  uint8_t boot_state = 0;

  memset(&ToF_Data, 0, sizeof(ToF_Data));

  LOG_INFO(LOG_TAG_SYS, "ToF init on I2C3, addr=0x%02X", TOF_I2C_ADDR);
  TOF_DELAY(50);

  /* Wait for sensor to boot */
  for (int i = 0; i < 100; i++) {
    status = VL53L1X_BootState(TOF_I2C_ADDR, &boot_state);
    if (i == 0) {
      LOG_INFO(LOG_TAG_SYS, "ToF BootState: status=%d boot=%d", status,
               boot_state);
      TOF_DELAY(50);
    }
    if (boot_state != 0)
      break;
    TOF_DELAY(10);
  }
  if (boot_state == 0) {
    LOG_ERROR(LOG_TAG_SYS, "ToF boot timeout (status=%d)", status);
    TOF_DELAY(50);
    return -1;
  }

  /* Verify sensor identity */
  status = VL53L1X_GetSensorId(TOF_I2C_ADDR, &sensor_id);
  if (status != 0) {
    LOG_ERROR(LOG_TAG_SYS, "ToF I2C read failed (status %d)", status);
    TOF_DELAY(50);
    return -1;
  }
  LOG_INFO(LOG_TAG_SYS, "ToF Sensor ID: 0x%04X", sensor_id);
  TOF_DELAY(50);

  /* Load default 135-byte configuration */
  status = VL53L1X_SensorInit(TOF_I2C_ADDR);
  if (status != 0) {
    LOG_ERROR(LOG_TAG_SYS, "ToF SensorInit failed (status %d)", status);
    TOF_DELAY(50);
    return -1;
  }

  /* Configure for long-range mode (up to 4m) */
  status = VL53L1X_SetDistanceMode(TOF_I2C_ADDR, 2);
  if (status != 0) {
    LOG_ERROR(LOG_TAG_SYS, "ToF SetDistanceMode failed");
    return -1;
  }

  /* 50ms timing budget */
  status = VL53L1X_SetTimingBudgetInMs(TOF_I2C_ADDR, 50);
  if (status != 0) {
    LOG_ERROR(LOG_TAG_SYS, "ToF SetTimingBudget failed");
    return -1;
  }

  /* Inter-measurement period must be >= timing budget */
  status = VL53L1X_SetInterMeasurementInMs(TOF_I2C_ADDR, 55);
  if (status != 0) {
    LOG_ERROR(LOG_TAG_SYS, "ToF SetInterMeasurement failed");
    return -1;
  }

  /* Start continuous ranging */
  status = VL53L1X_StartRanging(TOF_I2C_ADDR);
  if (status != 0) {
    LOG_ERROR(LOG_TAG_SYS, "ToF StartRanging failed");
    return -1;
  }

  tof_initialized = 1;
  LOG_INFO(LOG_TAG_SYS, "ToF VL53L1CX initialized - ranging started");
  TOF_DELAY(50);
  return 0;
}

static uint32_t tof_print_counter = 0;
static uint8_t tof_reset_latched = 0;

void ToF_Task(void) {
  if (!tof_initialized)
    return;

  uint8_t data_ready = 0;
  VL53L1X_ERROR status;

  status = VL53L1X_CheckForDataReady(TOF_I2C_ADDR, &data_ready);
  if (status != 0 || !data_ready)
    return;

  /* Read full result in one shot */
  VL53L1X_Result_t result;
  status = VL53L1X_GetResult(TOF_I2C_ADDR, &result);
  if (status != 0) {
    VL53L1X_ClearInterrupt(TOF_I2C_ADDR);
    return;
  }

  /* Clear interrupt for next measurement */
  VL53L1X_ClearInterrupt(TOF_I2C_ADDR);

  /* Update published data */
  ToF_Data.distance_mm = result.Distance;
  ToF_Data.range_status = result.Status;
  ToF_Data.signal_rate = result.SigPerSPAD;
  ToF_Data.ambient_rate = result.Ambient;
  ToF_Data.valid = (result.Status == 0) ? 1 : 0;

  if (ToF_Data.valid && result.Distance < TOF_RESET_THRESHOLD_MM) {
    if (!tof_reset_latched) {
      if (s_robot_config != NULL) {
        IMU_SetYaw(s_robot_config->aligned_yaw);
      }
      tof_reset_latched = 1;
      LOG_INFO(LOG_TAG_SYS, "ToF=%dmm < %dmm, EKF yaw reset",
               result.Distance, TOF_RESET_THRESHOLD_MM);
    }
  } else {
    tof_reset_latched = 0;
  }

  /* Print every 10th reading (~2Hz at 20Hz poll rate) */
  if (++tof_print_counter % 10 == 0) {
    LOG_INFO(
        LOG_TAG_SYS,
        "ToF: %dmm status=%d sig=%d amb=%d EKF: roll=%.2f pitch=%.2f yaw=%.2f",
        result.Distance, result.Status, result.SigPerSPAD, result.Ambient,
        (double)Gimbal_Sensor.ekf_roll, (double)Gimbal_Sensor.ekf_pitch,
        (double)Gimbal_Sensor.ekf_yaw);
  }

  /* Publish to message center */
  MsgCenter_Publish(TOPIC_TOF_UPDATE, &ToF_Data, sizeof(ToF_Data), 0);
}

void ToF_SetRobotConfig(const RobotConfig_t *robot_config) {
  s_robot_config = robot_config;
}
