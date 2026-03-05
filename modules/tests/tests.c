/**
 * @file tests.c
 * @brief Hardware test functions for IMU, CAN, and RC peripherals
 */
#include "tests.h"
#include "can.h"
#include "imu.h"
#include "remote_control.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Local buffer for formatting output */
static char test_buf[512];

void Test_IMU_Report(void) {
  int len = 0;

  len = sprintf(test_buf, "----- IMU DATA -----\r\n");
  CDC_Transmit_FS((uint8_t *)test_buf, len);

  if (imu_initialized) {
    System_Read_And_Process();

    len = sprintf(test_buf, "Accel: X=%d Y=%d Z=%d\r\n", Gimbal_Sensor.accel_x,
                  Gimbal_Sensor.accel_y, Gimbal_Sensor.accel_z);
    CDC_Transmit_FS((uint8_t *)test_buf, len);

    len = sprintf(test_buf, "Gyro:  X=%d Y=%d Z=%d\r\n", Gimbal_Sensor.gyro_x,
                  Gimbal_Sensor.gyro_y, Gimbal_Sensor.gyro_z);
    CDC_Transmit_FS((uint8_t *)test_buf, len);

    len = sprintf(test_buf, "Mag:   X=%d Y=%d Z=%d (bias: %d,%d,%d)\r\n",
                  Gimbal_Sensor.mag_x, Gimbal_Sensor.mag_y, Gimbal_Sensor.mag_z,
                  Gimbal_Sensor.mag_bias_x, Gimbal_Sensor.mag_bias_y,
                  Gimbal_Sensor.mag_bias_z);
    CDC_Transmit_FS((uint8_t *)test_buf, len);

    int16_t temp_int = (int16_t)(Gimbal_Sensor.temp_c * 10);
    len = sprintf(test_buf, "Temp:  %d.%d C\r\n", temp_int / 10,
                  abs(temp_int % 10));
    CDC_Transmit_FS((uint8_t *)test_buf, len);

    int16_t roll_int = (int16_t)(Gimbal_Sensor.roll * 10);
    int16_t pitch_int = (int16_t)(Gimbal_Sensor.pitch * 10);
    len = sprintf(test_buf, "Roll: %s%d.%d deg | Pitch: %s%d.%d deg\r\n",
                  (roll_int < 0) ? "-" : "", abs(roll_int) / 10,
                  abs(roll_int % 10), (pitch_int < 0) ? "-" : "",
                  abs(pitch_int) / 10, abs(pitch_int % 10));
    CDC_Transmit_FS((uint8_t *)test_buf, len);

    int32_t mag_noise_int = (int32_t)(Gimbal_Sensor.mag_noise * 1000);
    len = sprintf(test_buf, "Mag Noise: %ld.%03ld\r\n\r\n",
                  (long)(mag_noise_int / 1000),
                  (long)abs((int)(mag_noise_int % 1000)));
    CDC_Transmit_FS((uint8_t *)test_buf, len);

  } else {
    len = sprintf(test_buf, "IMU: Not initialized!\r\n\r\n");
    CDC_Transmit_FS((uint8_t *)test_buf, len);
  }
}

void Test_CAN_Loopback(uint32_t sent_number) {
  int len;

  len =
      sprintf(test_buf, "========== CAN LOOP %lu ==========\r\n", sent_number);
  CDC_Transmit_FS((uint8_t *)test_buf, len);

  if (can_send_number(sent_number)) {
    len = sprintf(test_buf, "CAN1 TX: Sent %lu\r\n", sent_number);
    CDC_Transmit_FS((uint8_t *)test_buf, len);
  } else {
    len = sprintf(test_buf, "CAN1 TX: ERROR!\r\n");
    CDC_Transmit_FS((uint8_t *)test_buf, len);
  }

  uint32_t rxval = 0;
  if (can_check_received(&rxval)) {
    len = sprintf(test_buf, "CAN2 RX: Received %lu\r\n\r\n", rxval);
    CDC_Transmit_FS((uint8_t *)test_buf, len);
  } else {
    len = sprintf(test_buf, "CAN2 RX: TIMEOUT!\r\n\r\n");
    CDC_Transmit_FS((uint8_t *)test_buf, len);
  }
}

void Test_IMU_PrintCompact(void) {
  if (!imu_initialized) {
    return;
  }

  int len = sprintf(test_buf, "A:%6d,%6d,%6d G:%6d,%6d,%6d\r\n",
                    Gimbal_Sensor.accel_x, Gimbal_Sensor.accel_y, Gimbal_Sensor.accel_z,
                    Gimbal_Sensor.gyro_x, Gimbal_Sensor.gyro_y, Gimbal_Sensor.gyro_z);

  CDC_Transmit_FS((uint8_t *)test_buf, len);
}
