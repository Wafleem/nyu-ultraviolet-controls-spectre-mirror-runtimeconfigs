/**
 * @file tests.c
 * @brief Hardware test functions for IMU, CAN, and RC peripherals
 */
#include "tests.h"
#include "imu.h"
#include "can.h"
#include "remote_control.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Local buffer for formatting output */
static char test_buf[512];

void Test_IMU_Report(void)
{
    int len = 0;

    len = sprintf(test_buf, "----- IMU DATA -----\r\n");
    CDC_Transmit_FS((uint8_t*)test_buf, len);

    if (imu_initialized) {
        System_Read_And_Process();

        len = sprintf(test_buf, "Accel: X=%d Y=%d Z=%d\r\n",
            IMU_System.accel_x, IMU_System.accel_y, IMU_System.accel_z);
        CDC_Transmit_FS((uint8_t*)test_buf, len);

        len = sprintf(test_buf, "Gyro:  X=%d Y=%d Z=%d\r\n",
            IMU_System.gyro_x, IMU_System.gyro_y, IMU_System.gyro_z);
        CDC_Transmit_FS((uint8_t*)test_buf, len);

        len = sprintf(test_buf, "Mag:   X=%d Y=%d Z=%d (bias: %d,%d,%d)\r\n",
            IMU_System.mag_x, IMU_System.mag_y, IMU_System.mag_z,
            IMU_System.mag_bias_x, IMU_System.mag_bias_y, IMU_System.mag_bias_z);
        CDC_Transmit_FS((uint8_t*)test_buf, len);

        int16_t temp_int = (int16_t)(IMU_System.temp_c * 10);
        len = sprintf(test_buf, "Temp:  %d.%d C\r\n",
            temp_int / 10, abs(temp_int % 10));
        CDC_Transmit_FS((uint8_t*)test_buf, len);

        int16_t roll_int = (int16_t)(IMU_System.roll * 10);
        int16_t pitch_int = (int16_t)(IMU_System.pitch * 10);
        len = sprintf(test_buf, "Roll: %s%d.%d deg | Pitch: %s%d.%d deg\r\n",
            (roll_int < 0) ? "-" : "", abs(roll_int) / 10, abs(roll_int % 10),
            (pitch_int < 0) ? "-" : "", abs(pitch_int) / 10, abs(pitch_int % 10));
        CDC_Transmit_FS((uint8_t*)test_buf, len);

        int32_t mag_noise_int = (int32_t)(IMU_System.mag_noise * 1000);
        len = sprintf(test_buf, "Mag Noise: %ld.%03ld\r\n\r\n",
            (long)(mag_noise_int / 1000), (long)abs((int)(mag_noise_int % 1000)));
        CDC_Transmit_FS((uint8_t*)test_buf, len);

    } else {
        len = sprintf(test_buf, "IMU: Not initialized!\r\n\r\n");
        CDC_Transmit_FS((uint8_t*)test_buf, len);
    }
}

void Test_CAN_Loopback(uint32_t sent_number)
{
    int len;

    len = sprintf(test_buf, "========== CAN LOOP %lu ==========\r\n", sent_number);
    CDC_Transmit_FS((uint8_t*)test_buf, len);

    if (can_send_number(sent_number)) {
        len = sprintf(test_buf, "CAN1 TX: Sent %lu\r\n", sent_number);
        CDC_Transmit_FS((uint8_t*)test_buf, len);
    } else {
        len = sprintf(test_buf, "CAN1 TX: ERROR!\r\n");
        CDC_Transmit_FS((uint8_t*)test_buf, len);
    }

    uint32_t rxval = 0;
    if (can_check_received(&rxval)) {
        len = sprintf(test_buf, "CAN2 RX: Received %lu\r\n\r\n", rxval);
        CDC_Transmit_FS((uint8_t*)test_buf, len);
    } else {
        len = sprintf(test_buf, "CAN2 RX: TIMEOUT!\r\n\r\n");
        CDC_Transmit_FS((uint8_t*)test_buf, len);
    }
}

void Test_FlySky_Report(void)
{
    int len;
    static uint8_t rx_frame[RC_FRAME_LENGTH];

    len = sprintf(test_buf, "----- FLYSKY RC DATA -----\r\n");

    if (RC_sync_state == RC_SYNCED) {
        const RC_ctrl_t *raw_rc = get_remote_control_point();
        len += sprintf(test_buf + len, "Status: CONNECTED, Frame: %ld\r\nChannels: ",
            RC_GetFrameCount());

        for (uint32_t i = 0; i < 6; ++i) {
            len += sprintf(test_buf + len, "%4d ", raw_rc->rc.ch[i]);
        }
        len += sprintf(test_buf + len, "\r\nSwitches: ");
        for (uint32_t i = 0; i < 4; ++i) {
            len += sprintf(test_buf + len, "%4d ", raw_rc->rc.s[i]);
        }
        len += sprintf(test_buf + len, "\r\n\r\n");
    } else {
        RC_GetLastFrame(rx_frame);
        len += sprintf(test_buf + len,
            "Status: NOT CONNECTED\r\n"
            "  Sync: %d, Header: 0x%02X 0x%02X\r\n"
            "  UART State: %ld, ErrorCode: 0x%lX\r\n\r\n",
            RC_sync_state, rx_frame[0], rx_frame[1],
            RC_UART->RxState, RC_UART->ErrorCode);
    }

    CDC_Transmit_FS((uint8_t*)test_buf, len);
}

void Test_IMU_PrintCompact(void)
{
    if (!imu_initialized) {
        return;
    }

    int16_t roll_int = (int16_t)(IMU_System.roll * 10);
    int16_t pitch_int = (int16_t)(IMU_System.pitch * 10);
    int32_t noise_int = (int32_t)(IMU_System.mag_noise * 1000);

    int len = sprintf(test_buf,
        "A:%6d,%6d,%6d G:%6d,%6d,%6d M:%6d,%6d,%6d R:%s%d.%d P:%s%d.%d N:%ld.%03ld\r\n",
        IMU_System.accel_x, IMU_System.accel_y, IMU_System.accel_z,
        IMU_System.gyro_x, IMU_System.gyro_y, IMU_System.gyro_z,
        IMU_System.mag_x, IMU_System.mag_y, IMU_System.mag_z,
        (roll_int < 0) ? "-" : "", abs(roll_int) / 10, abs(roll_int % 10),
        (pitch_int < 0) ? "-" : "", abs(pitch_int) / 10, abs(pitch_int % 10),
        (long)(noise_int / 1000), (long)abs((int)(noise_int % 1000)));

    CDC_Transmit_FS((uint8_t*)test_buf, len);
}
