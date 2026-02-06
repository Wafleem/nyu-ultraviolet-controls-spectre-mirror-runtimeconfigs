#include "imu.h"
#include "spi.h"
#include "gpio.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* FreeRTOS-aware delay macro */
#ifdef USE_FREERTOS
#include "cmsis_os.h"
#define IMU_DELAY(ms) osDelay(ms)
#else
#define IMU_DELAY(ms) HAL_Delay(ms)
#endif

IMU_System_Data_t IMU_System;
volatile uint8_t imu_initialized = 0;

// Extern uart buffer from main.c for debug messages
extern char uart_buf[256];

// Magnetometer noise measurement buffers
#define MAG_NOISE_BUFFER_SIZE 100
static int16_t mag_x_buffer[MAG_NOISE_BUFFER_SIZE];
static int16_t mag_y_buffer[MAG_NOISE_BUFFER_SIZE];
static int16_t mag_z_buffer[MAG_NOISE_BUFFER_SIZE];
static uint16_t buffer_index = 0;
static bool buffers_filled = false;

// Low-level SPI / chip-select helpers (use CubeMX-generated SPI handles)
static inline void IMU_CS_Select(void)   { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); }
static inline void IMU_CS_Deselect(void) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); }
static inline void MAG_CS_Select(void)   { HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET); }
static inline void MAG_CS_Deselect(void) { HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET); }

static int8_t icm_write(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg & 0x7F, val};  // Clear MSB for write
    IMU_CS_Select();
    if (HAL_SPI_Transmit(&hspi2, data, 2, 100) != HAL_OK) { IMU_CS_Deselect(); return -1; }
    IMU_CS_Deselect();
    return 0;
}

static int8_t icm_read(uint8_t reg, uint8_t *val) {
    uint8_t tx = reg | 0x80;  //Set MSB for read
    IMU_CS_Select();
    if (HAL_SPI_Transmit(&hspi2, &tx, 1, 100) != HAL_OK) { IMU_CS_Deselect(); return -1; }
    if (HAL_SPI_Receive(&hspi2, val, 1, 100) != HAL_OK) { IMU_CS_Deselect(); return -1; }
    IMU_CS_Deselect();
    return 0;
}

static int8_t icm_read_burst(uint8_t start_reg, uint8_t *buf, uint16_t len) {
    uint8_t tx = start_reg | 0x80;  //Set MSB for read
    IMU_CS_Select();
    if (HAL_SPI_Transmit(&hspi2, &tx, 1, 100) != HAL_OK) { IMU_CS_Deselect(); return -1; }
    if (HAL_SPI_Receive(&hspi2, buf, len, 100) != HAL_OK) { IMU_CS_Deselect(); return -1; }
    IMU_CS_Deselect();
    return 0;
}

static int8_t mlx_cmd(uint8_t cmd, uint8_t *rx, uint16_t len) {
    uint8_t tx = cmd;
    MAG_CS_Select();
    if (HAL_SPI_Transmit(&hspi5, &tx, 1, 100) != HAL_OK) { MAG_CS_Deselect(); return -1; }
    if (rx && len > 0) { HAL_SPI_Receive(&hspi5, rx, len, 100); }
    MAG_CS_Deselect();
    return 0;
}

static int8_t mlx_write_reg(uint8_t reg, uint16_t val) {
    uint8_t tx[4] = {MLX_CMD_WRITE_REG, (uint8_t)((reg<<2)&0xFF), (uint8_t)(val>>8), (uint8_t)(val&0xFF)};
    MAG_CS_Select();
    if(HAL_SPI_Transmit(&hspi5, tx, 4, 100) != HAL_OK) { MAG_CS_Deselect(); return -1; }
    MAG_CS_Deselect();
    return 0;
}

int8_t IMU_Test_WhoAmI(void) {
    uint8_t whoami = 0;
    icm_read(ICM_REG_WHO_AM_I, &whoami);  // Read from 0x01

    sprintf(uart_buf, "WHO_AM_I: Read 0x%02X, Expected 0x%02X\r\n", whoami, ICM_WHO_AM_I_VAL);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    if (whoami == ICM_WHO_AM_I_VAL) return 0;  // Should be 0x6A
    return -1;
}

void Mag_Save_Biases_To_EPROM(void) {
    // Placeholder for Flash logic — implement if persistent storage is required
}

void Mag_Calibrate_And_Check_Noise(void) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t readings_x[50], readings_y[50], readings_z[50];
    uint8_t mraw[7];

    for(int i=0; i<50; i++) {
        mlx_cmd(MLX_CMD_START_SINGLE, NULL, 0);
        HAL_Delay(10);
        mlx_cmd(MLX_CMD_READ_MEAS, mraw, 7);

        readings_x[i] = (int16_t)(mraw[1]<<8 | mraw[2]);
        readings_y[i] = (int16_t)(mraw[3]<<8 | mraw[4]);
        readings_z[i] = (int16_t)(mraw[5]<<8 | mraw[6]);

        sum_x += readings_x[i];
        sum_y += readings_y[i];
        sum_z += readings_z[i];
    }

    IMU_System.mag_bias_x = sum_x / 50;
    IMU_System.mag_bias_y = sum_y / 50;
    IMU_System.mag_bias_z = sum_z / 50;

    Mag_Save_Biases_To_EPROM();

    float var_x = 0, var_y = 0, var_z = 0;
    for(int i=0; i<50; i++) {
        float diff_x = readings_x[i] - IMU_System.mag_bias_x;
        float diff_y = readings_y[i] - IMU_System.mag_bias_y;
        float diff_z = readings_z[i] - IMU_System.mag_bias_z;
        var_x += diff_x * diff_x;
        var_y += diff_y * diff_y;
        var_z += diff_z * diff_z;
    }
    var_x /= 50.0f;
    var_y /= 50.0f;
    var_z /= 50.0f;
    IMU_System.mag_noise = sqrtf((var_x + var_y + var_z) / 3.0f);
}

int8_t System_Sensors_Init(void) {
    IMU_CS_Deselect();
    MAG_CS_Deselect();
    HAL_Delay(10);

    if (IMU_Test_WhoAmI() != 0) {
        sprintf(uart_buf, "IMU WHO_AM_I FAILED!\r\n");
        CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
        HAL_Delay(10);
        return -1;
    }

    icm_write(ICM_REG_PWR_CTRL, 0x0E);
    HAL_Delay(5);

    icm_write(ICM_REG_PWR_CTRL, 0x07);
    HAL_Delay(2);

    icm_write(ICM_REG_ACC_CONF, 0xA8);
    icm_write(ICM_REG_ACC_RANGE, ACC_RANGE_16G);
    icm_write(ICM_REG_GYR_CONF, 0xA8);
    icm_write(ICM_REG_GYR_RANGE, GYR_RANGE_2000DPS);

    sprintf(uart_buf, "HXY ICM-42688P configured successfully\r\n");
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    uint8_t rst = MLX_CMD_RESET;
    MAG_CS_Select();
    HAL_SPI_Transmit(&hspi5, &rst, 1, 100);
    MAG_CS_Deselect();
    HAL_Delay(10);

    mlx_write_reg(MLX_REG_GAIN, 0x0007);
    mlx_write_reg(MLX_REG_RES, 0x001C);

    Mag_Calibrate_And_Check_Noise();

    return 0;
}

void System_Read_And_Process(void) {
    uint8_t raw[12];
    uint8_t temp_raw[2];

    if (icm_read_burst(ICM_REG_ACC_XH, raw, 12) == 0) {
        IMU_System.accel_x = (int16_t)(raw[0] << 8 | raw[1]);
        IMU_System.accel_y = (int16_t)(raw[2] << 8 | raw[3]);
        IMU_System.accel_z = (int16_t)(raw[4] << 8 | raw[5]);

        IMU_System.gyro_x = (int16_t)(raw[6] << 8 | raw[7]);
        IMU_System.gyro_y = (int16_t)(raw[8] << 8 | raw[9]);
        IMU_System.gyro_z = (int16_t)(raw[10] << 8 | raw[11]);
    }

    if (icm_read_burst(ICM_REG_TEMP_H, temp_raw, 2) == 0) {
        IMU_System.temp_raw = (int16_t)(temp_raw[0] << 8 | temp_raw[1]);
        IMU_System.temp_c = (IMU_System.temp_raw / 512.0f) + 23.0f;
    }

    float ax = IMU_System.accel_x / 2048.0f;
    float ay = IMU_System.accel_y / 2048.0f;
    float az = IMU_System.accel_z / 2048.0f;

    IMU_System.roll  = atan2f(ay, az) * 180.0f / M_PI;
    IMU_System.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;

    mlx_cmd(MLX_CMD_START_SINGLE, NULL, 0);
    IMU_DELAY(2);  /* FreeRTOS-aware delay for magnetometer conversion */
    uint8_t mraw[7];
    if (mlx_cmd(MLX_CMD_READ_MEAS, mraw, 7) == 0) {
        IMU_System.mag_x = (int16_t)(mraw[1]<<8 | mraw[2]) - IMU_System.mag_bias_x;
        IMU_System.mag_y = (int16_t)(mraw[3]<<8 | mraw[4]) - IMU_System.mag_bias_y;
        IMU_System.mag_z = (int16_t)(mraw[5]<<8 | mraw[6]) - IMU_System.mag_bias_z;

        Mag_Update_Noise(IMU_System.mag_x, IMU_System.mag_y, IMU_System.mag_z);
    }
}
void Mag_Update_Noise(int16_t raw_x, int16_t raw_y, int16_t raw_z) {
    // Store raw readings in circular buffers
    mag_x_buffer[buffer_index] = raw_x;
    mag_y_buffer[buffer_index] = raw_y;
    mag_z_buffer[buffer_index] = raw_z;
    
    // Increment and wrap buffer index
    buffer_index++;
    if (buffer_index >= MAG_NOISE_BUFFER_SIZE) {
        buffer_index = 0;
        buffers_filled = true;
    }
    
    // Return early if buffers aren't filled yet
    if (!buffers_filled) {
        return;
    }
    
    // Calculate means
    float mean_x = 0, mean_y = 0, mean_z = 0;
    for (uint16_t i = 0; i < MAG_NOISE_BUFFER_SIZE; i++) {
        mean_x += mag_x_buffer[i];
        mean_y += mag_y_buffer[i];
        mean_z += mag_z_buffer[i];
    }
    mean_x /= MAG_NOISE_BUFFER_SIZE;
    mean_y /= MAG_NOISE_BUFFER_SIZE;
    mean_z /= MAG_NOISE_BUFFER_SIZE;
    
    // Calculate variances
    float var_x = 0, var_y = 0, var_z = 0;
    for (uint16_t i = 0; i < MAG_NOISE_BUFFER_SIZE; i++) {
        float diff_x = mag_x_buffer[i] - mean_x;
        float diff_y = mag_y_buffer[i] - mean_y;
        float diff_z = mag_z_buffer[i] - mean_z;
        var_x += diff_x * diff_x;
        var_y += diff_y * diff_y;
        var_z += diff_z * diff_z;
    }
    var_x /= MAG_NOISE_BUFFER_SIZE;
    var_y /= MAG_NOISE_BUFFER_SIZE;
    var_z /= MAG_NOISE_BUFFER_SIZE;
    
    // Calculate combined standard deviation (RMS of individual std devs)
    float std_dev = sqrtf((var_x + var_y + var_z) / 3.0f);
    
    // Update IMU_System.mag_noise
    IMU_System.mag_noise = std_dev;
}