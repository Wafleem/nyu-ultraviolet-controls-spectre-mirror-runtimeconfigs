#ifndef CORE_INC_IMU_H
#define CORE_INC_IMU_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// ==========================================
// HXY ICM-42688P Clone - register definitions
// ==========================================

// --- Identification ---
#define ICM_REG_WHO_AM_I 0x01 // NOT 0x75!
#define ICM_WHO_AM_I_VAL 0x6A // NOT 0x47!

// --- Configuration Registers ---
#define ICM_REG_OIS_CONF 0x04
#define ICM_REG_COM_CFG 0x05
#define ICM_REG_INT_CFG1 0x06
#define ICM_REG_INT_CFG2 0x07
#define ICM_REG_HPF_LPF_CFG 0x08 // NOT 0x51!

// --- Data Registers ---
#define ICM_REG_DATA_STAT 0x0B
#define ICM_REG_ACC_XH 0x0C // NOT 0x1F!
#define ICM_REG_ACC_XL 0x0D
#define ICM_REG_ACC_YH 0x0E
#define ICM_REG_ACC_YL 0x0F
#define ICM_REG_ACC_ZH 0x10
#define ICM_REG_ACC_ZL 0x11
#define ICM_REG_GYR_XH 0x12
#define ICM_REG_GYR_XL 0x13
#define ICM_REG_GYR_YH 0x14
#define ICM_REG_GYR_YL 0x15
#define ICM_REG_GYR_ZH 0x16
#define ICM_REG_GYR_ZL 0x17
#define ICM_REG_TEMP_H 0x22 // NOT 0x1D!
#define ICM_REG_TEMP_L 0x23

// --- Sensor Configuration ---
#define ICM_REG_ACC_CONF 0x40  // NOT 0x50!
#define ICM_REG_ACC_RANGE 0x41 // Separate register!
#define ICM_REG_GYR_CONF 0x42  // NOT 0x4F!
#define ICM_REG_GYR_RANGE 0x43 // Separate register!

// --- Power Control ---
#define ICM_REG_SOFT_RST 0x4A
#define ICM_REG_PWR_CTRL 0x7D // NOT 0x4E!

// --- Range Values ---
#define ACC_RANGE_2G 0x00
#define ACC_RANGE_4G 0x01
#define ACC_RANGE_8G 0x02
#define ACC_RANGE_16G 0x03

#define GYR_RANGE_2000DPS 0x00
#define GYR_RANGE_1000DPS 0x01
#define GYR_RANGE_500DPS 0x02
#define GYR_RANGE_250DPS 0x03
#define GYR_RANGE_125DPS 0x04

// --- MLX90393 (SPI5) Commands ---
#define MLX_CMD_RESET 0xF0
#define MLX_CMD_START_SINGLE 0x3E
#define MLX_CMD_READ_MEAS 0x4E
#define MLX_CMD_WRITE_REG 0x60
#define MLX_REG_GAIN 0x00
#define MLX_REG_RES 0x02

// --- PIN DEFINITIONS (fallback if not defined by CubeMX) ---
#ifndef IMU_CS_GPIO_Port
#define IMU_CS_GPIO_Port GPIOI
#define IMU_CS_Pin GPIO_PIN_0
#endif

#ifndef MAG_CS_GPIO_Port
#define MAG_CS_GPIO_Port GPIOK
#define MAG_CS_Pin GPIO_PIN_1
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Forward declarations for HAL handles used by IMU module (defined in CubeMX
// generated files)
typedef struct __SPI_HandleTypeDef
    SPI_HandleTypeDef; // opaque here; include spi.h in implementation
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi5;

// Public IMU data structure
typedef struct {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  int16_t temp_raw;

  float temp_c;
  float roll;
  float pitch;
  float ekf_roll;
  float ekf_pitch;
  float ekf_yaw;

  int16_t mag_bias_x;
  int16_t mag_bias_y;
  int16_t mag_bias_z;
  float mag_noise;
} Gimbal_Sensor_Data_t;

// Global instance (defined in imu.c)
extern Gimbal_Sensor_Data_t Gimbal_Sensor;
extern volatile uint8_t imu_initialized;

// Expose the original API names used by main.c
void IMU_Read(Gimbal_Sensor_Data_t *sensor);
int8_t IMU_Test_WhoAmI(void);
void IMU_Calibrate(void);
void Mag_Save_Biases_To_EPROM(void);
void Mag_Calibrate_And_Check_Noise(void);
int8_t System_Sensors_Init(void);
void System_Read_And_Process(void);
void IMU_ResetYawToZero(void);
void Mag_Update_Noise(int16_t raw_x, int16_t raw_y, int16_t raw_z);

#endif // CORE_INC_IMU_H
