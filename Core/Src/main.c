/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - CAN Testing + IMU Sensors
  *                   CORRECTED FOR HXY ICM-42688P CLONE
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// IMU System Data Structure
typedef struct {
    // Raw Data
    int16_t accel_x; int16_t accel_y; int16_t accel_z;
    int16_t gyro_x;  int16_t gyro_y;  int16_t gyro_z;
    int16_t mag_x;   int16_t mag_y;   int16_t mag_z;
    int16_t temp_raw;

    // Processed Data
    float temp_c;
    float roll;
    float pitch;

    // Mag Calibration
    int16_t mag_bias_x;
    int16_t mag_bias_y;
    int16_t mag_bias_z;
    float mag_noise;
} IMU_System_Data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// =====================================================
// HXY ICM-42688P Clone - CORRECTED Register Definitions
// =====================================================

// --- Identification ---
#define ICM_REG_WHO_AM_I      0x01    // NOT 0x75!
#define ICM_WHO_AM_I_VAL      0x6A    // NOT 0x47!

// --- Configuration Registers ---
#define ICM_REG_OIS_CONF      0x04
#define ICM_REG_COM_CFG       0x05
#define ICM_REG_INT_CFG1      0x06
#define ICM_REG_INT_CFG2      0x07
#define ICM_REG_HPF_LPF_CFG   0x08    // NOT 0x51!

// --- Data Registers ---
#define ICM_REG_DATA_STAT     0x0B
#define ICM_REG_ACC_XH        0x0C    // NOT 0x1F!
#define ICM_REG_ACC_XL        0x0D
#define ICM_REG_ACC_YH        0x0E
#define ICM_REG_ACC_YL        0x0F
#define ICM_REG_ACC_ZH        0x10
#define ICM_REG_ACC_ZL        0x11
#define ICM_REG_GYR_XH        0x12
#define ICM_REG_GYR_XL        0x13
#define ICM_REG_GYR_YH        0x14
#define ICM_REG_GYR_YL        0x15
#define ICM_REG_GYR_ZH        0x16
#define ICM_REG_GYR_ZL        0x17
#define ICM_REG_TEMP_H        0x22    // NOT 0x1D!
#define ICM_REG_TEMP_L        0x23

// --- Sensor Configuration ---
#define ICM_REG_ACC_CONF      0x40    // NOT 0x50!
#define ICM_REG_ACC_RANGE     0x41    // Separate register!
#define ICM_REG_GYR_CONF      0x42    // NOT 0x4F!
#define ICM_REG_GYR_RANGE     0x43    // Separate register!

// --- Power Control ---
#define ICM_REG_SOFT_RST      0x4A
#define ICM_REG_PWR_CTRL      0x7D    // NOT 0x4E!

// --- Range Values ---
#define ACC_RANGE_2G          0x00
#define ACC_RANGE_4G          0x01
#define ACC_RANGE_8G          0x02
#define ACC_RANGE_16G         0x03

#define GYR_RANGE_2000DPS     0x00
#define GYR_RANGE_1000DPS     0x01
#define GYR_RANGE_500DPS      0x02
#define GYR_RANGE_250DPS      0x03
#define GYR_RANGE_125DPS      0x04

// --- MLX90393 (SPI5) Commands (unchanged) ---
#define MLX_CMD_RESET         0xF0
#define MLX_CMD_START_SINGLE  0x3E
#define MLX_CMD_READ_MEAS     0x4E
#define MLX_CMD_WRITE_REG     0x60
#define MLX_REG_GAIN          0x00
#define MLX_REG_RES           0x02

// --- PIN DEFINITIONS (fallback if not defined by CubeMX) ---
#ifndef IMU_CS_GPIO_Port
#define IMU_CS_GPIO_Port GPIOI
#define IMU_CS_Pin       GPIO_PIN_0
#endif

#ifndef MAG_CS_GPIO_Port
#define MAG_CS_GPIO_Port GPIOK
#define MAG_CS_Pin       GPIO_PIN_1
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// CAN Variables
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
volatile uint8_t can_rx_flag = 0;
volatile uint32_t received_number = 0;
char uart_buf[256];

// IMU Variables
IMU_System_Data_t IMU_System;
uint8_t imu_initialized = 0;

// SPI Handles (Defined in spi.c by CubeMX)
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi5;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Config(void);

// IMU Function Prototypes
int8_t System_Sensors_Init(void);
void System_Read_And_Process(void);
int8_t IMU_Test_WhoAmI(void);
void Mag_Calibrate_And_Check_Noise(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==========================================
//       CAN CONFIGURATION
// ==========================================

void CAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    //Configure FDCAN2 filter to accept all standard ID messages into FIFO0
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    //Reject non-matching frames on FDCAN2
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    //RX interrupt on FDCAN2 for receiver
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    //Start CAN peripherals
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    //TX header for FDCAN1
    TxHeader.Identifier = 0x123;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        if (hfdcan->Instance == FDCAN2)
        {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
            {
                received_number = (RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3];
                can_rx_flag = 1;
            }
        }
    }
}

// ==========================================
//       LOW LEVEL SPI DRIVERS
// ==========================================

// --- HXY ICM-42688P (SPI2) ---
static inline void IMU_CS_Select(void)   { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); }
static inline void IMU_CS_Deselect(void) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); }

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

// --- MLX90393 (SPI5) ---
static inline void MAG_CS_Select(void)   { HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET); }
static inline void MAG_CS_Deselect(void) { HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET); }

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

// ==========================================
//       IMU HIGH LEVEL LOGIC (HXY CORRECTED)
// ==========================================

int8_t IMU_Test_WhoAmI(void) {
    uint8_t whoami = 0;
    icm_read(ICM_REG_WHO_AM_I, &whoami);  // Read from 0x01

    //Shows the result (one time)
    sprintf(uart_buf, "WHO_AM_I: Read 0x%02X, Expected 0x%02X\r\n", whoami, ICM_WHO_AM_I_VAL);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    if (whoami == ICM_WHO_AM_I_VAL) return 0;  // Should be 0x6A
    return -1;
}

void Mag_Save_Biases_To_EPROM(void) {
    // Placeholder for Flash logic
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

    float var_x = 0;
    for(int i=0; i<50; i++) {
        float diff = readings_x[i] - IMU_System.mag_bias_x;
        var_x += diff * diff;
    }
    IMU_System.mag_noise = sqrtf(var_x / 50.0f);
}

int8_t System_Sensors_Init(void) {
    IMU_CS_Deselect();
    MAG_CS_Deselect();
    HAL_Delay(10);

    // =========================================
    // HXY ICM-42688P Init (CORRECTED SEQUENCE)
    // =========================================

    //WHO_AM_I
    if (IMU_Test_WhoAmI() != 0) {
        sprintf(uart_buf, "IMU WHO_AM_I FAILED!\r\n");
        CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
        HAL_Delay(10);
        return -1;
    }

    //Write 0x0E to PWR_CTRL first, then delay (per HXY datasheet)
    icm_write(ICM_REG_PWR_CTRL, 0x0E);
    HAL_Delay(5);  // Required delay after initial power write

    //Enable all sensors: TEMP_EN (B2) + ACC_EN (B1) + GYR_EN (B0) = 0x07
    icm_write(ICM_REG_PWR_CTRL, 0x07);
    HAL_Delay(2);

    //Configure accelerometer
    //ACC_CONF (0x40): High performance mode with ODR
    icm_write(ICM_REG_ACC_CONF, 0xA8);
    //ACC_RANGE (0x41): ±16g (0x03)
    icm_write(ICM_REG_ACC_RANGE, ACC_RANGE_16G);

    //Configure gyroscope
    //GYR_CONF (0x42): High performance mode with ODR
    icm_write(ICM_REG_GYR_CONF, 0xA8);
    //GYR_RANGE (0x43): ±2000dps (0x00)
    icm_write(ICM_REG_GYR_RANGE, GYR_RANGE_2000DPS);


    sprintf(uart_buf, "HXY ICM-42688P configured successfully\r\n");
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    // =========================================
    // MLX90393 Magnetometer Init
    // =========================================
    uint8_t rst = MLX_CMD_RESET;
    MAG_CS_Select();
    HAL_SPI_Transmit(&hspi5, &rst, 1, 100);
    MAG_CS_Deselect();
    HAL_Delay(10);

    mlx_write_reg(MLX_REG_GAIN, 0x0007);    // Gain setting
    mlx_write_reg(MLX_REG_RES, 0x001C);     // Resolution setting

    Mag_Calibrate_And_Check_Noise();

    return 0;
}

void System_Read_And_Process(void) {
    uint8_t raw[12];
    uint8_t temp_raw[2];

    // =========================================
    // Read HXY ICM-42688P (CORRECTED ADDRESSES)
    // =========================================

    // Read Accel + Gyro (12 bytes starting at 0x0C)
    if (icm_read_burst(ICM_REG_ACC_XH, raw, 12) == 0) {
        // Accelerometer data (0x0C - 0x11)
        IMU_System.accel_x = (int16_t)(raw[0] << 8 | raw[1]);
        IMU_System.accel_y = (int16_t)(raw[2] << 8 | raw[3]);
        IMU_System.accel_z = (int16_t)(raw[4] << 8 | raw[5]);

        // Gyroscope data (0x12 - 0x17)
        IMU_System.gyro_x = (int16_t)(raw[6] << 8 | raw[7]);
        IMU_System.gyro_y = (int16_t)(raw[8] << 8 | raw[9]);
        IMU_System.gyro_z = (int16_t)(raw[10] << 8 | raw[11]);
    }

    // Read Temperature separately (2 bytes at 0x22)
    if (icm_read_burst(ICM_REG_TEMP_H, temp_raw, 2) == 0) {
        IMU_System.temp_raw = (int16_t)(temp_raw[0] << 8 | temp_raw[1]);
        // HXY Formula: T = raw/512 + 23  (NOT raw/132.48 + 25!)
        IMU_System.temp_c = (IMU_System.temp_raw / 512.0f) + 23.0f;
    }

    // Calculate Roll & Pitch from accelerometer (unchanged)
    // Scale factor for ±16g is 2048 LSB/g
    float ax = IMU_System.accel_x / 2048.0f;
    float ay = IMU_System.accel_y / 2048.0f;
    float az = IMU_System.accel_z / 2048.0f;

    IMU_System.roll  = atan2f(ay, az) * 180.0f / M_PI;
    IMU_System.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;

    // =========================================
    // Read MLX90393 Magnetometer (unchanged)
    // =========================================
    mlx_cmd(MLX_CMD_START_SINGLE, NULL, 0);
    HAL_Delay(2);
    uint8_t mraw[7];
    if (mlx_cmd(MLX_CMD_READ_MEAS, mraw, 7) == 0) {
        IMU_System.mag_x = (int16_t)(mraw[1]<<8 | mraw[2]) - IMU_System.mag_bias_x;
        IMU_System.mag_y = (int16_t)(mraw[3]<<8 | mraw[4]) - IMU_System.mag_bias_y;
        IMU_System.mag_z = (int16_t)(mraw[5]<<8 | mraw[6]) - IMU_System.mag_bias_z;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_UART7_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();

  // Wait for USB to enumerate
  HAL_Delay(1000);

  // Configure and start CAN
  CAN_Config();

  // Initialize IMU Sensors
  sprintf(uart_buf, "\r\n===== SYSTEM STARTUP =====\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  sprintf(uart_buf, "Initializing HXY ICM-42688P + MLX90393...\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  if (System_Sensors_Init() == 0) {
      imu_initialized = 1;
      sprintf(uart_buf, "IMU Init: SUCCESS\r\n");
      CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  } else {
      imu_initialized = 0;
      sprintf(uart_buf, "IMU Init: FAILED! Check SPI connections.\r\n");
      CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  }
  HAL_Delay(10);

  sprintf(uart_buf, "CAN configured. Starting main loop...\r\n\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  uint32_t sent_number = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // ==========================================
      //       CAN LOOPBACK TEST
      // ==========================================

      // Pack sent_number into TX data
      TxData[0] = (sent_number >> 24) & 0xFF;
      TxData[1] = (sent_number >> 16) & 0xFF;
      TxData[2] = (sent_number >> 8) & 0xFF;
      TxData[3] = sent_number & 0xFF;
      TxData[4] = 0x00;
      TxData[5] = 0x00;
      TxData[6] = 0x00;
      TxData[7] = 0x00;

      sprintf(uart_buf, "========== LOOP %lu ==========\r\n", sent_number);
      CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
      HAL_Delay(10);

      // CAN TX from FDCAN1
      sprintf(uart_buf, "----- CAN TEST -----\r\n");
      CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
      HAL_Delay(10);

      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) == HAL_OK)
      {
          sprintf(uart_buf, "CAN1 TX: Sent %lu\r\n", sent_number);
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
      }
      else
      {
          sprintf(uart_buf, "CAN1 TX: ERROR!\r\n");
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
      }
      HAL_Delay(10);

      // Check CAN RX on FDCAN2
      if (can_rx_flag)
      {
          can_rx_flag = 0;
          sprintf(uart_buf, "CAN2 RX: Received %lu\r\n", received_number);
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
      }
      else
      {
          sprintf(uart_buf, "CAN2 RX: TIMEOUT!\r\n");
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
      }
      HAL_Delay(10);

      // ==========================================
      //       IMU DATA READOUT
      // ==========================================

      sprintf(uart_buf, "----- IMU DATA -----\r\n");
      CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
      HAL_Delay(10);

      if (imu_initialized) {
          // Read and process sensor data
          System_Read_And_Process();

          // Accelerometer (raw values)
          sprintf(uart_buf, "Accel: X=%d Y=%d Z=%d\r\n",
                  IMU_System.accel_x, IMU_System.accel_y, IMU_System.accel_z);
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
          HAL_Delay(10);

          // Gyroscope (raw values)
          sprintf(uart_buf, "Gyro:  X=%d Y=%d Z=%d\r\n",
                  IMU_System.gyro_x, IMU_System.gyro_y, IMU_System.gyro_z);
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
          HAL_Delay(10);

          // Magnetometer (bias-corrected)
          sprintf(uart_buf, "Mag:   X=%d Y=%d Z=%d\r\n",
                  IMU_System.mag_x, IMU_System.mag_y, IMU_System.mag_z);
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
          HAL_Delay(10);

          // Temperature (HXY formula: raw/512 + 23)
          // Convert to integer: multiply by 10 for one decimal place
          int16_t temp_int = (int16_t)(IMU_System.temp_c * 10);
          sprintf(uart_buf, "Temp:  %d.%d C\r\n", temp_int / 10, abs(temp_int % 10));
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
          HAL_Delay(10);

          // Roll & Pitch (calculated from accel)
          // Convert to integer: multiply by 10 for one decimal place
          int16_t roll_int = (int16_t)(IMU_System.roll * 10);
          int16_t pitch_int = (int16_t)(IMU_System.pitch * 10);
          sprintf(uart_buf, "Roll: %s%d.%d deg | Pitch: %s%d.%d deg\r\n",
                  (roll_int < 0) ? "-" : "", abs(roll_int) / 10, abs(roll_int % 10),
                  (pitch_int < 0) ? "-" : "", abs(pitch_int) / 10, abs(pitch_int % 10));
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
          HAL_Delay(10);

      } else {
          sprintf(uart_buf, "IMU: Not initialized!\r\n");
          CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
          HAL_Delay(10);
      }

      sprintf(uart_buf, "\r\n");
      CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));

      sent_number++;
      HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB2;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
