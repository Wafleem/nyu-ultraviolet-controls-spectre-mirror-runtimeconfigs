#include "imu.h"
#include "QuaternionEKF.h"
#include "gpio.h"
#include "spi.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* FreeRTOS-aware delay macro */
#ifdef USE_FREERTOS
#include "cmsis_os.h"
#define IMU_DELAY(ms) osDelay(ms)
#else
#define IMU_DELAY(ms) HAL_Delay(ms)
#endif

Gimbal_Sensor_Data_t Gimbal_Sensor;
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

/* IMU scale factors for current sensor config: accel=±16g, gyro=±2000dps */
#define IMU_ACCEL_LSB_PER_G 2048.0f
#define IMU_GYRO_LSB_PER_DPS 16.4f
#define IMU_GRAVITY_MPS2 9.80665f
/* Sensor mounting compensation: +45 means sensor frame is CW-rotated 45deg vs
 * body */
#define IMU_MOUNT_YAW_DEG 45.0f
/* Magnetometer yaw assist gain (0.0~1.0), lower is smoother */
#define IMU_MAG_YAW_BLEND 0.005f
#define IMU_MAG_YAW_MAX_STEP_DEG 0.3f
#define IMU_MAG_YAW_LPF_ALPHA 0.10f
#define IMU_MAG_ASSIST_MAX_GYRO_DPS 120.0f
#define IMU_MAG_ASSIST_MAX_NOISE 120.0f
#define IMU_EKF_OUTPUT_ALPHA 0.12f

/* Online gyro temperature-drift compensation */
#define IMU_TEMP_COMP_STABLE_GYRO_DPS 2.5f
#define IMU_TEMP_COMP_STABLE_ACCEL_G_MIN 0.90f
#define IMU_TEMP_COMP_STABLE_ACCEL_G_MAX 1.10f
#define IMU_TEMP_COMP_BIAS_LR 0.01f
#define IMU_TEMP_COMP_COEFF_LR 0.0005f
#define IMU_TEMP_COMP_COEFF_MAX 50.0f

typedef struct {
  uint8_t initialized;
  float ref_temp_c;
  float bias_lsb[3];
  float coeff_lsb_per_c[3];
} GyroTempCompState_t;

static GyroTempCompState_t s_gyro_temp_comp = {0};
static uint32_t s_last_update_tick = 0;
static uint8_t s_ekf_output_initialized = 0;
static float s_ekf_roll_filtered = 0.0f;
static float s_ekf_pitch_filtered = 0.0f;
static float s_ekf_yaw_filtered = 0.0f;
static float s_ekf_yaw_reset_offset_deg = 0.0f;

static void IMU_Init_QuaternionEKF(void);
static void IMU_ApplyMountRotationXY(float *x, float *y);
static float IMU_WrapAngleDeg(float angle_deg);
static float IMU_AngleLerpDeg(float current_deg, float target_deg, float alpha);
static void IMU_ApplyGyroTempComp(float temp_c, float accel_norm_g,
                                  float *gx_lsb, float *gy_lsb, float *gz_lsb);

static float IMU_WrapAngleDeg(float angle_deg) {
  while (angle_deg > 180.0f)
    angle_deg -= 360.0f;
  while (angle_deg < -180.0f)
    angle_deg += 360.0f;
  return angle_deg;
}

static float IMU_AngleLerpDeg(float current_deg, float target_deg,
                              float alpha) {
  float err = IMU_WrapAngleDeg(target_deg - current_deg);
  return IMU_WrapAngleDeg(current_deg + alpha * err);
}

static void IMU_ApplyGyroTempComp(float temp_c, float accel_norm_g,
                                  float *gx_lsb, float *gy_lsb, float *gz_lsb) {
  float gyro_lsb[3] = {*gx_lsb, *gy_lsb, *gz_lsb};

  if (!s_gyro_temp_comp.initialized) {
    s_gyro_temp_comp.initialized = 1;
    s_gyro_temp_comp.ref_temp_c = temp_c;
    s_gyro_temp_comp.bias_lsb[0] = gyro_lsb[0];
    s_gyro_temp_comp.bias_lsb[1] = gyro_lsb[1];
    s_gyro_temp_comp.bias_lsb[2] = gyro_lsb[2];
    s_gyro_temp_comp.coeff_lsb_per_c[0] = 0.0f;
    s_gyro_temp_comp.coeff_lsb_per_c[1] = 0.0f;
    s_gyro_temp_comp.coeff_lsb_per_c[2] = 0.0f;
  }

  float dtemp = temp_c - s_gyro_temp_comp.ref_temp_c;
  float bias_est[3];
  float gyro_comp[3];
  for (uint8_t i = 0; i < 3; i++) {
    bias_est[i] = s_gyro_temp_comp.bias_lsb[i] +
                  s_gyro_temp_comp.coeff_lsb_per_c[i] * dtemp;
    gyro_comp[i] = gyro_lsb[i] - bias_est[i];
  }

  float gyro_norm_dps =
      sqrtf(gyro_comp[0] * gyro_comp[0] + gyro_comp[1] * gyro_comp[1] +
            gyro_comp[2] * gyro_comp[2]) /
      IMU_GYRO_LSB_PER_DPS;
  uint8_t stable = (gyro_norm_dps < IMU_TEMP_COMP_STABLE_GYRO_DPS) &&
                   (accel_norm_g > IMU_TEMP_COMP_STABLE_ACCEL_G_MIN) &&
                   (accel_norm_g < IMU_TEMP_COMP_STABLE_ACCEL_G_MAX);

  if (stable) {
    float denom = dtemp * dtemp + 1.0f;
    for (uint8_t i = 0; i < 3; i++) {
      float err = gyro_comp[i];
      s_gyro_temp_comp.bias_lsb[i] += IMU_TEMP_COMP_BIAS_LR * err;
      s_gyro_temp_comp.coeff_lsb_per_c[i] +=
          IMU_TEMP_COMP_COEFF_LR * err * dtemp / denom;

      if (s_gyro_temp_comp.coeff_lsb_per_c[i] > IMU_TEMP_COMP_COEFF_MAX) {
        s_gyro_temp_comp.coeff_lsb_per_c[i] = IMU_TEMP_COMP_COEFF_MAX;
      }
      if (s_gyro_temp_comp.coeff_lsb_per_c[i] < -IMU_TEMP_COMP_COEFF_MAX) {
        s_gyro_temp_comp.coeff_lsb_per_c[i] = -IMU_TEMP_COMP_COEFF_MAX;
      }

      bias_est[i] = s_gyro_temp_comp.bias_lsb[i] +
                    s_gyro_temp_comp.coeff_lsb_per_c[i] * dtemp;
      gyro_comp[i] = gyro_lsb[i] - bias_est[i];
    }
  }

  *gx_lsb = gyro_comp[0];
  *gy_lsb = gyro_comp[1];
  *gz_lsb = gyro_comp[2];
}

static void IMU_ApplyMountRotationXY(float *x, float *y) {
  const float theta = IMU_MOUNT_YAW_DEG * (M_PI / 180.0f);
  const float c = cosf(theta);
  const float s = sinf(theta);
  const float x_in = *x;
  const float y_in = *y;

  *x = c * x_in - s * y_in;
  *y = s * x_in + c * y_in;
}

// Low-level SPI / chip-select helpers (use CubeMX-generated SPI handles)
static inline void IMU_CS_Select(void) {
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}
static inline void IMU_CS_Deselect(void) {
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}
static inline void MAG_CS_Select(void) {
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
}
static inline void MAG_CS_Deselect(void) {
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
}

static int8_t icm_write(uint8_t reg, uint8_t val) {
  uint8_t data[2] = {reg & 0x7F, val}; // Clear MSB for write
  IMU_CS_Select();
  if (HAL_SPI_Transmit(&hspi2, data, 2, 100) != HAL_OK) {
    IMU_CS_Deselect();
    return -1;
  }
  IMU_CS_Deselect();
  return 0;
}

static int8_t icm_read(uint8_t reg, uint8_t *val) {
  uint8_t tx = reg | 0x80; // Set MSB for read
  IMU_CS_Select();
  if (HAL_SPI_Transmit(&hspi2, &tx, 1, 100) != HAL_OK) {
    IMU_CS_Deselect();
    return -1;
  }
  if (HAL_SPI_Receive(&hspi2, val, 1, 100) != HAL_OK) {
    IMU_CS_Deselect();
    return -1;
  }
  IMU_CS_Deselect();
  return 0;
}

static int8_t icm_read_burst(uint8_t start_reg, uint8_t *buf, uint16_t len) {
  uint8_t tx = start_reg | 0x80; // Set MSB for read
  IMU_CS_Select();
  if (HAL_SPI_Transmit(&hspi2, &tx, 1, 100) != HAL_OK) {
    IMU_CS_Deselect();
    return -1;
  }
  if (HAL_SPI_Receive(&hspi2, buf, len, 100) != HAL_OK) {
    IMU_CS_Deselect();
    return -1;
  }
  IMU_CS_Deselect();
  return 0;
}

static int8_t mlx_cmd(uint8_t cmd, uint8_t *rx, uint16_t len) {
  uint8_t tx = cmd;
  MAG_CS_Select();
  if (HAL_SPI_Transmit(&hspi5, &tx, 1, 100) != HAL_OK) {
    MAG_CS_Deselect();
    return -1;
  }
  if (rx && len > 0) {
    HAL_SPI_Receive(&hspi5, rx, len, 100);
  }
  MAG_CS_Deselect();
  return 0;
}

static int8_t mlx_write_reg(uint8_t reg, uint16_t val) {
  uint8_t tx[4] = {MLX_CMD_WRITE_REG, (uint8_t)((reg << 2) & 0xFF),
                   (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
  MAG_CS_Select();
  if (HAL_SPI_Transmit(&hspi5, tx, 4, 100) != HAL_OK) {
    MAG_CS_Deselect();
    return -1;
  }
  MAG_CS_Deselect();
  return 0;
}

int8_t IMU_Test_WhoAmI(void) {
  uint8_t whoami = 0;
  icm_read(ICM_REG_WHO_AM_I, &whoami); // Read from 0x01

  sprintf(uart_buf, "WHO_AM_I: Read 0x%02X, Expected 0x%02X\r\n", whoami,
          ICM_WHO_AM_I_VAL);
  CDC_Transmit_FS((uint8_t *)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  if (whoami == ICM_WHO_AM_I_VAL)
    return 0; // Should be 0x6A
  return -1;
}

void Mag_Save_Biases_To_EPROM(void) {
  // Placeholder for Flash logic — implement if persistent storage is required
}

void Mag_Calibrate_And_Check_Noise(void) {
  int32_t sum_x = 0, sum_y = 0, sum_z = 0;
  int16_t readings_x[50], readings_y[50], readings_z[50];
  uint8_t mraw[7];

  for (int i = 0; i < 50; i++) {
    mlx_cmd(MLX_CMD_START_SINGLE, NULL, 0);
    HAL_Delay(10);
    mlx_cmd(MLX_CMD_READ_MEAS, mraw, 7);

    readings_x[i] = (int16_t)(mraw[1] << 8 | mraw[2]);
    readings_y[i] = (int16_t)(mraw[3] << 8 | mraw[4]);
    readings_z[i] = (int16_t)(mraw[5] << 8 | mraw[6]);

    sum_x += readings_x[i];
    sum_y += readings_y[i];
    sum_z += readings_z[i];
  }

  Gimbal_Sensor.mag_bias_x = sum_x / 50;
  Gimbal_Sensor.mag_bias_y = sum_y / 50;
  Gimbal_Sensor.mag_bias_z = sum_z / 50;

  Mag_Save_Biases_To_EPROM();

  float var_x = 0, var_y = 0, var_z = 0;
  for (int i = 0; i < 50; i++) {
    float diff_x = readings_x[i] - Gimbal_Sensor.mag_bias_x;
    float diff_y = readings_y[i] - Gimbal_Sensor.mag_bias_y;
    float diff_z = readings_z[i] - Gimbal_Sensor.mag_bias_z;
    var_x += diff_x * diff_x;
    var_y += diff_y * diff_y;
    var_z += diff_z * diff_z;
  }
  var_x /= 50.0f;
  var_y /= 50.0f;
  var_z /= 50.0f;
  Gimbal_Sensor.mag_noise = sqrtf((var_x + var_y + var_z) / 3.0f);
}

int8_t System_Sensors_Init(void) {
  IMU_CS_Deselect();
  MAG_CS_Deselect();
  HAL_Delay(10);

  if (IMU_Test_WhoAmI() != 0) {
    sprintf(uart_buf, "IMU WHO_AM_I FAILED!\r\n");
    CDC_Transmit_FS((uint8_t *)uart_buf, strlen(uart_buf));
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
  CDC_Transmit_FS((uint8_t *)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  uint8_t rst = MLX_CMD_RESET;
  MAG_CS_Select();
  HAL_SPI_Transmit(&hspi5, &rst, 1, 100);
  MAG_CS_Deselect();
  HAL_Delay(10);

  mlx_write_reg(MLX_REG_GAIN, 0x0007);
  mlx_write_reg(MLX_REG_RES, 0x001C);

  Mag_Calibrate_And_Check_Noise();

  IMU_Init_QuaternionEKF();

  return 0;
}

static void IMU_Init_QuaternionEKF(void) {
  float init_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  uint8_t raw[6];

  if (icm_read_burst(ICM_REG_ACC_XH, raw, 6) == 0) {
    float ax = ((int16_t)(raw[0] << 8 | raw[1])) / IMU_ACCEL_LSB_PER_G;
    float ay = ((int16_t)(raw[2] << 8 | raw[3])) / IMU_ACCEL_LSB_PER_G;
    float az = ((int16_t)(raw[4] << 8 | raw[5])) / IMU_ACCEL_LSB_PER_G;

    IMU_ApplyMountRotationXY(&ax, &ay);

    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    init_q[0] = cr * cp;
    init_q[1] = sr * cp;
    init_q[2] = cr * sp;
    init_q[3] = -sr * sp;
  }

  IMU_QuaternionEKF_Init(init_q, 10.0f, 0.001f, 1000000.0f, 1.0f, 0.0f);

  Gimbal_Sensor.ekf_roll = 0.0f;
  Gimbal_Sensor.ekf_pitch = 0.0f;
  Gimbal_Sensor.ekf_yaw = 0.0f;
}

void IMU_ResetYawToZero(void) {
  /* Keep current roll/pitch from EKF and only re-zero the reported yaw. */
  s_ekf_yaw_reset_offset_deg = IMU_WrapAngleDeg(-QEKF_INS.Yaw);

  if (s_ekf_output_initialized) {
    s_ekf_yaw_filtered = 0.0f;
  }

  Gimbal_Sensor.ekf_yaw = 0.0f;
}

void System_Read_And_Process(void) {
  // static uint8_t mag_yaw_initialized = 0;
  // static float mag_yaw_filtered_deg = 0.0f;
  uint8_t raw[12];
  uint8_t temp_raw[2];

  if (icm_read_burst(ICM_REG_ACC_XH, raw, 12) == 0) {
    float accel_x = (float)((int16_t)(raw[0] << 8 | raw[1]));
    float accel_y = (float)((int16_t)(raw[2] << 8 | raw[3]));
    float accel_z = (float)((int16_t)(raw[4] << 8 | raw[5]));

    float gyro_x = (float)((int16_t)(raw[6] << 8 | raw[7]));
    float gyro_y = (float)((int16_t)(raw[8] << 8 | raw[9]));
    float gyro_z = (float)((int16_t)(raw[10] << 8 | raw[11]));

    IMU_ApplyMountRotationXY(&accel_x, &accel_y);
    IMU_ApplyMountRotationXY(&gyro_x, &gyro_y);

    Gimbal_Sensor.accel_x = (int16_t)accel_x;
    Gimbal_Sensor.accel_y = (int16_t)accel_y;
    Gimbal_Sensor.accel_z = (int16_t)accel_z;

    Gimbal_Sensor.gyro_x = (int16_t)gyro_x;
    Gimbal_Sensor.gyro_y = (int16_t)gyro_y;
    Gimbal_Sensor.gyro_z = (int16_t)gyro_z;
  }

  if (icm_read_burst(ICM_REG_TEMP_H, temp_raw, 2) == 0) {
    Gimbal_Sensor.temp_raw = (int16_t)(temp_raw[0] << 8 | temp_raw[1]);
    Gimbal_Sensor.temp_c = (Gimbal_Sensor.temp_raw / 512.0f) + 23.0f;
  }

  float ax = Gimbal_Sensor.accel_x / 2048.0f;
  float ay = Gimbal_Sensor.accel_y / 2048.0f;
  float az = Gimbal_Sensor.accel_z / 2048.0f;
  float accel_norm_g = sqrtf(ax * ax + ay * ay + az * az);
  // float gyro_norm_dps = 0.0f;

  // Deadband gyroscope
  if (fabsf(Gimbal_Sensor.gyro_x) <= 1) Gimbal_Sensor.gyro_x = 0;
  if (fabsf(Gimbal_Sensor.gyro_y) <= 1) Gimbal_Sensor.gyro_y = 0;
  if (fabsf(Gimbal_Sensor.gyro_z) <= 1) Gimbal_Sensor.gyro_z = 0;

  {
    float gx_lsb = (float)Gimbal_Sensor.gyro_x;
    float gy_lsb = (float)Gimbal_Sensor.gyro_y;
    float gz_lsb = (float)Gimbal_Sensor.gyro_z;

    IMU_ApplyGyroTempComp(Gimbal_Sensor.temp_c, accel_norm_g, &gx_lsb, &gy_lsb,
                          &gz_lsb);

    Gimbal_Sensor.gyro_x = (int16_t)gx_lsb;
    Gimbal_Sensor.gyro_y = (int16_t)gy_lsb;
    Gimbal_Sensor.gyro_z = (int16_t)gz_lsb;

    // gyro_norm_dps = sqrtf(gx_lsb * gx_lsb + gy_lsb * gy_lsb + gz_lsb *
    // gz_lsb) /
    //                 IMU_GYRO_LSB_PER_DPS;
  }

  Gimbal_Sensor.roll = atan2f(ay, az) * 180.0f / M_PI;
  Gimbal_Sensor.pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

  {
    float gx_rad =
        (Gimbal_Sensor.gyro_x / IMU_GYRO_LSB_PER_DPS) * (M_PI / 180.0f);
    float gy_rad =
        (Gimbal_Sensor.gyro_y / IMU_GYRO_LSB_PER_DPS) * (M_PI / 180.0f);
    float gz_rad =
        (Gimbal_Sensor.gyro_z / IMU_GYRO_LSB_PER_DPS) * (M_PI / 180.0f);

    float ax_mps2 = ax * IMU_GRAVITY_MPS2;
    float ay_mps2 = ay * IMU_GRAVITY_MPS2;
    float az_mps2 = az * IMU_GRAVITY_MPS2;

    uint32_t current_tick = HAL_GetTick();
    float dt = 0.005f;
    if (s_last_update_tick != 0) {
      dt = (current_tick - s_last_update_tick) * 0.001f;
    }
    s_last_update_tick = current_tick;

    if (dt < 0.0005f)
      dt = 0.0005f;
    if (dt > 0.02f)
      dt = 0.02f;

    if (!QEKF_INS.Initialized) {
      IMU_Init_QuaternionEKF();
    }

    IMU_QuaternionEKF_Update(gx_rad, gy_rad, gz_rad, ax_mps2, ay_mps2, az_mps2,
                             dt);
  }

  /*mlx_cmd(MLX_CMD_START_SINGLE, NULL, 0);
  IMU_DELAY(2);
  uint8_t mraw[7];
  if (mlx_cmd(MLX_CMD_READ_MEAS, mraw, 7) == 0) {
    Gimbal_Sensor.mag_x =
        (int16_t)(mraw[1] << 8 | mraw[2]) - Gimbal_Sensor.mag_bias_x;
    Gimbal_Sensor.mag_y =
        (int16_t)(mraw[3] << 8 | mraw[4]) - Gimbal_Sensor.mag_bias_y;
    Gimbal_Sensor.mag_z =
        (int16_t)(mraw[5] << 8 | mraw[6]) - Gimbal_Sensor.mag_bias_z;

    Mag_Update_Noise(Gimbal_Sensor.mag_x, Gimbal_Sensor.mag_y,
  Gimbal_Sensor.mag_z);

    // Mag yaw assist (mag values intentionally not mount-rotated)
    {
      float ekf_yaw_deg = QEKF_INS.Yaw;
      float mag_xy_norm2 = (float)(Gimbal_Sensor.mag_x * Gimbal_Sensor.mag_x +
                                   Gimbal_Sensor.mag_y * Gimbal_Sensor.mag_y);

      if (mag_xy_norm2 > 1.0f && gyro_norm_dps < IMU_MAG_ASSIST_MAX_GYRO_DPS &&
          Gimbal_Sensor.mag_noise < IMU_MAG_ASSIST_MAX_NOISE) {
        float mag_yaw_raw_deg =
            atan2f((float)Gimbal_Sensor.mag_y, (float)Gimbal_Sensor.mag_x) *
  180.0f / M_PI;

        if (!mag_yaw_initialized) {
          mag_yaw_filtered_deg = mag_yaw_raw_deg;
          mag_yaw_initialized = 1;
        } else {
          mag_yaw_filtered_deg = IMU_AngleLerpDeg(
              mag_yaw_filtered_deg, mag_yaw_raw_deg, IMU_MAG_YAW_LPF_ALPHA);
        }

        float yaw_err = IMU_WrapAngleDeg(mag_yaw_filtered_deg - ekf_yaw_deg);
        float yaw_step = IMU_MAG_YAW_BLEND * yaw_err;
        if (yaw_step > IMU_MAG_YAW_MAX_STEP_DEG)
          yaw_step = IMU_MAG_YAW_MAX_STEP_DEG;
        if (yaw_step < -IMU_MAG_YAW_MAX_STEP_DEG)
          yaw_step = -IMU_MAG_YAW_MAX_STEP_DEG;

        ekf_yaw_deg = IMU_WrapAngleDeg(ekf_yaw_deg + yaw_step);
        QEKF_INS.Yaw = ekf_yaw_deg;
      }
    }
  }*/

  {
    float raw_roll = QEKF_INS.Pitch;
    float raw_pitch = QEKF_INS.Roll;
    float raw_yaw = IMU_WrapAngleDeg(QEKF_INS.Yaw + s_ekf_yaw_reset_offset_deg);

    if (!s_ekf_output_initialized) {
      s_ekf_roll_filtered = raw_roll;
      s_ekf_pitch_filtered = raw_pitch;
      s_ekf_yaw_filtered = raw_yaw;
      s_ekf_output_initialized = 1;
    } else {
      s_ekf_roll_filtered =
          IMU_AngleLerpDeg(s_ekf_roll_filtered, raw_roll, IMU_EKF_OUTPUT_ALPHA);
      s_ekf_pitch_filtered = IMU_AngleLerpDeg(s_ekf_pitch_filtered, raw_pitch,
                                              IMU_EKF_OUTPUT_ALPHA);
      s_ekf_yaw_filtered =
          IMU_AngleLerpDeg(s_ekf_yaw_filtered, raw_yaw, IMU_EKF_OUTPUT_ALPHA);
    }

    Gimbal_Sensor.ekf_roll = s_ekf_roll_filtered;
    Gimbal_Sensor.ekf_pitch = s_ekf_pitch_filtered;
    Gimbal_Sensor.ekf_yaw = s_ekf_yaw_filtered;
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

  // Update Gimbal_Sensor.mag_noise
  Gimbal_Sensor.mag_noise = std_dev;
}