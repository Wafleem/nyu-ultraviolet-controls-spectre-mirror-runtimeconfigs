/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25q128jv.h"
#include "quadspi.h"
#include "can_comm.h"
#include "can_manager.h"
#include "imu.h"
#include "message_center.h"
#include "printing.h"
#include "ref_structs.h"
#include "referee.h"
#include "remote_control.h"
#include "tests.h"
#include "usbd_cdc_if.h"
#include "logger.h"
#include "motor_driver.h"
#include "vision_comm.h"
#include "robot_config.h"
#include "app_subscriptions.h"
#include "chassis_controller.h"
#include "shooter_controller.h"
#include "gimbal_controller.h"
#include "cmd_controller.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "m24c64_w.h"
#include "i2c.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_CENTER_QUEUE_LEN 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* CAN manager globals */
extern CAN_Manager_t can1_manager;
extern CAN_Manager_t can2_manager;

/* Robot config */
static robot_id_t s_robot_id;
static robot_status_t s_robot_status;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MsgDispatch */
osThreadId_t MsgDispatchHandle;
const osThreadAttr_t MsgDispatch_attributes = {
  .name = "MsgDispatch",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RefereeTask */
osThreadId_t RefereeTaskHandle;
const osThreadAttr_t RefereeTask_attributes = {
  .name = "RefereeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void on_robot_status(const MsgEvent *ev, void *user_data);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMsgDispatchTask(void *argument);
void StartControlTask(void *argument);
void StartRefereeTask(void *argument);
void StartIMUTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  LOG_INFO(LOG_TAG_SYS, "Boot step 1\r\n");
  /* Initialize message center */
  if (MsgCenter_Init(MSG_CENTER_QUEUE_LEN) != 0) {
      /* Message center failed to initialize - halt */
      Error_Handler();
  }

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MsgDispatch */
  MsgDispatchHandle = osThreadNew(StartMsgDispatchTask, NULL, &MsgDispatch_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of RefereeTask */
  RefereeTaskHandle = osThreadNew(StartRefereeTask, NULL, &RefereeTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  // --- Containers ---
  char qspi_status_msg[64] = "Not started";
  char ee_status_msg[64] = "Not Started";
  uint8_t qspi_fail_stage = 0;
  uint8_t ee_fail_stage = 0;
  
  uint8_t qspi_write_buf[] = "Hello from QSPI Flash!\r\n";
  uint8_t qspi_read_buf[64] = {0};
  uint8_t qspi_status[3];

  uint8_t ee_write_buf[] = "Hello EEPROM!";
  uint8_t ee_read_buf[32] = {0};

  /* -------- QSPI Flash Test (Aggressive Debug) -------- */
  if (CSP_QUADSPI_Init() != HAL_OK) {
      qspi_fail_stage = 1;
      snprintf(qspi_status_msg, sizeof(qspi_status_msg), "INIT_FAIL");
  } else {
      // FORCE A RESET & SANE STATE
      CSP_QSPI_EraseBlock(0); 
      QSPI_AutoPollingMemReady();

      // Attempt the write
      if (CSP_QSPI_WriteMemory(qspi_write_buf, 0, sizeof(qspi_write_buf)) != HAL_OK) {
          qspi_fail_stage = 4;
          snprintf(qspi_status_msg, sizeof(qspi_status_msg), "WRITE_FUNC_RETURNED_ERR");
      } else {
          osDelay(10); // Give the controller a moment
          if (CSP_QSPI_Read(qspi_read_buf, 0, sizeof(qspi_write_buf)) != HAL_OK) {
              qspi_fail_stage = 5;
              snprintf(qspi_status_msg, sizeof(qspi_status_msg), "READ_FUNC_RETURNED_ERR");
          } else if (memcmp(qspi_write_buf, qspi_read_buf, sizeof(qspi_write_buf)) != 0) {
              qspi_fail_stage = 6;
              // Check if it's all FFs (Erase worked, Write failed) or 00s (Bus dead)
              snprintf(qspi_status_msg, sizeof(qspi_status_msg), "VAL[0]=%02X VAL[1]=%02X", qspi_read_buf[0], qspi_read_buf[1]);
          } else {
              snprintf(qspi_status_msg, sizeof(qspi_status_msg), "SUCCESS");
          }
      }
  }

  /* -------- EEPROM Test (Anti-Hang + Bus Check) -------- */
  // Hardware Pin Check
  HAL_GPIO_WritePin(EEPROM0_GPIO_Port, EEPROM0_Pin, GPIO_PIN_RESET);
  osDelay(10);

  // Bus Check: If the bus is busy here, the I2C peripheral is misconfigured or lacks pull-ups
  if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY)) {
      ee_fail_stage = 99;
      snprintf(ee_status_msg, sizeof(ee_status_msg), "I2C_BUSY_LINE_STUCK");
  } else {
      // Attempt Write with a very short timeout so we don't freeze the CPU
      if (HAL_I2C_Mem_Write(&hi2c2, EEPROM1_ADDR, 0x0000, I2C_MEMADD_SIZE_16BIT, 
                            ee_write_buf, sizeof(ee_write_buf), 20) != HAL_OK) {
          ee_fail_stage = 1;
          snprintf(ee_status_msg, sizeof(ee_status_msg), "WR_ERR_CODE_%lu", (unsigned long)hi2c2.ErrorCode);
      } else {
          osDelay(15); // Wait for EEPROM internal write
          if (HAL_I2C_Mem_Read(&hi2c2, EEPROM1_ADDR, 0x0000, I2C_MEMADD_SIZE_16BIT, 
                               ee_read_buf, sizeof(ee_write_buf), 20) != HAL_OK) {
              ee_fail_stage = 2;
              snprintf(ee_status_msg, sizeof(ee_status_msg), "RD_ERR_CODE_%lu", (unsigned long)hi2c2.ErrorCode);
          } else if (memcmp(ee_write_buf, ee_read_buf, sizeof(ee_write_buf)) != 0) {
              ee_fail_stage = 3;
              snprintf(ee_status_msg, sizeof(ee_status_msg), "DATA_MISMATCH");
          } else {
              snprintf(ee_status_msg, sizeof(ee_status_msg), "SUCCESS");
          }
      }
  }
  HAL_GPIO_WritePin(EEPROM0_GPIO_Port, EEPROM0_Pin, GPIO_PIN_SET);

  /* -------- Loop Section -------- */
  uint32_t last_log_time = 0;
  for(;;) {
      if (HAL_GetTick() - last_log_time >= 2000) {
          last_log_time = HAL_GetTick();
          LOG_INFO(LOG_TAG_SYS, "--- DIAGNOSTICS ---");
          LOG_INFO(LOG_TAG_SYS, "[QSPI]   %s (Stage: %d)", qspi_status_msg, qspi_fail_stage);
          LOG_INFO(LOG_TAG_SYS, "[EEPROM] %s (Stage: %d)", ee_status_msg, ee_fail_stage);
      }
      osDelay(100);
  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMsgDispatchTask */
/**
 * @brief Function implementing the MsgDispatch thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMsgDispatchTask */
void StartMsgDispatchTask(void *argument)
{
  /* USER CODE BEGIN StartMsgDispatchTask */
  /*
   * This task processes all message center events.
   * It blocks on the queue and wakes immediately when events are published.
   * Higher priority ensures low-latency event dispatch.
   */
  MsgCenter_DispatcherTask(argument);

  /* Should never reach here - dispatcher runs forever */
  for (;;) {
    osDelay(1000);
  }
  /* USER CODE END StartMsgDispatchTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief Function implementing the ControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  const TickType_t xFrequency = pdMS_TO_TICKS(5);  // 5ms = 200Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Initialize robot ID tracking
  s_robot_id = 0;
  memset(&s_robot_status, 0, sizeof(robot_status_t));
  (void)MsgCenter_Subscribe(TOPIC_ROBOT_STATUS, on_robot_status, NULL);

  // Initialize modules that subscribe to topics
  MotorDriver_ModuleInit(s_robot_id);
  VisionComm_Init();
  CmdController_Init();
  ChassisApp_Init();
  GimbalApp_Init();
  ShooterApp_Init();

  // Wait for USB and other tasks to stabilize
  osDelay(1500);
  Debug_Printf("[ControlTask] Started - running at 200Hz\r\n");

  for(;;)
  {
    CmdController_Task(HAL_GetTick());
    // vTaskDelayUntil ensures consistent period even if execution time varies
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartRefereeTask */
/**
 * @brief Function implementing the RefereeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRefereeTask */
void StartRefereeTask(void *argument)
{
  /* USER CODE BEGIN StartRefereeTask */
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    referee_unpack_fifo_data();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartRefereeTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5);  /* 5ms = 200Hz */

  for(;;)
  {
    /* Read all IMU sensors (ICM-42688P accel/gyro + MLX90393 magnetometer) */
    if (imu_initialized) {
        /*
         * System_Read_And_Process() does:
         * 1. Read accel/gyro via SPI (ICM-42688P)
         * 2. Calculate roll/pitch from accelerometer
         * 3. Read magnetometer via SPI (MLX90393)
         * 4. Apply bias correction to mag values
         * 5. Call Mag_Update_Noise() which updates Gimbal_Sensor.mag_noise
         *    using a 100-sample circular buffer RMS calculation
         */
        System_Read_And_Process();

        /* Publish complete IMU data including noise estimate to message center */
        MsgCenter_Publish(TOPIC_IMU_UPDATE, &Gimbal_Sensor, sizeof(Gimbal_Sensor), 0);
    }

    /* Precise 200Hz timing */
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartIMUTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// If robot ID changes, restart ControlTask with new robot config
static void on_robot_status(const MsgEvent *ev, void *user_data) {
    (void)user_data;
    if (ev->size == sizeof(robot_status_t)) {
        memcpy(&s_robot_status, ev->data, sizeof(robot_status_t));
        if (s_robot_status.robot_id != s_robot_id) {
            const RobotConfig_t *robot_cfg = RobotConfig_Get(s_robot_status.robot_id);
            s_robot_id = s_robot_status.robot_id;
            LOG_INFO(LOG_TAG_SYS, "Changing robot config to %s\r\n", robot_cfg->name);

            // Restart motor registries used in CAN managers
            MotorRegistry_Init(can1_manager.registry, robot_cfg, can1_manager.channel);
            MotorRegistry_Init(can2_manager.registry, robot_cfg, can2_manager.channel);

            // Restart Controls modules and applications
            MotorDriver_ModuleInit(s_robot_status.robot_id);
            ChassisApp_Init();
            GimbalApp_Init();
            ShooterApp_Init();
        }
    }
}

/* USER CODE END Application */

