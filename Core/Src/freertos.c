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
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Cached sensor data from message center callbacks */
static IMU_System_Data_t s_last_imu;
static RC_ctrl_t s_last_rc;
static robot_status_t s_robot_status;

/* CAN dump: last received raw frame */
static volatile uint32_t s_can_rx_count = 0;
static CanRxFrame s_last_can_frame;

/* CAN manager globals (defined in main.c) */
extern CAN_Manager_t can1_manager;
extern CAN_Manager_t can2_manager;

/* Output buffer for USB CDC */
static char output_buf[512];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for MsgDispatch */
osThreadId_t MsgDispatchHandle;
const osThreadAttr_t MsgDispatch_attributes = {
    .name = "MsgDispatch",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal7,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
    .name = "ControlTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for RefereeTask */
osThreadId_t RefereeTaskHandle;
const osThreadAttr_t RefereeTask_attributes = {
    .name = "RefereeTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void on_imu_update(const MsgEvent *ev, void *user_data);
static void on_rc_update(const MsgEvent *ev, void *user_data);
static void on_can_rx(const MsgEvent *ev, void *user_data);
static void on_robot_status(const MsgEvent *ev, void *user_data);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMsgDispatchTask(void *argument);
void StartControlTask(void *argument);
void StartRefereeTask(void *argument);

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
  /* Initialize message center with 64 event queue depth */
  if (MsgCenter_Init(64) != 0) {
    /* Message center failed to initialize - halt */
    Error_Handler();
  }

  /* Subscribe to sensor updates (ControlTask owns these) */
  MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
  MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);
  MsgCenter_Subscribe(TOPIC_ROBOT_STATUS, on_robot_status, NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MsgDispatch */
  MsgDispatchHandle =
      osThreadNew(StartMsgDispatchTask, NULL, &MsgDispatch_attributes);

  /* creation of ControlTask */
  ControlTaskHandle =
      osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of RefereeTask */
  RefereeTaskHandle =
      osThreadNew(StartRefereeTask, NULL, &RefereeTask_attributes);

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
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN StartDefaultTask */

  /* This task owns the CAN dump - subscribe to raw CAN frames here */
  MsgCenter_Subscribe(TOPIC_CAN_RX, on_can_rx, NULL);

  uint32_t counter = 0;

  for (;;) {
    /* Every 500ms (50 iterations at 10ms), print compact IMU data */
    if (counter % 50 == 0) {
      int16_t ekf_roll_tenths = (int16_t)(-s_last_imu.ekf_roll * 10.0f);
      int16_t ekf_pitch_tenths = (int16_t)(-s_last_imu.ekf_pitch * 10.0f);
      int16_t ekf_yaw_tenths = (int16_t)(s_last_imu.ekf_yaw * 10.0f);

      int len =
          sprintf(output_buf,
                  "A:%6d,%6d,%6d G:%6d,%6d,%6d M:%6d,%6d,%6d "
                  "E:%s%d.%d,%s%d.%d,%s%d.%d\r\n",
                  -s_last_imu.accel_x, -s_last_imu.accel_y, s_last_imu.accel_z,
                  -s_last_imu.gyro_x, -s_last_imu.gyro_y, s_last_imu.gyro_z,
                  -s_last_imu.mag_x, -s_last_imu.mag_y, s_last_imu.mag_z,
                  (ekf_roll_tenths < 0) ? "-" : "", abs(ekf_roll_tenths) / 10,
                  abs(ekf_roll_tenths % 10), (ekf_pitch_tenths < 0) ? "-" : "",
                  abs(ekf_pitch_tenths) / 10, abs(ekf_pitch_tenths % 10),
                  (ekf_yaw_tenths < 0) ? "-" : "", abs(ekf_yaw_tenths) / 10,
                  abs(ekf_yaw_tenths % 10));

      CDC_Transmit_FS((uint8_t *)output_buf, len);
    }

    counter++;
    osDelay(10); /* 10ms = 100Hz loop */
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
void StartMsgDispatchTask(void *argument) {
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
void StartControlTask(void *argument) {
  /* USER CODE BEGIN StartControlTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5); /* 5ms = 200Hz */

  for (;;) {
    /* Read all IMU sensors (ICM-42688P accel/gyro + MLX90393 magnetometer) */
    if (imu_initialized) {
      /*
       * System_Read_And_Process() does:
       * 1. Read accel/gyro via SPI (ICM-42688P)
       * 2. Calculate roll/pitch from accelerometer
       * 3. Read magnetometer via SPI (MLX90393)
       * 4. Apply bias correction to mag values
       * 5. Call Mag_Update_Noise() which updates IMU_System.mag_noise
       *    using a 100-sample circular buffer RMS calculation
       */
      System_Read_And_Process();

      /* Publish complete IMU data including noise estimate to message center */
      MsgCenter_Publish(TOPIC_IMU_UPDATE, &IMU_System, sizeof(IMU_System), 0);
    }

    /* Precise 200Hz timing */
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
void StartRefereeTask(void *argument) {
  /* USER CODE BEGIN StartRefereeTask */
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    referee_unpack_fifo_data();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartRefereeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Callback for IMU data updates from message center
 * @param ev Event containing IMU_System_Data_t
 * @param user_data Unused
 *
 * Called in MsgDispatch task context (not ISR) whenever ControlTask
 * publishes new IMU data. Copies the complete IMU structure including
 * the mag_noise field calculated by Mag_Update_Noise().
 */
static void on_imu_update(const MsgEvent *ev, void *user_data) {
  (void)user_data;
  if (ev->size == sizeof(IMU_System_Data_t)) {
    memcpy(&s_last_imu, ev->data, sizeof(IMU_System_Data_t));
  }
}

/**
 * @brief Callback for RC data updates from message center
 * @param ev Event containing RC_ctrl_t
 * @param user_data Unused
 *
 * Called in MsgDispatch task context whenever the ISR
 * publishes new RC data via MsgCenter_PublishFromISR().
 */
static void on_rc_update(const MsgEvent *ev, void *user_data) {
  (void)user_data;
  if (ev->size == sizeof(RC_ctrl_t)) {
    memcpy(&s_last_rc, ev->data, sizeof(RC_ctrl_t));
  }
}

/**
 * @brief Callback for raw CAN frames from message center
 * @param ev Event containing CanRxFrame
 * @param user_data Unused
 *
 * Called in MsgDispatch task context whenever CAN manager receives
 * any frame on either FDCAN bus. Caches the last frame for the
 * defaultTask CAN dump display.
 */
static void on_can_rx(const MsgEvent *ev, void *user_data) {
  (void)user_data;
  if (ev->size == sizeof(CanRxFrame)) {
    memcpy((void *)&s_last_can_frame, ev->data, sizeof(CanRxFrame));
    s_can_rx_count++;
  }
}

static void on_robot_status(const MsgEvent *ev, void *user_data) {
  (void)user_data;
  if (ev->size == sizeof(robot_status_t)) {
    memcpy((void *)&s_robot_status, ev->data, sizeof(robot_status_t));
  }
}

/* USER CODE END Application */
