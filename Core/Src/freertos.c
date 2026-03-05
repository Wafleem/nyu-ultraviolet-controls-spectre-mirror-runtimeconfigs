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
#include "app_subscriptions.h"
#include "chassis_controller.h"
#include "shooter_controller.h"
#include "gimbal_controller.h"
#include "cmd_controller.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
  /* -------- QSPI Flash Test -------- */
  uint8_t qspi_test_passed = 0;

  uint8_t qspi_write_buf[] = "Hello from QSPI Flash!\r\n";

  uint8_t qspi_read_buf[64] = {0};

  // Step 1: Initialize the QSPI flash (reset chip, configure quad mode)
  if (CSP_QUADSPI_Init() != HAL_OK) {
      // Init failed -- check pin config, clock, QSPI parameters
      Error_Handler();
    }

  // Step 2: Read status registers (sanity check -- chip is alive)
  uint8_t qspi_status[3];

  if (QSPI_ReadStatus(qspi_status) != HAL_OK) {

      Error_Handler();

  }

  // Step 3: Erase the first 64KB block (flash must be erased before writing)
  if (CSP_QSPI_EraseBlock(0) != HAL_OK) Error_Handler();

  if (QSPI_AutoPollingMemReady() != HAL_OK) Error_Handler();

  // Step 4: Write test string at address 0
  if (CSP_QSPI_WriteMemory(qspi_write_buf, 0, sizeof(qspi_write_buf)) != HAL_OK) {

      Error_Handler();

  }

  // Step 5: Read it back
  if (CSP_QSPI_Read(qspi_read_buf, 0, sizeof(qspi_write_buf)) != HAL_OK) {

      Error_Handler();

  }

  // Step 6: Verify
  if (memcmp(qspi_write_buf, qspi_read_buf, sizeof(qspi_write_buf)) == 0) {

      qspi_test_passed = 1;

  }
  // After this, you can read flash via pointer: volatile uint8_t *p = (uint8_t*)0x90000000;

  // WARNING: Once in memory-mapped mode, you can't use the other QSPI functions anymore

  //          until you re-init. Only enable this if you're done with direct read/write.

  // if (CSP_QSPI_EnableMemoryMappedMode() != HAL_OK) Error_Handler();


  if (QSPI_AutoPollingMemReady() != HAL_OK) Error_Handler();

  // Small delay for USB to stabilize
  osDelay(100);

  // Print boot banner - FreeRTOS scheduler is now running!
  Debug_Printf("========================================\r\n");
  Debug_Printf("   FreeRTOS Started Successfully!\r\n");
  Debug_Printf("========================================\r\n");
  Debug_Printf("[DefaultTask] Running!\r\n");

  // External CAN managers for status reporting
  extern CAN_Manager_t can1_manager;
  extern CAN_Manager_t can2_manager;

  uint32_t heartbeat_counter = 0;
  uint8_t led_state = 0;

  for(;;)
  {
    heartbeat_counter++;

    // Toggle LED every 500ms as heartbeat (1Hz blink)
    if (heartbeat_counter % 500 == 0) {
      led_state = !led_state;
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    // Print system status every 2 seconds
    if (heartbeat_counter % 2000 == 0) {
      uint32_t rc_frames = RC_GetFrameCount();
      uint32_t can1_rx = CAN_Manager_GetRxFrames(&can1_manager);
      uint32_t can2_rx = CAN_Manager_GetRxFrames(&can2_manager);

      LOG_INFO(LOG_TAG_SYS, "tick=%lu RC_frames=%lu CAN1_rx=%lu CAN2_rx=%lu\r\n",
              (unsigned long)HAL_GetTick(),
              (unsigned long)rc_frames,
              (unsigned long)can1_rx,
              (unsigned long)can2_rx);
    }

    osDelay(1);
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

  // Initialize modules that subscribe to topics
  VisionComm_Init();
  MotorDriver_ModuleInit();
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

/* USER CODE END Application */

