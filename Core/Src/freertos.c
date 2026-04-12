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
#include "app_subscriptions.h"
#include "can_comm.h"
#include "can_manager.h"
#include "chassis_controller.h"
#include "cmd_controller.h"
#include "fatfs.h"
#include "gimbal_controller.h"
#include "imu.h"
#include "logger.h"
#include "message_center.h"
#include "motor_driver.h"
#include "printing.h"
#include "quadspi.h"
#include "ref_structs.h"
#include "referee.h"
#include "remote_control.h"
#include "robot_config.h"
#include "sdcard_logger.h"
#include "shooter_controller.h"
#include "tof_sensor.h"
#include "usbd_cdc_if.h"
#include "vision_comm.h"
#include "w25q128jv.h"
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

/* Robot config */
static robot_id_t s_robot_id;
static robot_status_t s_robot_status;

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
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ToFTask */
osThreadId_t ToFTaskHandle;
const osThreadAttr_t ToFTask_attributes = {
  .name = "ToFTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for LoggerTask */
osThreadId_t SDCardTaskHandle;
const osThreadAttr_t SDCardTask_attributes = {
  .name = "SDCardTask",
  .stack_size = 512 * 4,   
  .priority = (osPriority_t) osPriorityLow,
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void on_robot_status(const MsgEvent *ev, void *user_data);
static void on_supercap_feedback(const MsgEvent *ev, void *user_data);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMsgDispatchTask(void *argument);
void StartControlTask(void *argument);
void StartRefereeTask(void *argument);
void StartIMUTask(void *argument);
void StartToFTask(void *argument);
void StartSDCardTask(void *argument);

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

  /* creation of ToFTask */
  ToFTaskHandle = osThreadNew(StartToFTask, NULL, &ToFTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  SDCardTaskHandle = osThreadNew(StartSDCardTask, NULL, &SDCardTask_attributes);
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


  // Print boot banner
  Debug_Printf("========================================\r\n");
  Debug_Printf("   FreeRTOS Started Successfully!\r\n");
  Debug_Printf("========================================\r\n");
  Debug_Printf("[DefaultTask] Running!\r\n");

  // Subscribe to Wraith (supercap) telemetry. Callback runs in MsgDispatch
  // task context and prints the latest snapshot via the logger.
  (void)MsgCenter_Subscribe(TOPIC_SUPERCAP_FEEDBACK, on_supercap_feedback, NULL);

  // External CAN managers for status reporting
  extern CAN_Manager_t can1_manager;
  extern CAN_Manager_t can2_manager;

  uint32_t heartbeat_counter = 0;
  uint8_t led_state = 0;

  for (;;) {
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

      LOG_INFO(LOG_TAG_SYS,
               "tick=%lu RC_frames=%lu CAN1_rx=%lu CAN2_rx=%lu\r\n",
               (unsigned long)HAL_GetTick(), (unsigned long)rc_frames,
               (unsigned long)can1_rx, (unsigned long)can2_rx);
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
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms = 200Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Initialize robot ID tracking
  s_robot_id = 0;
  memset(&s_robot_status, 0, sizeof(robot_status_t));
  (void)MsgCenter_Subscribe(TOPIC_ROBOT_STATUS, on_robot_status, NULL);
  const RobotConfig_t *robot_cfg = RobotConfig_Get(s_robot_status.robot_id);

  // Initialize modules that subscribe to topics
  MotorDriver_ModuleInit(s_robot_id);
  VisionComm_Init();
  CmdController_Init();
  ChassisApp_Init();
  GimbalApp_Init();
  ShooterApp_Init(robot_cfg);

  // Wait for USB and other tasks to stabilize
  osDelay(1500);
  Debug_Printf("[ControlTask] Started - running at 200Hz\r\n");

  for (;;) {
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

/* USER CODE BEGIN Header_StartIMUTask */
/**
 * @brief Function implementing the IMUTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument) {
  /* USER CODE BEGIN StartIMUTask */
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

void StartToFTask(void *argument) {
  /* USER CODE BEGIN StartToFTask */
  const TickType_t xFrequency = pdMS_TO_TICKS(50); /* 50ms = 20Hz */
  TickType_t xLastWakeTime;

  /* Wait for USB CDC and peripherals to stabilize */
  osDelay(2000);

  /* Use LOG_INFO(LOG_TAG_SYS) since it's proven to work on serial */
  LOG_INFO(LOG_TAG_SYS, "ToF task started");
  osDelay(50);

  if (ToF_Init() != 0) {
    LOG_ERROR(LOG_TAG_SYS, "ToF init FAILED - task suspended");
    osDelay(50);
    vTaskSuspend(NULL);
  }

  LOG_INFO(LOG_TAG_SYS, "ToF running at 20Hz");
  osDelay(50);
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    ToF_Task();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartToFTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void on_supercap_feedback(const MsgEvent *ev, void *user_data) {
  (void)user_data;
  if (ev->size != sizeof(SupercapFeedbackEvent)) return;
  const SupercapFeedbackEvent *sc = (const SupercapFeedbackEvent *)ev->data;

  static const char *const mode_names[] = {
      "DISABLED", "CHARGING", "DISCHARGING", "UNDERVOLTAGE"
  };
  const char *mode_str = (sc->mode < 4) ? mode_names[sc->mode] : "UNKNOWN";

  LOG_INFO(LOG_TAG_CAN,
            "[Wraith] PMM=%.1fW Chassis=%.1fW Cap=%.1f%% Mode=%s tick=%lu\r\n",
            (double)sc->pmm_w, (double)sc->chassis_w,
            (double)sc->voltage_pct, mode_str,
            (unsigned long)sc->tick_ms);
}

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
      MotorRegistry_Init(can1_manager.registry, robot_cfg,
                         can1_manager.channel);
      MotorRegistry_Init(can2_manager.registry, robot_cfg,
                         can2_manager.channel);

      // Restart Controls modules and applications
      MotorDriver_ModuleInit(s_robot_status.robot_id);
      ChassisApp_Init();
      GimbalApp_Init();
      ShooterApp_Init(robot_cfg);
    }
  }
}

void StartSDCardTask(void *argument)
{
  (void)argument;

  const TickType_t log_period_ticks   = pdMS_TO_TICKS(100);   // 10Hz
  TickType_t last_wake_time = xTaskGetTickCount();

  osDelay(300);
  SDCard_Logger_Init();
  for (;;)
  {
    SDCard_Logger_Task();
    vTaskDelayUntil(&last_wake_time, log_period_ticks);
  }
}

/* USER CODE END Application */
