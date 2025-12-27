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
#include "imu.h"
#include "can.h"
#include "flysky.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* IMU types and prototypes moved to Core/Inc/imu.h */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* IMU register defines moved to Core/Inc/imu.h */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// CAN Variables defined in Core/Src/can.c
char uart_buf[256];

// IMU Variables (defined in Core/Src/imu.c)

// SPI Handles are defined in CubeMX-generated `spi.c` and declared where needed.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
// IMU APIs are declared in Core/Inc/imu.h

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CAN implementation moved to Core/Src/can.c */

/* IMU implementation moved to Core/Src/imu.c */
// Read FlySky RC data and print formatted output (uses uart_buf)
static void Run_FlySky_Report(void)
{
  sprintf(uart_buf, "----- FLYSKY RC DATA -----\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  if (FLAGS.FLYSKY_SYNC_STATES == FLYSKY_SYNC_VERIFIED) {
    sprintf(uart_buf, "Status: CONNECTED %s\r\n", 
        FLAGS.FAIL_SAFE ? "(FAILSAFE!)" : "(OK)");
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    // Right stick
    sprintf(uart_buf, "Right Stick:  Horiz(CH1)=%4d  Vert(CH2)=%4d\r\n",
        ServoList.Channel_1, ServoList.Channel_2);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    // Left stick
    sprintf(uart_buf, "Left Stick:   Throt(CH3)=%4d  Yaw(CH4)=%4d\r\n",
        ServoList.Channel_3, ServoList.Channel_4);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    // Switches and knobs
    sprintf(uart_buf, "SwA(CH5)=%4d  SwB(CH6)=%4d  SwC(CH7)=%4d  SwD(CH8)=%4d\r\n",
        ServoList.Channel_5, ServoList.Channel_6,
        ServoList.Channel_7, ServoList.Channel_8);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    // Aux channels
    sprintf(uart_buf, "VrA(CH9)=%4d  VrB(CH10)=%4d\r\n",
        ServoList.Channel_9, ServoList.Channel_10);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

  } else {
    sprintf(uart_buf, "Status: NOT CONNECTED (Sync: %d)\r\n", 
        FLAGS.FLYSKY_SYNC_STATES);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);
  }
}

// Run the CAN loopback test and print results (uses uart_buf)
static void Run_CAN_Loopback(uint32_t sent_number)
{
  sprintf(uart_buf, "========== LOOP %lu ==========" "\r\n", sent_number);
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  sprintf(uart_buf, "----- CAN TEST -----\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  if (can_send_number(sent_number)) {
    sprintf(uart_buf, "CAN1 TX: Sent %lu\r\n", sent_number);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  } else {
    sprintf(uart_buf, "CAN1 TX: ERROR!\r\n");
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  }
  HAL_Delay(10);

  uint32_t rxval = 0;
  if (can_check_received(&rxval)) {
    sprintf(uart_buf, "CAN2 RX: Received %lu\r\n", rxval);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  } else {
    sprintf(uart_buf, "CAN2 RX: TIMEOUT!\r\n");
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  }
  HAL_Delay(10);
}

// Read/process IMU data and print formatted output (uses uart_buf)
static void Run_IMU_Report(void)
{
  sprintf(uart_buf, "----- IMU DATA -----\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  if (imu_initialized) {
    System_Read_And_Process();

    sprintf(uart_buf, "Accel: X=%d Y=%d Z=%d\r\n",
        IMU_System.accel_x, IMU_System.accel_y, IMU_System.accel_z);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    sprintf(uart_buf, "Gyro:  X=%d Y=%d Z=%d\r\n",
        IMU_System.gyro_x, IMU_System.gyro_y, IMU_System.gyro_z);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    sprintf(uart_buf, "Mag:   X=%d Y=%d Z=%d\r\n",
        IMU_System.mag_x, IMU_System.mag_y, IMU_System.mag_z);
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

    int16_t temp_int = (int16_t)(IMU_System.temp_c * 10);
    sprintf(uart_buf, "Temp:  %d.%d C\r\n", temp_int / 10, abs(temp_int % 10));
    CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
    HAL_Delay(10);

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
   // Initialize FlySky RC receiver
  sprintf(uart_buf, "Initializing FlySky iBus receiver...\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  Servo_UART_Flysky_Init(&huart7);

  sprintf(uart_buf, "FlySky Init: READY (waiting for signal)\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  sprintf(uart_buf, "CAN configured. Starting main loop...\r\n\r\n");
  CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));
  HAL_Delay(10);

  uint32_t sent_number = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //WHICHEVER TESTS YOU WANT TO RUN,COMMENT/UNCOMMENT HERE.
    while (1)
    {
      //you must connect CAN1 TX to CAN2 RX for loopback test
      Run_CAN_Loopback(sent_number); //sent number gets incremented at end of loop
      Run_IMU_Report();
      Run_FlySky_Report();

      sprintf(uart_buf, "\r\n");
      CDC_Transmit_FS((uint8_t*)uart_buf, strlen(uart_buf));

      sent_number++; //used in can test
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
  if (htim->Instance == TIM1)
  {
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
#ifdef USE_FULL_ASSERT
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
