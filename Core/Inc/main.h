/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_CS_Pin GPIO_PIN_0
#define IMU_CS_GPIO_Port GPIOI
#define SD_DETECT_Pin GPIO_PIN_7
#define SD_DETECT_GPIO_Port GPIOB
#define EEPROM0_Pin GPIO_PIN_2
#define EEPROM0_GPIO_Port GPIOF
#define EEPROM0F3_Pin GPIO_PIN_3
#define EEPROM0F3_GPIO_Port GPIOF
#define MAG_CS_Pin GPIO_PIN_1
#define MAG_CS_GPIO_Port GPIOK
#define DataLine2_Pin GPIO_PIN_9
#define DataLine2_GPIO_Port GPIOE
#define DEBUG_Pin GPIO_PIN_11
#define DEBUG_GPIO_Port GPIOH
#define DataLine1_Pin GPIO_PIN_8
#define DataLine1_GPIO_Port GPIOE
#define QUADSPI_BK2_IO0_Pin GPIO_PIN_7
#define QUADSPI_BK2_IO0_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define DMA_SECTION __attribute__((section(".dma_bss"), aligned(32)))
#define EEPROM0_ADDR 0xA2
#define EEPROM1_ADDR 0xA0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
