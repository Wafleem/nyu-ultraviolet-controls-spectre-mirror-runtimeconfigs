/**
  ******************************************************************************
  * @file			m24c64_w.h
  * @brief		Header for the M24C64-W EEPROM driver
  ******************************************************************************
  * @attention
  *
  * License and copyright idk
  *
  ******************************************************************************
  */

#ifndef CHIP_DRIVERS_M24C64_W_H_
#define CHIP_DRIVERS_M24C64_W_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

void EEPROM_Write(I2C_HandleTypeDef* hi2c, uint8_t DevAddress, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_Read(I2C_HandleTypeDef* hi2c, uint8_t DevAddress, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_Erase_Page(I2C_HandleTypeDef* hi2c, uint8_t DevAddress, uint16_t page);


#ifdef __cplusplus
}
#endif

#endif /* CHIP_DRIVERS_M24C64_W_H_ */
