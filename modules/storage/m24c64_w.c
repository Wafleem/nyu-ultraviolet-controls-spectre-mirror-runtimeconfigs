/**
  ******************************************************************************
  * @file			m24c64_w.c
  * @brief		Driver for the M24C64-W EEPROM with the STM32 HAL
  ******************************************************************************
  * @attention
  *
  * License and copyright idk
  *
  ******************************************************************************
  *
  * @section Usage Example (from integration test)
  *
  * Hardware:
  *   - I2C2 on PH4 (SCL) / PH5 (SDA), fast mode, timing 0x00B03FDB
  *   - EEPROM WC pin on PF2 (EEPROM0), PF3 (EEPROM0F3) as GPIO outputs
  *   - EEPROM1_ADDR = device I2C address (e.g. 0xA0)
  *
  * @code
  *   #define EEPROM1_ADDR  0xA0   // adjust A0-A2 pins to match hardware
  *
  *   char ee_status_msg[64] = "Not Started";
  *   uint8_t ee_fail_stage = 0;
  *
  *   uint8_t ee_write_buf[] = "Hello EEPROM!";
  *   uint8_t ee_read_buf[32] = {0};
  *
  *   // Pull WC low to enable write
  *   HAL_GPIO_WritePin(EEPROM0_GPIO_Port, EEPROM0_Pin, GPIO_PIN_RESET);
  *   osDelay(10);
  *
  *   // Bus sanity check before attempting I2C
  *   if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY)) {
  *       ee_fail_stage = 99;
  *       snprintf(ee_status_msg, sizeof(ee_status_msg), "I2C_BUSY_LINE_STUCK");
  *   } else {
  *       // Write with short timeout to avoid hanging
  *       if (HAL_I2C_Mem_Write(&hi2c2, EEPROM1_ADDR, 0x0000, I2C_MEMADD_SIZE_16BIT,
  *                             ee_write_buf, sizeof(ee_write_buf), 20) != HAL_OK) {
  *           ee_fail_stage = 1;
  *           snprintf(ee_status_msg, sizeof(ee_status_msg), "WR_ERR_CODE_%lu",
  *                    (unsigned long)hi2c2.ErrorCode);
  *       } else {
  *           osDelay(15);  // Wait for EEPROM internal write cycle
  *           if (HAL_I2C_Mem_Read(&hi2c2, EEPROM1_ADDR, 0x0000, I2C_MEMADD_SIZE_16BIT,
  *                                ee_read_buf, sizeof(ee_write_buf), 20) != HAL_OK) {
  *               ee_fail_stage = 2;
  *               snprintf(ee_status_msg, sizeof(ee_status_msg), "RD_ERR_CODE_%lu",
  *                        (unsigned long)hi2c2.ErrorCode);
  *           } else if (memcmp(ee_write_buf, ee_read_buf, sizeof(ee_write_buf)) != 0) {
  *               ee_fail_stage = 3;
  *               snprintf(ee_status_msg, sizeof(ee_status_msg), "DATA_MISMATCH");
  *           } else {
  *               snprintf(ee_status_msg, sizeof(ee_status_msg), "SUCCESS");
  *           }
  *       }
  *   }
  *
  *   // Pull WC high to protect writes
  *   HAL_GPIO_WritePin(EEPROM0_GPIO_Port, EEPROM0_Pin, GPIO_PIN_SET);
  *
  *   // Log result
  *   LOG_INFO(LOG_TAG_SYS, "[EEPROM] %s (Stage: %d)", ee_status_msg, ee_fail_stage);
  * @endcode
  *
  ******************************************************************************
  */

#include "m24c64_w.h"

#include <string.h>

// Define the Page Size and number of pages
#define PAGE_SIZE 32     // in Bytes
#define PAGE_NUM  256    // number of pages

static uint16_t bytestowrite (uint16_t size, uint16_t offset)
{
	if ((size+offset)<PAGE_SIZE) return size;
	else return PAGE_SIZE-offset;
}

void EEPROM_Write(I2C_HandleTypeDef* hi2c, uint8_t DevAddress, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{

	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(PAGE_SIZE)/log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	// write the data
	for (int i=0; i<numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);  // calculate the remaining bytes to be written

		HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, 2, &data[pos], bytesremaining, 1000);  // write the data to the EEPROM

		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay(5);  // Write cycle delay (5ms)
	}
}

void EEPROM_Read(I2C_HandleTypeDef* hi2c, uint8_t DevAddress, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	int paddrposition = log(PAGE_SIZE)/log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	for (int i=0; i<numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, 2, &data[pos], bytesremaining, 1000);
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;
	}
}

void EEPROM_Erase_Page(I2C_HandleTypeDef* hi2c, uint8_t DevAddress, uint16_t page)
{
	// calculate the memory address based on the page number
	int paddrposition = log(PAGE_SIZE)/log(2);
	uint16_t MemAddress = page<<paddrposition;

	// create a buffer to store the reset values
	uint8_t data[PAGE_SIZE];
	memset(data,0xff,PAGE_SIZE);

	// write the data to the EEPROM
	HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, 2, data, PAGE_SIZE, 1000);

	HAL_Delay (5);  // write cycle delay
}
