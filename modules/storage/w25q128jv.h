/**
  ******************************************************************************
  * @file			w25q128jv.h
  * @brief		Header for the W25Q128JV SPI Flash chip
  ******************************************************************************
  * @attention
  *
  * License and copyright idk
  *
  ******************************************************************************
  */

#ifndef CHIP_DRIVERS_W25Q128JV_H_
#define CHIP_DRIVERS_W25Q128JV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "quadspi.h"

/*W25Q128 memory parameters*/
#define MEMORY_FLASH_SIZE				0x1000000 /* 16 Mb total  */
#define MEMORY_BLOCK_SIZE				0x10000   /*  blocks of 64KBytes */
#define MEMORY_SECTOR_SIZE				0x1000    /* 4kBytes */
#define MEMORY_PAGE_SIZE				0x100     /* 256 bytes */


/* W25Q128 status register bits */
#define QUAD_ENABLE 0x02

#define OUTPUT_DRIVER_100 0x9f

/*W25Q128JV commands */
#define CHIP_ERASE_CMD 0xC7 /* or 0x60, chip supports both erase commands */
#define READ_STATUS_REG_CMD 0x05
#define WRITE_ENABLE_CMD 0x06
#define VOLATILE_SR_WRITE_ENABLE             0x50
#define READ_STATUS_REG2_CMD 0x35
#define WRITE_STATUS_REG2_CMD 0x31
#define READ_STATUS_REG3_CMD 0x15
#define WRITE_STATUS_REG3_CMD                0x11
#define SECTOR_ERASE_CMD 0x20
#define BLOCK_ERASE_CMD 0xD8
#define QUAD_IN_FAST_PROG_CMD 0x32
#define FAST_PROG_CMD 0x02
#define QUAD_OUT_FAST_READ_CMD 0x6B
#define DUMMY_CLOCK_CYCLES_READ_QUAD 8
#define QUAD_IN_OUT_FAST_READ_CMD 0xEB
#define RESET_ENABLE_CMD 0x66
#define RESET_EXECUTE_CMD 0x99

/* function headers */
uint8_t CSP_QUADSPI_Init(void);
uint8_t CSP_QSPI_EraseSector(uint32_t EraseStartAddress ,uint32_t EraseEndAddress);
uint8_t CSP_QSPI_EraseBlock(uint32_t flash_address);
uint8_t CSP_QSPI_WriteMemory(uint8_t* buffer, uint32_t address, uint32_t buffer_size);
uint8_t CSP_QSPI_EnableMemoryMappedMode(void);
uint8_t CSP_QSPI_Erase_Chip (void);
uint8_t QSPI_AutoPollingMemReady(void);
uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
uint8_t QSPI_ReadStatus(uint8_t status[3]);

#ifdef __cplusplus
}
#endif

#endif /* CHIP_DRIVERS_W25Q128JV_H_ */
