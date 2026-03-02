#include "crc16.h"
#include <stddef.h>

static uint8_t crc_tab16_init = 0;
static uint16_t crc_tab16[256];

/*
 * CRC16/CCITT-FALSE: poly=0x1021, init=0xFFFF, MSB-first
 * Matches Jetson Seasky implementation.
 */
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes)
{
    if (!crc_tab16_init)
        init_crc16_tab();
    uint16_t crc = CRC_START_16;
    if (input_str != NULL) {
        for (uint16_t a = 0; a < num_bytes; a++) {
            crc = (crc << 8) ^ crc_tab16[((crc >> 8) ^ input_str[a]) & 0xFF];
        }
    }
    return crc;
}

uint16_t update_crc_16(uint16_t crc, uint8_t c)
{
    if (!crc_tab16_init)
        init_crc16_tab();
    return (crc << 8) ^ crc_tab16[((crc >> 8) ^ c) & 0xFF];
}

void init_crc16_tab(void)
{
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (uint16_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC_POLY_16;
            else
                crc = crc << 1;
        }
        crc_tab16[i] = crc;
    }
    crc_tab16_init = 1;
}
