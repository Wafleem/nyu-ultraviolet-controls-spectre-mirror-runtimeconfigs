#include "sdcard.h"
#include "fatfs.h"
#include "printing.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Use the CubeMX-generated globals for the filesystem
// SDFatFS, SDPath are declared in fatfs.h

static FIL s_file;
static bool s_mounted = false;
static bool s_file_open = false;
static char s_buf[256];

int SDCard_Init(void)
{
    if (s_mounted) return 0;

    FRESULT fr = f_mount(&SDFatFS, SDPath, 1);
    if (fr != FR_OK) {
        Debug_Printf("[SD] Mount failed: error %d\r\n", fr);
        return -1;
    }
    s_mounted = true;
    Debug_Printf("[SD] Mounted OK\r\n");
    return 0;
}

void SDCard_Deinit(void)
{
    if (s_file_open) {
        SDCard_Close();
    }
    if (s_mounted) {
        f_mount(NULL, SDPath, 0);
        s_mounted = false;
    }
}

bool SDCard_Inserted(void) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET;
}

bool SDCard_IsReady(void)
{
    return s_mounted;
}

int SDCard_Open(const char *filename)
{
    if (!s_mounted) return -1;
    if (s_file_open) {
        f_close(&s_file);
        s_file_open = false;
    }

    FRESULT fr = f_open(&s_file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        Debug_Printf("[SD] Open '%s' failed: error %d\r\n", filename, fr);
        return -1;
    }
    s_file_open = true;
    return 0;
}

void SDCard_Close(void)
{
    if (s_file_open) {
        f_close(&s_file);
        s_file_open = false;
    }
}

int SDCard_Write(const char *fmt, ...)
{
    if (!s_file_open) return -1;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(s_buf, sizeof(s_buf), fmt, args);
    va_end(args);

    if (len <= 0) return -1;
    if ((size_t)len >= sizeof(s_buf)) len = sizeof(s_buf) - 1;

    UINT bw;
    FRESULT fr = f_write(&s_file, s_buf, (UINT)len, &bw);
    if (fr != FR_OK || bw != (UINT)len) return -1;

    return (int)bw;
}

int SDCard_Writeln(const char *fmt, ...)
{
    if (!s_file_open) return -1;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(s_buf, sizeof(s_buf) - 2, fmt, args);  // reserve room for \r\n
    va_end(args);

    if (len < 0) return -1;
    if ((size_t)len >= sizeof(s_buf) - 2) len = sizeof(s_buf) - 3;

    s_buf[len++] = '\r';
    s_buf[len++] = '\n';
    s_buf[len] = '\0';

    UINT bw;
    FRESULT fr = f_write(&s_file, s_buf, (UINT)len, &bw);
    if (fr != FR_OK || bw != (UINT)len) return -1;

    return (int)bw;
}

void SDCard_Flush(void)
{
    if (s_file_open) {
        f_sync(&s_file);
    }
}
