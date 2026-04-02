#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the SD card module.
 * Mounts the filesystem. Call once from a FreeRTOS task (not main).
 * @return 0 on success, -1 on failure
 */
int SDCard_Init(void);

/**
 * Deinitialize / unmount the SD card.
 */
void SDCard_Deinit(void);

/**
 * @return true if the SD card is inserted
 */
bool SDCard_Inserted(void);

/**
 * @return true if the SD card is mounted and ready
 */
bool SDCard_IsReady(void);

/**
 * Open (or create) a log file for writing. Appends by default.
 * @param filename  e.g. "log.txt" or "match_01.csv"
 * @return 0 on success, -1 on failure
 */
int SDCard_Open(const char *filename);

/**
 * Close the currently open file (flushes data).
 */
void SDCard_Close(void);

/**
 * Printf-style write to the open file. No trailing newline added.
 * @param fmt  printf format string
 * @return number of bytes written, or -1 on error
 */
int SDCard_Write(const char *fmt, ...);

/**
 * Printf-style write with automatic trailing "\r\n".
 * @param fmt  printf format string
 * @return number of bytes written, or -1 on error
 */
int SDCard_Writeln(const char *fmt, ...);

/**
 * Flush any buffered data to the SD card.
 * Call periodically or before unmounting.
 */
void SDCard_Flush(void);

#ifdef __cplusplus
}
#endif

#endif // SDCARD_H
