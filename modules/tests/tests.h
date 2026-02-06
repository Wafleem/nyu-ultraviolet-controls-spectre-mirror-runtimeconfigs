/**
 * @file tests.h
 * @brief Hardware test functions for IMU, CAN, and RC peripherals
 *
 * These test functions can be called from FreeRTOS tasks to verify
 * peripheral functionality. Each test outputs results via USB CDC.
 */
#ifndef TESTS_H
#define TESTS_H

#include <stdint.h>

/**
 * @brief Run IMU sensor test and report via USB
 *
 * Reads all IMU data (accel, gyro, mag, temp) and prints formatted output.
 * Also displays calculated roll/pitch and magnetometer noise estimate.
 *
 * @note Calls System_Read_And_Process() internally
 * @note Uses USB CDC for output - ensure USB is initialized
 */
void Test_IMU_Report(void);

/**
 * @brief Run CAN loopback test and report via USB
 *
 * Sends a counter value on CAN1 and attempts to receive on CAN2.
 * Reports success/failure for both TX and RX operations.
 *
 * @param sent_number Counter value to send (useful for tracking test iterations)
 *
 * @note Requires CAN1 TX connected to CAN2 RX for loopback
 * @note Uses USB CDC for output - ensure USB is initialized
 */
void Test_CAN_Loopback(uint32_t sent_number);

/**
 * @brief Run FlySky RC receiver test and report via USB
 *
 * Reads RC channel data and switch states, prints formatted output.
 * Shows connection status and debug info if disconnected.
 *
 * @note Uses USB CDC for output - ensure USB is initialized
 */
void Test_FlySky_Report(void);

/**
 * @brief Print a single IMU data snapshot (compact format)
 *
 * Lighter weight than Test_IMU_Report() - just prints current values
 * without headers or extra formatting. Suitable for continuous monitoring.
 *
 * @param imu Pointer to IMU data structure (uses IMU_System if NULL)
 */
void Test_IMU_PrintCompact(void);

#endif /* TESTS_H */
