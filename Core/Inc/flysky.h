/*
 * flysky.h
 * This file contains all the function prototypes for the flysky.c file.
 * Adapted from https://github.com/AtaberkOKLU/STM32-FlySky-IBus
 */

#ifndef __FLYSKY_H
#define __FLYSKY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdio.h>

#define SERVO_HEADER_1 		0x20
#define SERVO_HEADER_2 		0x40
#define SERVO_BUFFER_SIZE 	0x20
#define TELM_CHECKSUM_CONST 0xFFFF


typedef struct FlyskyServoStruct {
	uint16_t 	Header;
	uint16_t 	Channel_1;
	uint16_t 	Channel_2;
	uint16_t 	Channel_3;
	uint16_t 	Channel_4;
	uint16_t 	Channel_5;
	uint16_t 	Channel_6;
	uint16_t 	Channel_7;
	uint16_t 	Channel_8;
	uint16_t 	Channel_9;
	uint16_t 	Channel_10;
	uint16_t 	Channel_11;
	uint16_t 	Channel_12;
	uint16_t 	Channel_13;
	uint16_t 	Channel_14;
	uint16_t 	Checksum;
} FlyskyServoStruct;

struct __FLAGS {
	volatile uint8_t MOTOR_ARMING:1;
	volatile uint8_t FAIL_SAFE:1;
	volatile uint8_t Transiever_RX_Sync:1;
	volatile enum FLYSKY_SYNC_STATES {
		FLYSKY_SYNC_SYNC0,
		FLYSKY_SYNC_SYNC1,
		FLYSKY_SYNC_SYNCED,
		FLYSKY_SYNC_VERIFIED} FLYSKY_SYNC_STATES:2;
	volatile uint8_t UNUSED:3;
};

extern FlyskyServoStruct ServoList;
extern volatile struct __FLAGS FLAGS;

void Servo_UART_Flysky_Init(UART_HandleTypeDef *huart);
void Servo_UART_RxComplete_Callback(UART_HandleTypeDef *huart);
void Servo_UART_Error_Callback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
#endif /* __FLYSKY_H */
