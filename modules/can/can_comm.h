#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <stdint.h>

typedef struct {
    uint16_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanRxFrame;

// Simplified motor feedback parsed from standard motor frames (0x201..0x20B)
typedef struct {
    uint8_t  id;       // same as (StdId - 0x201), 0..7
    uint16_t angle;
    int16_t  speed;
    int16_t  current;
    uint8_t  temp;
    uint32_t tick_ms;
} MotorFeedbackEvent;

// GM6020 specific feedback (angle + speed), format differs from M3508/M2006
typedef struct {
    uint8_t  id;       // 1..7
    uint16_t angle;
    int16_t  speed;
    uint32_t tick_ms;
    int16_t   current; 
} GM6020FeedbackEvent;

typedef struct {
    float power;    // watts
} PowerFeedbackEvent;

typedef struct {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} ChassisIMUFeedbackEvent;

typedef struct {
    float offset[3];
    float normal[3];
} ChassisCalibEvent;
#endif // CAN_COMM_H
