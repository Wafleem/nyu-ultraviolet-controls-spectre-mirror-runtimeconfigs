#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H

#include <stdint.h>

// Motor feedback structure
typedef struct {
    uint16_t angle;
    int16_t  speed;
    int16_t  current;
    uint8_t  temp;
    uint32_t last_update_time;
} Motor_Feedback;

#endif // MOTOR_FEEDBACK_H
