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

// Supercap (Wraith) telemetry parsed from CAN IDs 0x405 and 0x406.
// Per Controls_Supercap/Datasheets/CAN_PROTOCOL.md.
typedef struct {
    float    pmm_w;        // 0x405 bytes 0-3, PMM input power (W)
    float    chassis_w;    // 0x405 bytes 4-7, chassis output power (W)
    float    voltage_pct;  // 0x406 bytes 0-3, supercap voltage % (0..100)
    uint8_t  mode;         // 0x406 byte 4, charger mode
                           // 0=DISABLED, 1=CHARGING, 2=DISCHARGING,
                           // 3=UNDERVOLTAGE, 4=READY (pre-primed, discharge will be fast)
    uint32_t tick_ms;      // last update tick (HAL_GetTick)
} SupercapFeedbackEvent;

typedef struct {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} ChassisIMUFeedbackEvent;

typedef struct {
    float offset[3];
    float normal[3];
} ChassisCalibEvent;

// Operator-state snapshot for HUD overlay (chassis mode + auto-aim engage)
typedef struct {
    uint8_t spin_mode;       // 1 = small-gyro spin, 0 = no spin
    uint8_t gimbal_follow;   // 1 = chassis follows gimbal, 0 = chassis-frame
    uint8_t aimbot_engaged;  // 1 = vision auto-aim toggle on, 0 = off
} HudOpStateEvent;
#endif // CAN_COMM_H
