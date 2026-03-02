#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

/**
 * =============================================================================
 *                         MESSAGE CENTER (Pub-Sub System)
 * =============================================================================
 *
 * This is the communication backbone of the robot firmware. All modules
 * communicate through TOPICS - named channels that carry specific data types.
 *
 *
 *                           DATA FLOW DIAGRAM
 *                           =================
 *
 *   HARDWARE INTERRUPTS                    APPLICATION TASKS
 *   (Producers)                            (Consumers & Producers)
 *   ==================                     ======================
 *
 *   [CAN1/CAN2 RX ISR]                          [Control Task]
 *         |                                      200Hz loop
 *         | PublishFromISR()                         |
 *         v                                          | reads stored data
 *   +------------------+                             v
 *   | TOPIC_CAN_RX     |                    +------------------+
 *   | TOPIC_MOTOR_FB   |----> Queue ------->| cmd_controller   |
 *   | TOPIC_GM6020_FB  |                    | - processes RC   |
 *   +------------------+                    | - spin mode      |
 *                                           | - gimbal follow  |
 *   [UART7 RX ISR]                          +--------+---------+
 *   (Remote Controller)                              |
 *         |                                          | Publish()
 *         v                                          v
 *   +------------------+                    +------------------+
 *   | TOPIC_RC_UPDATE  |----> Queue ------->| TOPIC_CHASSIS_CMD|---> chassis_controller
 *   +------------------+                    | TOPIC_GIMBAL_CMD |---> gimbal_controller
 *                                           | TOPIC_SHOOT_CMD  |---> shooter_controller
 *   [SPI2 / ControlTask]                    +------------------+
 *   (ICM-42688P IMU)
 *         |
 *         v
 *   +------------------+
 *   | TOPIC_IMU_UPDATE |----> Queue
 *   +------------------+
 *
 *                         HOW IT WORKS
 *                         ============
 *
 *   1. PUBLISH: Producer puts data into the FreeRTOS queue
 *      - From ISR: MsgCenter_PublishFromISR() - non-blocking
 *      - From Task: MsgCenter_Publish() - can block if queue full
 *
 *   2. DISPATCH: MsgDispatch task (high priority) wakes up immediately
 *      - Reads event from queue
 *      - Calls all registered callbacks for that topic
 *
 *   3. SUBSCRIBE: Consumers register callbacks during init
 *      - Callback runs in MsgDispatch task context (NOT in ISR!)
 *      - Keep callbacks fast (<1ms) to avoid blocking other events
 *
 * =============================================================================
 */

#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 *                              TOPIC DEFINITIONS
 * =============================================================================
 * Each topic carries a specific data structure. Producers and consumers must
 * agree on the data format.
 */

typedef enum {
    /* -------------------------------------------------------------------------
     * INPUT TOPICS (Hardware -> Software)
     * These are published from ISRs when hardware events occur
     * ------------------------------------------------------------------------- */

    TOPIC_RC_UPDATE = 0,      // Remote control data from receiver
                              // Data: RC_ctrl_t (24 bytes)
                              // Publisher: UART7 RX ISR (ibus.c)
                              // Subscribers: cmd_controller

    TOPIC_IMU_UPDATE,         // IMU sensor data (gyro, accel, angles)
                              // Data: Gimbal_Sensor_Data_t
                              // Publisher: ControlTask (imu.c)
                              // Subscribers: cmd_controller, gimbal_controller

    TOPIC_CAN_RX,             // Raw CAN frame received (for debugging)
                              // Data: CanRxFrame (12 bytes)
                              // Publisher: FDCAN RX ISR (can_manager.c)
                              // Subscribers: (typically none in production)

    TOPIC_MOTOR_FEEDBACK,     // M3508/M2006 motor feedback (chassis, shooter)
                              // Data: MotorFeedbackEvent (16 bytes)
                              // Publisher: FDCAN RX ISR -> motor parsing
                              // Subscribers: motor_driver -> chassis_controller

    TOPIC_CHASSIS_POWER,      // Chassis power reading
                              // Data: PowerFeedbackEvent

    TOPIC_GM6020_FEEDBACK,    // GM6020 gimbal motor feedback
                              // Data: GM6020FeedbackEvent (12 bytes)
                              // Publisher: FDCAN RX ISR -> motor parsing
                              // Subscribers: motor_driver -> gimbal_controller

    /* -------------------------------------------------------------------------
     * COMMAND TOPICS (Control Task -> Motor Controllers)
     * These are published by the control task at 200Hz
     * ------------------------------------------------------------------------- */

    TOPIC_CHASSIS_CMD,        // Chassis movement command
                              // Data: ChassisCmd { vx, vy, wz, enabled }
                              // Publisher: cmd_controller (200Hz)
                              // Subscribers: chassis_controller

    TOPIC_SHOOT_CMD,          // Shooter command (friction wheels, feeder)
                              // Data: ShootCmd { fire, friction_on, ... }
                              // Publisher: cmd_controller (200Hz)
                              // Subscribers: shooter_controller

    TOPIC_GIMBAL_CMD,         // Gimbal command (yaw, pitch angles/rates)
                              // Data: GimbalCmd { yaw_rate, pitch_rate, enabled, ... }
                              // Publisher: cmd_controller (200Hz)
                              // Subscribers: gimbal_controller

    /* -------------------------------------------------------------------------
     * REFEREE TOPICS (Referee Task -> Motor Controllers)
     * These are published by the referee task at 100 Hz
     * ------------------------------------------------------------------------- */
    TOPIC_BARREL_HEAT,        // Barrel heat data
                              // Data: power_heat_data_t
                              // Publisher: referee (10Hz)
    TOPIC_ROBOT_STATUS,       // Robot status data
                              // Data: robot_status_t
                              // Publisher: referee (10Hz)
    TOPIC_VTM_RC,             // VTM remote control data
                              // Data: remote_control_t
                              // Publisher: referee (30Hz)

    /* -------------------------------------------------------------------------
     * VISION TOPICS (External -> Gimbal)
     * High-priority path for auto-aim
     * ------------------------------------------------------------------------- */

    TOPIC_VISION_DATA,        // Vision system target data
                              // Data: VisionData { yaw_err, pitch_err, fire_cmd, ... }
                              // Publisher: vision_comm.c (USB/UART RX)
                              // Subscribers: cmd_controller, gimbal_controller

    TOPIC_NUM_TOPICS          // Total number of topics (must be last!)

} MsgTopic;


/* =============================================================================
 *                              CONFIGURATION
 * ============================================================================= */

#ifndef MC_MAX_PAYLOAD
#define MC_MAX_PAYLOAD 128    // Maximum bytes per message (increase if needed)
#endif

#define MC_MAX_SUBS_PER_TOPIC 8   // Maximum subscribers per topic


/* =============================================================================
 *                              DATA STRUCTURES
 * ============================================================================= */

/**
 * @brief Event structure passed through the queue
 *
 * This is the "envelope" that wraps all messages. The actual data
 * is copied into the data[] array.
 */
typedef struct {
    uint16_t topic;                 // Which topic (MsgTopic enum value)
    uint16_t size;                  // How many bytes are in data[]
    uint8_t  data[MC_MAX_PAYLOAD];  // The actual message payload
} MsgEvent;

/**
 * @brief Callback function signature
 *
 * @param ev        Pointer to the event (read-only, do NOT store this pointer!)
 * @param user_data User data pointer passed during subscribe (can be NULL)
 *
 * IMPORTANT: Callbacks run in the MsgDispatch task context, NOT in ISRs.
 * Keep callbacks fast (<1ms). For slow operations, publish another event.
 */
typedef void (*MsgCallback)(const MsgEvent *ev, void *user_data);

/**
 * @brief Statistics for debugging and monitoring
 */
typedef struct {
    uint32_t events_published;      // Total events published (all time)
    uint32_t events_dispatched;     // Total events dispatched (all time)
    uint32_t queue_overflow_count;  // Times publish failed (queue full)
    uint32_t current_queue_depth;   // Current events waiting in queue
    uint32_t max_queue_depth;       // High water mark
} MsgCenter_Stats;




/* =============================================================================
 *                              PUBLIC API
 * ============================================================================= */

/**
 * @brief Initialize the message center
 *
 * Call this ONCE during startup, AFTER osKernelInitialize() but BEFORE
 * osKernelStart(). Creates the FreeRTOS queue and mutex.
 *
 * @param queue_length  How many events the queue can hold (32-128 typical)
 * @return 0 on success, -1 on failure
 */
int MsgCenter_Init(size_t queue_length);

/**
 * @brief Publish an event from a TASK (not ISR!)
 *
 * @param topic         Which topic to publish to
 * @param data          Pointer to data to send
 * @param size          Size of data in bytes (max MC_MAX_PAYLOAD)
 * @param block_time_ms How long to wait if queue is full (0 = don't wait)
 * @return 0 on success, -1 if queue full, -2 if invalid params
 */
int MsgCenter_Publish(MsgTopic topic, const void *data, size_t size, uint32_t block_time_ms);

/**
 * @brief Publish an event from an ISR (interrupt handler)
 *
 * This is the ISR-safe version. It never blocks and automatically
 * triggers a context switch to the dispatcher task if needed.
 *
 * @param topic  Which topic to publish to
 * @param data   Pointer to data to send
 * @param size   Size of data in bytes
 * @return 0 on success, -1 if queue full
 */
int MsgCenter_PublishFromISR(MsgTopic topic, const void *data, size_t size);

/**
 * @brief Subscribe to a topic
 *
 * Call this during initialization. The callback will be invoked whenever
 * someone publishes to this topic.
 *
 * @param topic     Which topic to subscribe to
 * @param cb        Your callback function
 * @param user_data Optional pointer passed to callback (can be NULL)
 * @return 0 on success, -1 invalid params, -2 too many subscribers
 */
int MsgCenter_Subscribe(MsgTopic topic, MsgCallback cb, void *user_data);

/**
 * @brief The dispatcher task - DO NOT CALL DIRECTLY
 *
 * This function runs as a FreeRTOS task. It blocks on the queue and
 * dispatches events to subscribers. Created automatically in freertos.c.
 */
void MsgCenter_DispatcherTask(void *pvParameters);

/**
 * @brief Get statistics for debugging
 */
void MsgCenter_GetStats(MsgCenter_Stats *stats);


#ifdef __cplusplus
}
#endif

#endif // MESSAGE_CENTER_H
