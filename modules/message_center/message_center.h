/**
 * @file message_center.h
 * @brief Minimal FreeRTOS-based pub-sub message center for IMU data
 */
#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include <stdint.h>
#include <stddef.h>

/* Maximum payload size in bytes */
#define MC_MAX_PAYLOAD       128

/* Maximum subscribers per topic */
#define MC_MAX_SUBS_PER_TOPIC 4

/* Topic definitions - add more as needed */
typedef enum {
    TOPIC_IMU_UPDATE = 0,
    TOPIC_RC_UPDATE,
    /* Add more topics here as needed:
     * TOPIC_CAN_RX,
     */
    TOPIC_COUNT
} MsgTopic;

/* Event structure passed to subscribers */
typedef struct {
    uint16_t topic;
    uint16_t size;
    uint8_t  data[MC_MAX_PAYLOAD];
} MsgEvent;

/* Subscriber callback type */
typedef void (*MsgCallback)(const MsgEvent *ev, void *user_data);

/**
 * @brief Initialize the message center
 * @param queue_len Number of events the queue can hold
 * @return 0 on success, -1 on failure
 */
int MsgCenter_Init(size_t queue_len);

/**
 * @brief Subscribe to a topic
 * @param topic Topic to subscribe to
 * @param cb Callback function (called in MsgDispatch task context)
 * @param user_data User data passed to callback
 * @return 0 on success, -1 if subscription slots full
 */
int MsgCenter_Subscribe(MsgTopic topic, MsgCallback cb, void *user_data);

/**
 * @brief Publish a message from task context
 * @param topic Topic to publish to
 * @param data Pointer to data
 * @param size Size of data (must be <= MC_MAX_PAYLOAD)
 * @param block_time_ms Time to wait if queue is full (0 = non-blocking)
 * @return 0 on success, -1 on failure
 */
int MsgCenter_Publish(MsgTopic topic, const void *data, size_t size, uint32_t block_time_ms);

/**
 * @brief Publish a message from ISR context
 * @param topic Topic to publish to
 * @param data Pointer to data
 * @param size Size of data
 * @return 0 on success, -1 on failure
 * @note Automatically triggers context switch if higher-priority task is waiting
 */
int MsgCenter_PublishFromISR(MsgTopic topic, const void *data, size_t size);

/**
 * @brief Dispatcher task entry point - called by FreeRTOS
 * @param pvParameters Unused
 * @note This function never returns
 */
void MsgCenter_DispatcherTask(void *pvParameters);

#endif /* MESSAGE_CENTER_H */
