/**
 * @file message_center.c
 * @brief FreeRTOS queue-based pub-sub message center implementation
 */
#include "message_center.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

/* Subscription entry */
typedef struct {
    MsgCallback cb;
    void *user;
} Subscription;

/* Static storage */
static QueueHandle_t mc_queue = NULL;
static Subscription mc_subs[TOPIC_COUNT][MC_MAX_SUBS_PER_TOPIC];

int MsgCenter_Init(size_t queue_len)
{
    /* Create the event queue */
    mc_queue = xQueueCreate(queue_len, sizeof(MsgEvent));
    if (mc_queue == NULL) {
        return -1;
    }

    /* Clear all subscriptions */
    memset(mc_subs, 0, sizeof(mc_subs));

    return 0;
}

int MsgCenter_Subscribe(MsgTopic topic, MsgCallback cb, void *user_data)
{
    if (topic >= TOPIC_COUNT || cb == NULL) {
        return -1;
    }

    /* Find empty slot */
    for (size_t i = 0; i < MC_MAX_SUBS_PER_TOPIC; i++) {
        if (mc_subs[topic][i].cb == NULL) {
            mc_subs[topic][i].cb = cb;
            mc_subs[topic][i].user = user_data;
            return 0;
        }
    }

    return -1; /* No slots available */
}

int MsgCenter_Publish(MsgTopic topic, const void *data, size_t size, uint32_t block_time_ms)
{
    if (mc_queue == NULL || topic >= TOPIC_COUNT || size > MC_MAX_PAYLOAD) {
        return -1;
    }

    MsgEvent ev;
    ev.topic = (uint16_t)topic;
    ev.size = (uint16_t)size;
    memcpy(ev.data, data, size);

    TickType_t ticks = (block_time_ms == portMAX_DELAY)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(block_time_ms);

    if (xQueueSend(mc_queue, &ev, ticks) == pdTRUE) {
        return 0;
    }
    return -1;
}

int MsgCenter_PublishFromISR(MsgTopic topic, const void *data, size_t size)
{
    if (mc_queue == NULL || topic >= TOPIC_COUNT || size > MC_MAX_PAYLOAD) {
        return -1;
    }

    MsgEvent ev;
    ev.topic = (uint16_t)topic;
    ev.size = (uint16_t)size;
    memcpy(ev.data, data, size);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xQueueSendFromISR(mc_queue, &ev, &xHigherPriorityTaskWoken) == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return 0;
    }
    return -1;
}

void MsgCenter_DispatcherTask(void *pvParameters)
{
    (void)pvParameters;
    MsgEvent ev;

    for (;;) {
        /* Block until an event arrives */
        if (xQueueReceive(mc_queue, &ev, portMAX_DELAY) == pdTRUE) {
            /* Call all subscribers for this topic */
            if (ev.topic < TOPIC_COUNT) {
                for (size_t i = 0; i < MC_MAX_SUBS_PER_TOPIC; i++) {
                    if (mc_subs[ev.topic][i].cb != NULL) {
                        mc_subs[ev.topic][i].cb(&ev, mc_subs[ev.topic][i].user);
                    }
                }
            }
        }
    }
}
