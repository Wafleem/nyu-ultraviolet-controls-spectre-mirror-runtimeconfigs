/**
 * =============================================================================
 *                    MESSAGE CENTER IMPLEMENTATION
 * =============================================================================
 *
 * This file implements the FreeRTOS-based publish-subscribe message system.
 * See message_center.h for the data flow diagram and API documentation.
 *
 * ARCHITECTURE:
 *   - One FreeRTOS queue holds all pending events
 *   - One mutex protects the subscriber registry during setup
 *   - One dispatcher task processes events and calls callbacks
 *
 * =============================================================================
 */

#include "message_center.h"
#include "printing.h"
#include <string.h>


/* =============================================================================
 *                           PRIVATE DATA STRUCTURES
 * ============================================================================= */

/**
 * @brief Single subscriber entry
 */
typedef struct {
    MsgCallback cb;     // Callback function (NULL = slot empty)
    void *user;         // User data passed to callback
} Subscriber;


/* =============================================================================
 *                              PRIVATE STATE
 * ============================================================================= */

// FreeRTOS primitives
static QueueHandle_t     mc_queue      = NULL;   // The event queue
static SemaphoreHandle_t mc_subs_mutex = NULL;   // Protects subscriber registry
static size_t            mc_queue_len  = 0;      // Queue capacity

// Subscriber registry: mc_subs[topic][subscriber_index]
static Subscriber mc_subs[TOPIC_NUM_TOPICS][MC_MAX_SUBS_PER_TOPIC];

// Statistics
static MsgCenter_Stats mc_stats = {0};

// Initialization flag
static uint8_t mc_initialized = 0;


/* =============================================================================
 *                              INITIALIZATION
 * ============================================================================= */

int MsgCenter_Init(size_t queue_length)
{
    // Prevent double initialization
    if (mc_initialized) {
        return -1;
    }

    // Log memory requirements
    USB_CDC_Printf("[MsgCenter] Init: queue_len=%u, event_size=%u, total=%u bytes\r\n",
                   (unsigned)queue_length,
                   (unsigned)sizeof(MsgEvent),
                   (unsigned)(queue_length * sizeof(MsgEvent)));

    // --- Create the event queue ---
    mc_queue = xQueueCreate(queue_length, sizeof(MsgEvent));
    if (mc_queue == NULL) {
        USB_CDC_Printf("[MsgCenter] ERROR: Queue creation failed!\r\n");
        return -1;
    }
    mc_queue_len = queue_length;

    // --- Create mutex for subscriber registry ---
    mc_subs_mutex = xSemaphoreCreateMutex();
    if (mc_subs_mutex == NULL) {
        USB_CDC_Printf("[MsgCenter] ERROR: Mutex creation failed!\r\n");
        vQueueDelete(mc_queue);
        mc_queue = NULL;
        return -1;
    }

    // --- Clear subscriber registry ---
    for (size_t topic = 0; topic < TOPIC_NUM_TOPICS; topic++) {
        for (size_t slot = 0; slot < MC_MAX_SUBS_PER_TOPIC; slot++) {
            mc_subs[topic][slot].cb = NULL;
            mc_subs[topic][slot].user = NULL;
        }
    }

    // --- Clear statistics ---
    memset(&mc_stats, 0, sizeof(mc_stats));

    mc_initialized = 1;
    USB_CDC_Printf("[MsgCenter] Init complete!\r\n");
    return 0;
}


/* =============================================================================
 *                              PUBLISHING
 * ============================================================================= */

int MsgCenter_Publish(MsgTopic topic, const void *data, size_t size, uint32_t block_time_ms)
{
    // --- Validate inputs ---
    if (!mc_initialized || mc_queue == NULL) {
        return -1;
    }
    if (topic < 0 || topic >= TOPIC_NUM_TOPICS || size > MC_MAX_PAYLOAD) {
        return -2;
    }

    // --- Build the event ---
    MsgEvent ev;
    ev.topic = (uint16_t)topic;
    ev.size  = (uint16_t)size;
    if (data != NULL && size > 0) {
        memcpy(ev.data, data, size);
    }

    // --- Send to queue ---
    TickType_t ticks = (block_time_ms == portMAX_DELAY)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(block_time_ms);

    if (xQueueSend(mc_queue, &ev, ticks) == pdTRUE) {
        mc_stats.events_published++;

        // Update queue depth stats
        UBaseType_t depth = uxQueueMessagesWaiting(mc_queue);
        mc_stats.current_queue_depth = depth;
        if (depth > mc_stats.max_queue_depth) {
            mc_stats.max_queue_depth = depth;
        }
        return 0;
    } else {
        mc_stats.queue_overflow_count++;
        return -1;  // Queue full
    }
}


int MsgCenter_PublishFromISR(MsgTopic topic, const void *data, size_t size)
{
    // --- Validate inputs ---
    if (!mc_initialized || mc_queue == NULL) {
        return -1;
    }
    if (topic < 0 || topic >= TOPIC_NUM_TOPICS || size > MC_MAX_PAYLOAD) {
        return -2;
    }

    // --- Build the event ---
    MsgEvent ev;
    ev.topic = (uint16_t)topic;
    ev.size  = (uint16_t)size;
    if (data != NULL && size > 0) {
        memcpy(ev.data, data, size);
    }

    // --- Send to queue from ISR context ---
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t result = xQueueSendFromISR(mc_queue, &ev, &xHigherPriorityTaskWoken);

    // Yield to dispatcher if it has higher priority
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    if (result == pdTRUE) {
        mc_stats.events_published++;
        return 0;
    } else {
        mc_stats.queue_overflow_count++;
        return -1;  // Queue full
    }
}


/* =============================================================================
 *                              SUBSCRIBING
 * ============================================================================= */

int MsgCenter_Subscribe(MsgTopic topic, MsgCallback cb, void *user_data)
{
    // --- Validate inputs ---
    if (!mc_initialized || topic < 0 || topic >= TOPIC_NUM_TOPICS || cb == NULL) {
        return -1;
    }

    // --- Take mutex (block forever - this is only called during init) ---
    if (xSemaphoreTake(mc_subs_mutex, portMAX_DELAY) != pdTRUE) {
        return -1;
    }

    // --- Find empty slot ---
    int result = -2;  // Assume full
    for (size_t slot = 0; slot < MC_MAX_SUBS_PER_TOPIC; slot++) {
        if (mc_subs[topic][slot].cb == NULL) {
            mc_subs[topic][slot].cb = cb;
            mc_subs[topic][slot].user = user_data;
            result = 0;
            break;
        }
    }

    xSemaphoreGive(mc_subs_mutex);
    return result;
}


/* =============================================================================
 *                           DISPATCHER TASK
 * ============================================================================= */

void MsgCenter_DispatcherTask(void *pvParameters)
{
    (void)pvParameters;

    // --- Sanity check ---
    if (!mc_initialized || mc_queue == NULL) {
        USB_CDC_Printf("[MsgCenter] FATAL: Dispatcher started but not initialized!\r\n");
        vTaskDelete(NULL);
        return;
    }

    USB_CDC_Printf("[MsgCenter] Dispatcher task running\r\n");

    MsgEvent ev;

    // --- Main dispatch loop ---
    for (;;) {
        // Block until an event arrives (zero CPU when idle)
        if (xQueueReceive(mc_queue, &ev, portMAX_DELAY) == pdTRUE) {

            mc_stats.events_dispatched++;
            mc_stats.current_queue_depth = uxQueueMessagesWaiting(mc_queue);

            // Validate topic
            if (ev.topic >= TOPIC_NUM_TOPICS) {
                continue;  // Invalid topic, skip
            }

            // --- Invoke all subscribers for this topic ---
            MsgTopic t = (MsgTopic)ev.topic;
            for (size_t slot = 0; slot < MC_MAX_SUBS_PER_TOPIC; slot++) {
                if (mc_subs[t][slot].cb != NULL) {
                    mc_subs[t][slot].cb(&ev, mc_subs[t][slot].user);
                }
            }
        }
    }
}


/* =============================================================================
 *                              STATISTICS
 * ============================================================================= */

void MsgCenter_GetStats(MsgCenter_Stats *stats)
{
    if (stats != NULL && mc_initialized) {
        stats->events_published     = mc_stats.events_published;
        stats->events_dispatched    = mc_stats.events_dispatched;
        stats->queue_overflow_count = mc_stats.queue_overflow_count;
        stats->current_queue_depth  = mc_stats.current_queue_depth;
        stats->max_queue_depth      = mc_stats.max_queue_depth;
    }
}
