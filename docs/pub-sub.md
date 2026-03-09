# Publish-Subscribe System (FreeRTOS Implementation)

## What is Pub-Sub?

The publish-subscribe pattern is like a radio station:
- **Publishers** broadcast messages on specific **topics** (like radio channels)
- **Subscribers** tune in to topics they care about and receive messages automatically
- Publishers and subscribers don't need to know about each other directly

This makes the code more modular - different modules can communicate without tight coupling.

## Message Center Overview

The message center (`modules/message_center/`) is the central hub for all inter-module communication. It uses **FreeRTOS queues** for zero-CPU-idle, event-driven message dispatch with a dedicated high-priority task.

### Key Components

1. **Topics** - Different types of messages (like IMU data, motor feedback, commands)
2. **Events** - The actual message data wrapped in a structure
3. **Subscribers** - Callback functions that get called when a topic is published
4. **FreeRTOS Queue** - Thread-safe queue that stores pending messages
5. **Dispatcher Task** - High-priority FreeRTOS task that processes events

## Available Topics

All topics are defined in `message_center_rtos.h`:

```c
typedef enum {
    TOPIC_RC_UPDATE = 0,        // Remote control data
    TOPIC_IMU_UPDATE,           // IMU sensor data (gyro, accel)
    TOPIC_CAN_RX,               // Raw CAN messages received
    TOPIC_MOTOR_FEEDBACK,       // Motor feedback (M3508 chassis motors)
    TOPIC_GM6020_FEEDBACK,      // GM6020 motor feedback (gimbal)
    TOPIC_CHASSIS_CMD,          // Chassis movement commands
    TOPIC_SHOOT_CMD,            // Shooter commands
    TOPIC_GIMBAL_CMD,           // Gimbal commands
    TOPIC_BARREL_HEAT,          // Barrel heat data
    TOPIC_ROBOT_STATUS,         // Robot status data
    TOPIC_SHOOT_DATA,           // Shoot data
    TOPIC_PROJECTILE_ALLOWANCE, // Projectile allowance data
    TOPIC_VISION_DATA,          // Vision system data
    TOPIC_NUM_TOPICS
} MsgTopic;
```

## How to Publish a Message

The message center provides **two publish functions** depending on context:

### Publishing from ISRs (Interrupt Handlers)

Use `MsgCenter_PublishFromISR()` when publishing from interrupt handlers (CAN RX, UART RX, etc.):

```c
// Example: Publishing IMU data from UART ISR
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1) {  // WT61C IMU
        // Parse IMU data
        SensorData sensor_data;
        // ... fill sensor_data with readings ...

        // Publish from ISR context
        MsgCenter_PublishFromISR(TOPIC_IMU_UPDATE, &sensor_data, sizeof(SensorData));
        // This automatically triggers context switch to dispatcher task if higher priority!
    }
}
```

**Key points:**
- Automatically calls `portYIELD_FROM_ISR()` to trigger immediate context switch
- Non-blocking - returns immediately
- High-priority dispatcher task wakes up within microseconds
- **NEVER** use `MsgCenter_Publish()` from ISR - system will crash!

### Publishing from Tasks

Use `MsgCenter_Publish()` when publishing from FreeRTOS task context:

```c
// Example: Publishing chassis command from Control task
void CmdController_Task(uint32_t current_tick) {
    // ... process RC input and generate command ...

    ChassisCmd chassis_cmd;
    chassis_cmd.vx = vx_normalized;
    chassis_cmd.vy = vy_normalized;
    chassis_cmd.wz = wz_normalized;
    chassis_cmd.enabled = true;

    // Publish from task context (last param: block time in ms)
    MsgCenter_Publish(TOPIC_CHASSIS_CMD, &chassis_cmd, sizeof(ChassisCmd), 0);
    // block_time = 0: non-blocking
    // block_time = portMAX_DELAY: wait forever if queue full
}
```

**Parameters:**
- `topic` - Which topic to publish on
- `data` - Pointer to data to send
- `size` - Size of data in bytes (max 128 bytes)
- `block_time_ms` - How long to wait if queue is full
  - `0` - Non-blocking (returns immediately if queue full)
  - `portMAX_DELAY` - Wait forever until space available
  - Any value in ms - Wait up to that time

**Return value:** `0` on success, `-1` on failure (queue full or invalid params)

### What Happens When You Publish

1. Message is packaged into a `MsgEvent` structure
2. Event is added to FreeRTOS queue using `xQueueSend()` or `xQueueSendFromISR()`
3. If published from ISR, `portYIELD_FROM_ISR()` triggers immediate context switch
4. High-priority dispatcher task wakes up (was blocked on `xQueueReceive()`)
5. Dispatcher calls all subscriber callbacks
6. Callbacks execute in **dispatcher task context** (not ISR!)

**Latency:** <1ms from ISR publish to callback execution

## How to Subscribe to a Topic

To receive messages on a topic, subscribe with a callback function:

```c
// Callback function that gets called when TOPIC_CHASSIS_CMD is published
static void on_chassis_cmd(const MsgEvent *ev, void *user_data) {
    (void)user_data;  // user_data is optional, can be NULL

    if (ev->size == sizeof(ChassisCmd)) {
        ChassisCmd cmd;
        memcpy(&cmd, ev->data, sizeof(ChassisCmd));

        // Process the command
        ChassisController_Update(&s_ctrl, &cmd);
        ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
    }
}

// Subscribe in initialization code
void ChassisApp_Init(void) {
    MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
}
```

### Callback Function Signature

```c
typedef void (*MsgCallback)(const MsgEvent *ev, void *user_data);
```

- **`ev`** - The message event containing topic, size, and data
- **`user_data`** - Optional pointer you can pass when subscribing (can be NULL)

**Important:** Callbacks run in the **dispatcher task context** (priority 5), not in ISR context. This means:
- ✅ You can call FreeRTOS functions (`osDelay`, `MsgCenter_Publish`, etc.)
- ✅ You can access shared resources safely
- ⚠️ **Keep callbacks fast** (<1ms) to avoid blocking other event processing
- ❌ Don't use long delays or blocking operations in callbacks

## Message Event Structure

Each message is wrapped in a `MsgEvent`:

```c
typedef struct {
    uint16_t topic;                      // Which topic this message is for
    uint16_t size;                       // Size of data in bytes
    uint8_t  data[MC_MAX_PAYLOAD];       // The actual data (max 128 bytes)
} MsgEvent;
```

**Note:** In FreeRTOS implementation, `MC_MAX_PAYLOAD = 128` bytes (increased from 64 in bare-metal).

## Message Dispatch (Automatic via FreeRTOS Task)

Messages are dispatched automatically by a dedicated **FreeRTOS task** - no manual dispatch needed!

### The Dispatcher Task

**File:** `modules/message_center/message_center_rtos.c:157`

```c
void MsgCenter_DispatcherTask(void *pvParameters) {
    MsgEvent ev;

    for (;;) {
        // Block until event available (zero CPU waste!)
        xQueueReceive(mc_queue, &ev, portMAX_DELAY);

        // Get topic
        MsgTopic t = (MsgTopic)ev.topic;

        // Find all subscribers for this topic
        for (size_t i = 0; i < MC_MAX_SUBS_PER_TOPIC; ++i) {
            if (mc_subs[t][i].cb) {
                // Call subscriber callback
                mc_subs[t][i].cb(&ev, mc_subs[t][i].user);
            }
        }
    }
}
```

**Task Configuration:**
- **Priority:** AboveNormal (5) - **HIGHER** than control task (3)
- **Stack:** 2KB (512 words × 4 bytes)
- **Behavior:** Blocks on empty queue (zero CPU usage when idle)

**Why this works:**
- ISR publishes event → context switch → dispatcher wakes up immediately
- High priority ensures low latency (<1ms)
- Blocks when idle instead of polling (zero CPU waste)
- All callbacks run in safe task context (not ISR)

## Thread Safety

The FreeRTOS message center uses queue primitives for thread safety:

### Queue Operations (Thread-Safe)
- **Publishing** (from ISRs): `xQueueSendFromISR()` - inherently thread-safe
- **Publishing** (from tasks): `xQueueSend()` - inherently thread-safe
- **Dispatching**: `xQueueReceive()` - inherently thread-safe

### Subscriber List Protection
- Protected by FreeRTOS mutex (`mc_subs_mutex`)
- Safe to subscribe from any task during initialization
- Typically only subscribe once during `*App_Init()`

**No manual interrupt disabling needed** - FreeRTOS handles all synchronization!

## Real-World Example: Chassis Controller

Let's see how the chassis controller uses pub-sub with FreeRTOS:

```c
// 1. Subscribe to topics in initialization
void ChassisApp_Init(void) {
    ChassisController_Init(&s_ctrl);

    // Subscribe to chassis commands (published by Control task)
    MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);

    // Subscribe to IMU updates (published from UART ISR)
    MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

    // Subscribe to motor feedback (published from CAN RX ISR)
    MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
}

// 2. Callback for chassis commands (runs in MsgDispatch task, priority 5)
static void on_chassis_cmd(const MsgEvent *ev, void *user) {
    if (ev->size == sizeof(ChassisCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));

        // Execute motor control immediately
        ChassisController_Update(&s_ctrl, &s_last_sensor);
        ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
        // This sends CAN commands to motors!
    }
}

// 3. Callback for IMU updates (runs in MsgDispatch task)
static void on_imu_update(const MsgEvent *ev, void *user) {
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
        // Just store data - will be used next time chassis command arrives
    }
}

// 4. Callback for motor feedback (runs in MsgDispatch task)
static void on_motor_feedback(const MsgEvent *ev, void *user) {
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
        if (m->motor_id < 4) {  // Only process chassis motors
            // Update motor context with fresh feedback
            // Used by PID controller in next control cycle
        }
    }
}
```

### Event Flow

```
Time   Event                                     Task/ISR           Context
----   -----                                     --------           -------
0ms    UART1 receives IMU data                  ISR                Interrupt
       └─ MsgCenter_PublishFromISR(IMU)         ISR                (20μs)
       └─ portYIELD_FROM_ISR()                  ISR                Context switch!

0.02ms MsgDispatch task wakes up               MsgDispatch (P5)   Task
       └─ on_imu_update() called                MsgDispatch (P5)   (5μs)
       └─ Block on queue again                  MsgDispatch (P5)   Sleeping

5ms    Control task wakes (200Hz timer)        Control (P3)       Task
       └─ CmdController_Task()                  Control (P3)       (50μs)
       └─ MsgCenter_Publish(CHASSIS_CMD)        Control (P3)       Queue write

5.05ms MsgDispatch wakes (queue not empty)     MsgDispatch (P5)   Preempts Control!
       └─ on_chassis_cmd() called               MsgDispatch (P5)   (100μs)
         └─ PID control                         MsgDispatch (P5)
         └─ Send CAN commands                   MsgDispatch (P5)
       └─ Block on queue                        MsgDispatch (P5)   Sleeping

5.15ms Control task resumes                    Control (P3)       Task
       └─ vTaskDelayUntil(10ms)                 Control (P3)       Sleeping
```

## Performance Characteristics

### Latency
- **ISR → Callback:** <1ms (typically 20-50μs)
- **Task → Callback:** <1ms (depends on queue depth)

### CPU Usage
- **Dispatcher task:** ~0% when idle (blocked on queue)
- **Control task:** ~1% (50μs every 5ms)
- **Total overhead:** <2%

### Memory Usage
- **Queue:** 128 events × 132 bytes = ~16KB
- **Dispatcher stack:** 2KB
- **Total:** ~18KB

## Limitations

1. **Maximum payload size**: 128 bytes (`MC_MAX_PAYLOAD`)
   - Larger messages will be rejected
   - Check return value of publish functions

2. **Maximum subscribers per topic**: 8 (`MC_MAX_SUBS_PER_TOPIC`)
   - Typically more than enough for robot control
   - Can be increased in `message_center_rtos.h`

3. **Queue depth**: 128 events (set in `MsgCenter_Init()`)
   - If queue fills, publish will fail (task) or drop event (ISR)
   - Monitor with `MsgCenter_GetStats()` to check overflow

4. **Callback execution time**: Keep <1ms
   - Callbacks run in high-priority dispatcher task
   - Long callbacks block other event processing
   - For long operations, publish another event to trigger async processing

## Debugging Tips

### Check Queue Statistics

```c
MsgCenter_Stats stats;
MsgCenter_GetStats(&stats);

USB_CDC_Printf("Events published: %lu\n", stats.events_published);
USB_CDC_Printf("Events dispatched: %lu\n", stats.events_dispatched);
USB_CDC_Printf("Queue depth: %lu / max %lu\n",
               stats.current_queue_depth, stats.max_queue_depth);
USB_CDC_Printf("Overflows: %lu\n", stats.queue_overflow_count);
```

### Common Issues

**Queue overflow:**
- `queue_overflow_count > 0` means events were dropped
- Increase queue size in `MsgCenter_Init()`
- Or reduce publish frequency

**Slow event processing:**
- Check `max_queue_depth` - if near limit, callbacks are too slow
- Profile callback execution time
- Move long operations out of callbacks

**Missing events:**
- Verify subscription was called during `*App_Init()`
- Check callback function signature matches `MsgCallback`
- Ensure payload size matches expected struct

## Migration from Bare-Metal

If you have old bare-metal code using `MsgCenter_Dispatch()`:

**Old (Bare-Metal):**
```c
// main.c - OLD CODE
while (1) {
    MsgCenter_Dispatch();  // Manual polling
    HAL_Delay(5);
}
```

**New (FreeRTOS):**
```c
// Src/freertos.c - NEW CODE
// No manual dispatch needed!
// Dispatcher task runs automatically
osKernelStart();  // FreeRTOS handles everything
```

**Changes needed:**
1. ✅ Remove `MsgCenter_Dispatch()` from main loop
2. ✅ Use `MsgCenter_Init(128)` instead of `MsgCenter_Init(buffer, size)`
3. ✅ Use `MsgCenter_PublishFromISR()` in interrupt handlers
4. ✅ Use `MsgCenter_Publish()` with block time parameter in tasks
5. ✅ Ensure FreeRTOS dispatcher task is created in `MX_FREERTOS_Init()`

**That's it!** The pub-sub pattern and callback signatures remain identical.
