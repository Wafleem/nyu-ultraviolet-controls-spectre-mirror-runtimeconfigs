# Architecture Overview

Our architecture draws inspiration from the foundational work by Hunan University, specifically their [basic framework](https://github.com/HNUYueLuRM/basic_framework.git).

## Three-Layer Architecture

The control system follows a three-layer architecture pattern:

1. **Application Layer** (`application/`) - High-level robot control logic
2. **Module Layer** (`modules/`) - Reusable functional modules
3. **Hardware Abstraction Layer** (`Inc/`, `Src/`) - STM32 HAL and hardware drivers

```
┌─────────────────────────────────────┐
│   Application Layer (application/) │
│   - Chassis controller              │
│   - Gimbal controller               │
│   - Shooter controller              │
│   - Command controller              │
└─────────────────────────────────────┘
              ↓ uses
┌─────────────────────────────────────┐
│   Module Layer (modules/)           │
│   - Message center (pub/sub)       │
│   - CAN communication               │
│   - IMU sensors                     │
│   - Motor drivers                   │
│   - Vision communication            │
└─────────────────────────────────────┘
              ↓ uses
┌─────────────────────────────────────┐
│   Hardware Layer (Inc/, Src/)       │
│   - STM32 HAL drivers               │
│   - Peripheral initialization       │
│   - Interrupt handlers              │
└─────────────────────────────────────┘
```

## Application Layer

The application layer contains the high-level control logic for different robot subsystems:

- **`chassis/`** - Controls the four-wheel chassis motors
- **`gimbal/`** - Controls the gimbal (pitch/yaw) motors
- **`shoot/`** - Controls the shooter mechanism
- **`cmd/`** - Central command controller that coordinates all subsystems
- **`app_subscriptions.h`** - Defines how application modules subscribe to messages

Each controller subscribes to relevant topics from the message center and publishes commands or status updates. For example, the chassis controller subscribes to:
- `TOPIC_CHASSIS_CMD` - Movement commands
- `TOPIC_IMU_UPDATE` - IMU sensor data
- `TOPIC_MOTOR_FEEDBACK` - Motor feedback data

## Module Layer

The module layer provides reusable functional components:

- **`message_center/`** - Publish-subscribe message system
- **`can_comm/`** - CAN bus communication and motor management
- **`imu/`** - IMU sensor drivers (BMI088, WT61C)
- **`motor/`** - Motor driver interfaces (GM6020)
- **`remote/`** - Remote control receiver
- **`vision_comm/`** - Vision system communication protocol
- **`algorithm/`** - Control algorithms (PID)
- **`debug_print/`** - Debug printing utilities

These modules are independent and can be used by multiple application controllers. They communicate through the message center using topics.

## Hardware Layer (Inc/ and Src/)

The `Inc/` and `Src/` directories contain STM32CubeMX-generated code for hardware initialization and HAL (Hardware Abstraction Layer) drivers. When you configure peripherals (CAN, UART, SPI, etc.) in STM32CubeMX and generate code, it creates these files.

**Only modify code between `USER CODE BEGIN` and `USER CODE END` markers!**

For example, in `Src/main.c`:

```c
/* USER CODE BEGIN Includes */
#include "message_center.h"
#include "chassis_controller.h"
// ... your includes
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
// Your initialization code here
MsgCenter_Init(128);  // Creates FreeRTOS queue with 128 events
ChassisApp_Init();
/* USER CODE END 2 */
```

If you modify code outside these markers, your changes will be **overwritten** the next time you regenerate code from STM32CubeMX.

## How It All Works Together

1. **Initialization** (`main.c`):
   - STM32CubeMX-generated code initializes hardware peripherals
   - User code initializes the message center
   - Application controllers subscribe to topics
   - Module drivers start (CAN, IMU, etc.)

2. **Runtime** (FreeRTOS tasks):

   **Three concurrent tasks run simultaneously:**

   - **defaultTask** (Priority: Normal, 3) - Handles USB device initialization, then idles
   - **MsgDispatch** (Priority: AboveNormal, 5) - High-priority event dispatcher
     - Blocks on FreeRTOS queue waiting for events
     - Wakes instantly when ISR publishes event (context switch from ISR)
     - Calls all subscriber callbacks for the topic
     - Zero CPU usage when idle (blocked on queue)
   - **Control** (Priority: Normal, 3) - Periodic command generator at 200Hz
     - Runs every 5ms using vTaskDelayUntil() for precise timing
     - Reads stored RC/IMU/vision data
     - Generates high-level commands (chassis, gimbal, shooter)
     - Publishes commands to message center

   **Event flow:**
   - Hardware interrupts (CAN RX, UART RX) publish data from ISR using `MsgCenter_PublishFromISR()`
   - ISR triggers context switch to high-priority MsgDispatch task
   - MsgDispatch calls subscriber callbacks (e.g., stores sensor data, executes motor control)
   - Control task periodically generates commands based on latest sensor data
   - Commands trigger callbacks that execute PID control and send CAN messages

3. **Communication Flow**:
   ```
   Hardware Event (e.g., CAN message received)
        ↓
   Module Layer (CAN manager publishes TOPIC_CAN_RX)
        ↓
   Message Center (dispatches to subscribers)
        ↓
   Application Layer (chassis controller processes event)
        ↓
   Module Layer (sends motor command via CAN)
        ↓
   Hardware (CAN bus sends to motors)
   ```