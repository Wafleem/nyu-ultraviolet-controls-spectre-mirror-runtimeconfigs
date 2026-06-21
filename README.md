# Spectre Controls - Runtime Robot Configuration

**NYU RoboMaster Team Spectre** | STM32H7 embedded controls firmware

One binary runs on every robot. The robot discovers its identity at runtime through the RoboMaster referee system serial protocol, then reconfigures its entire control stack — motors, PID gains, shooter parameters, vision targeting, HUD graphics, and supercap power limits — without reflashing.

Licensed under the [MIT License](LICENSE).

---

## How Runtime Configuration Works

### The Problem

We field multiple robot types (Hero, Standard, Sentry) with different motor layouts, shooter mechanisms, and power systems. Maintaining separate firmware per robot is error-prone and slows iteration.

### The Solution

A single firmware binary handles all robot types. The referee system tells the robot what it is, and the firmware reconfigures on the fly.

### The Flow

```
Referee serial frame (cmd 0x0201)
  → ref_structs.c parses robot_status_t, publishes TOPIC_ROBOT_STATUS
    → freertos.c on_robot_status() detects robot_id change
      → configure_robot() calls RobotConfig_Get(robot_id)
        → robot_config.c switch maps robot_id → static config struct
          → All modules re-initialize with new config
```

---

## Key Files

### 1. Configuration Definitions

| File | What It Does |
|------|-------------|
| [`config/config_types.h`](config/config_types.h) | Defines `RobotConfig_t`, `MotorConfig_t`, `PIDParams_t`, and enums for motor types, roles, CAN channels, yaw sources, and power limit sources |
| [`config/robot_config.c`](config/robot_config.c) | `RobotConfig_Get()` — maps `robot_id_t` to a static config pointer via switch statement |
| [`config/robot_config.h`](config/robot_config.h) | Declaration of `RobotConfig_Get()` |
| [`config/hero.h`](config/hero.h) | Hero config: 10 motors, GM6020 direct-drive yaw, 42mm pusher shooter, `supercap_limit = 100W` |
| [`config/standard_2026.h`](config/standard_2026.h) | Standard 2026 config: 9 motors, M3508 belt-driven yaw (asymmetric scaling), 17mm indexer shooter, `supercap_limit = 75W` |
| [`config/unknown.h`](config/unknown.h) | Fallback config: 0 motors, returned before referee connects |

### 2. Robot Identity & Referee Parsing

| File | What It Does |
|------|-------------|
| [`modules/referee/ref_structs.h`](modules/referee/ref_structs.h) | `robot_id_t` enum (RED_HERO=1 through BLUE_BASE=111), `robot_status_t` struct, HUD drawing types |
| [`modules/referee/ref_structs.c`](modules/referee/ref_structs.c) | Parses referee serial frames. On cmd `0x0201`, publishes `TOPIC_ROBOT_STATUS` (line 189) triggering reconfiguration. Also contains HUD drawing functions that use `robot_id` to set sender/receiver IDs for the operator client |

### 3. Reconfiguration Entry Point

| File | What It Does |
|------|-------------|
| [`Core/Src/freertos.c`](Core/Src/freertos.c) | `configure_robot()` (line 475) — the runtime reconfiguration function. Tears down and rebuilds motor registries, motor driver, chassis, gimbal, shooter, command controller, ToF sensor config, and sends the supercap power limit over CAN. `StartControlTask()` (line 307) runs the 200Hz control loop: cmd → chassis → gimbal → shooter → flush CAN |

### 4. Config-Dependent Modules

#### Shooter (`application/shoot/shooter_controller.c`)
- `ShooterApp_Init()` (line 450) copies the `RobotConfig_t` for feeder speed, friction wheel speed, and pusher angles
- **Hero** uses 42mm projectiles with a pusher state machine (`pusher_retracted_angle` / `pusher_extended_angle` from config) — each shot costs 100 heat
- **Standard** uses 17mm projectiles with a continuous indexer — each shot costs 10 heat
- Heat management (line 186) branches on `robot_id == RED_HERO || BLUE_HERO` to pick the right barrel heat sensor and buffer calculation

#### Vision Communication (`modules/vision_comm/vision_comm.c`)
- `on_robot_status()` (line 117) derives the enemy team color from `robot_id`: IDs >= 100 are blue team, so target red; otherwise target blue
- `VisionComm_Send()` packs the enemy color into a SeaSky protocol flags register, sent to the Jetson at 100Hz over UART
- The Jetson uses this to filter detected armor plates by color — no hardcoded team allegiance

#### SeaSky Protocol (`modules/vision_comm/seasky_protocol.c`)
- Wire protocol for STM32 ↔ Jetson: framing with CRC8 header check + CRC16 payload check
- Carries yaw/pitch angles, enemy color flag, and auto-aim commands

#### Supercap Power Limit (`modules/can/can_manager.c`, `config/`)
- Each robot config defines a `supercap_limit` in watts — Hero gets `100W`, Standards get `75W`
- When `configure_robot()` runs (line 499), it calls `CAN_Manager_SendSupercapChargeLimit(robot_cfg->supercap_limit)`
- This sends the per-robot power ceiling to the Wraith supercap board over CAN (standard ID `0x408`, 4-byte float LE)
- The Wraith board clamps its charging current to stay under this ceiling — different robots get different power budgets based on weight class and strategy
- The chassis controller also uses `power_limit_source` from config to decide whether it reads power feedback from the supercap board or the PDB

#### HUD / Operator UI (`modules/referee/ref_structs.c`)
- `build_hud_data_for_robot()` (line 390) and `build_hud_text_for_robot()` (line 463) construct HUD draw commands using `robot_id` as `sender_id` and `robot_id + 0x100` as `receiver_id`
- The referee system routes HUD graphics to the correct operator client based on these IDs — same code, different robots, correct operator screen

#### Chassis Power Limiting (`application/chassis/chassis_controller.c`)
- `power_limit_source` in `RobotConfig_t` selects whether the chassis reads power feedback from the supercap board (`POWER_LIMIT_SOURCE_SUPERCAP`) or PDB (`POWER_LIMIT_SOURCE_PDB`)
- Hero uses PDB (no supercap mounted yet); Standards use supercap feedback
- Speed limit toggles between `CHASSIS_SPEED_NORMAL` (6500 RPM) and `CHASSIS_SPEED_SUPERCAP` (8500 RPM) based on discharge state

---

## Hero vs Standard: What Changes at Runtime

| Aspect | Hero | Standard 2026 |
|--------|------|---------------|
| Motor count | 10 | 9 |
| Yaw drive | GM6020 direct | M3508 belt (asymmetric: L=1.2x, R=0.8x) |
| Yaw source | GM6020 encoder | Dev C IMU |
| Shooter type | 42mm pusher (feeder + friction + push motor) | 17mm indexer (feeder + friction) |
| Feeder speed | 4000 | 3000 |
| Friction speed | 10000 | 7000 |
| Heat tracking | `shooter_heat_42mm`, 100 heat/shot buffer | `shooter_heat_17mm`, 50 heat buffer (5 shots) |
| Supercap limit | 100W | 75W |
| Power limit source | PDB | Supercap |
| Startup alignment | Yes | Yes |

All of this is defined in the config header files and takes effect through `configure_robot()` — zero conditional compilation, zero reflashing.

---

## Build Instructions

### Prerequisites

1. **ARM GNU Toolchain** (`arm-none-eabi-gcc`)
   - Download from [Arm Developer](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
   - Add the `bin/` directory to your system `PATH`
   - Verify: `arm-none-eabi-gcc --version`

2. **CMake** (>= 3.22)
   - Download from [cmake.org](https://cmake.org/download/) or install via your package manager
   - Verify: `cmake --version`

3. **Ninja** (recommended) or Make
   - Download from [ninja-build.org](https://ninja-build.org/) or install via your package manager
   - Verify: `ninja --version`

### Building

```bash
# Configure (from project root)
cmake --preset default
# Or manually:
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

# Build
cmake --build build
```

The output binary is `build/Spectre_Controls.elf`.

### Flashing

**Option A: STM32CubeProgrammer (GUI)**
1. Connect the STM32H7 board via USB (DFU mode: hold BOOT0 during reset)
2. Open STM32CubeProgrammer, select USB, connect
3. Load `build/Spectre_Controls.elf`, click "Start Programming"

**Option B: dfu-util (CLI)**
```bash
dfu-util -a 0 -s 0x08000000:leave -D build/Spectre_Controls.bin
```

**Option C: OpenOCD + ST-Link**
```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/Spectre_Controls.elf verify reset exit"
```

### Running

Once flashed, the firmware boots into the `unknown` configuration (0 motors). Connect the referee system serial link, and the robot will automatically reconfigure when it receives its robot ID assignment. No additional setup needed — the runtime configuration is fully automatic.

---

## Hardware Target: Spectre Board

This firmware is built for our custom **Spectre board**, an STM32H743-based controller designed in-house by our electrical team. The `Spectre_Controls.ioc` CubeMX project, the linker script (`STM32H743XX_FLASH.ld`), and all HAL initialization in `Core/` are generated specifically for this board's pin assignments, clock tree, and peripheral mapping (which FDCAN buses, which UARTs, which SPI/I2C, etc.).

### Porting to a Different Board

The runtime configuration system (`config/`, `application/`, and most of `modules/`) is hardware-agnostic — it operates on top of the HAL, not directly on registers. If you want to run this logic on a different STM32 board (e.g. a RoboMaster Type-A/C board, or your own custom PCB):

1. **Regenerate the HAL** — Create a new CubeMX `.ioc` project for your target MCU. Configure the same peripherals (2x FDCAN, UART for referee, UART for vision, SPI for IMU, etc.) mapped to your board's actual pins.
2. **Audit the module layer** — The modules in `modules/` call HAL functions like `HAL_FDCAN_AddMessageToTxFifoQ()`, `HAL_UART_Receive_DMA()`, etc. with handles (`hfdcan1`, `huart1`, ...) that are tied to the CubeMX-generated peripheral instances. You'll need to go through each module and update these handles to match your new HAL configuration. The application layer (`application/`) should not need changes.
3. **Update the linker script** — Replace `STM32H743XX_FLASH.ld` with one matching your target MCU's flash/RAM layout.

The key point: the config system and control logic are portable. The HAL plumbing is not.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────┐
│  Application Layer                              │
│  cmd_controller  chassis  gimbal  shooter       │
│  (200Hz control loop in StartControlTask)       │
├─────────────────────────────────────────────────┤
│  Module Layer                                   │
│  motor_driver  referee  vision_comm  CAN  IMU   │
│  message_center (pub/sub)                       │
├─────────────────────────────────────────────────┤
│  Hardware Layer                                 │
│  STM32 HAL  FreeRTOS  FDCAN  UART  SPI  USB    │
├─────────────────────────────────────────────────┤
│  Config Layer                                   │
│  config_types.h  robot_config.c  hero.h  etc.   │
│  (static const structs, selected at runtime)    │
└─────────────────────────────────────────────────┘
```

The message center provides publish-subscribe decoupling between layers. When the referee parser publishes `TOPIC_ROBOT_STATUS`, subscribers in freertos.c, shooter_controller.c, and vision_comm.c each react independently.

---

## Repository Layout

- `config/` — Robot configuration structs and the `RobotConfig_Get()` lookup
- `application/` — High-level controllers (chassis, gimbal, shooter, command)
- `modules/` — Hardware abstraction and protocol drivers (CAN, referee, vision, motors, IMU)
- `Core/` — STM32CubeMX-generated code + FreeRTOS task entry points
- `Drivers/`, `Middlewares/` — Vendor libraries (STM32 HAL, FreeRTOS, USB)
- `cmake/` — CMake toolchain files for cross-compilation
- `docs/` — Additional documentation (setup guide, architecture, CAN protocol, vision protocol)

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
