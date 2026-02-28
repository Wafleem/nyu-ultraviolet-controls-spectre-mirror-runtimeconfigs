# RoboMaster Control

STM32H7 project for the Spectre board. 

- Quick start: see the [Setup Guide](docs/tutorials/setup-guide.md)
- Toolchain: CMake + Ninja, ARM GNU Toolchain, STM32CubeProgrammer
- Target: STM32H7

## Documentation

- **[Setup Guide](docs/tutorials/setup-guide.md)** - Getting started with development environment setup
- **[Architecture Overview](docs/architecture.md)** - Three-layer architecture (application, module, hardware)
- **[Publish-Subscribe System](docs/pub-sub.md)** - Message center and event-driven communication
- **[Vision Protocol](docs/vision-protocol.md)** - Seasky protocol for vision system communication
- **[CAN Communication](docs/tutorials/can.md)** - CAN bus protocol and motor communication

## Prerequisites
- GNU Arm Embedded Toolchain (`gcc-arm-none-eabi`) — matching version used for the project
- CMake (>= 3.15)
- Ninja or Make (Ninja recommended)

## Repository layout

- `Inc/`, `Src/` — application sources and headers
- `Drivers/`, `Middlewares/` — vendor libraries (STM32 HAL, USB device)
- `cmake/`, `CMakeLists.txt` — CMake configuration
- `Debug/`, `build/` — build outputs (generated)
- `docs/` — guides and reference

### Notes on repository contents
- Generated build outputs (object files, .elf, .bin, `build/`) are ignored via `.gitignore` so clones can build locally.
- The `cmake/` toolchain file is included so others can reproduce the build configuration.
- If you want to include linker maps or `compile_commands.json` in the repo for debugging or tooling, remove them from `.gitignore` before committing.

## Opening in STM32CubeMX
Open `Spectre_Controls.ioc` in STM32CubeMX to inspect or regenerate peripheral configuration. Be careful when regenerating project files — keep track of intentional local changes. Remember to edit code in the sections marked user code in order to avoid cubemx overwriting it if you change something and regenerate the code

## Build & flash (summary)

### Build
- Use CMake Tools in VS Code (see Setup Guide Step 5)
- Or use VSCode tasks: `Ctrl+Shift+B` / `Cmd+Shift+B`

### Flash
Two methods available:

1. **STM32CubeProgrammer** (GUI method)
   - Use STM32CubeProgrammer via USB (see Setup Guide Step 6)

2. **VSCode Tasks** (command-line method)
   - `flash: dfu-util` - Flash via USB using DFU mode (default build task)
   - `flash: openocd (stlink)` - Flash using ST-Link debugger
   - See Setup Guide Step 7 for detailed instructions
