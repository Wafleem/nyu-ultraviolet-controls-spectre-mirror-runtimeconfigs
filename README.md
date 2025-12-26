# spectre2_can_usb_sensors

STM32H7 project for the Spectre rev2 sensors board. This repository contains the MCU firmware sources, CMake build files and a linker script. Currently, all relevant testing functions/code are in main.c, and the important config/lower level code for things like imu or can are in imu.c/h can.c/h respectively.

## What’s here
- All files in here were the default output of CubeMX upon code generation.
- See main.c for current user code. It has a serial test for the CAN (in the form of a loopback, where you connect CAN1 and CAN2) as well as the readout for the IMU sensor values. Use a serial moniter of your choice to view the results.

## Prerequisites
- GNU Arm Embedded Toolchain (`gcc-arm-none-eabi`) — matching version used for the project
- CMake (>= 3.15)
- Ninja or Make (Ninja recommended)

## Flashing / Programming

- You can just use STM32 Cube Programmer. Build with the tools of your choice (I use cmake extension in vscode).

## Notes on repository contents
- Generated build outputs (object files, .elf, .bin, `build/`) are ignored via `.gitignore` so clones can build locally.
- The `cmake/` toolchain file is included so others can reproduce the build configuration.
- If you want to include linker maps or `compile_commands.json` in the repo for debugging or tooling, remove them from `.gitignore` before committing.

## Opening in STM32CubeMX
Open `spectre2_can_usb_sensor.ioc` in STM32CubeMX to inspect or regenerate peripheral configuration. Be careful when regenerating project files — keep track of intentional local changes. Remember to edit code in the sections marked user code in order to avoid cubemx overwriting it if you change something and regenerate the code
