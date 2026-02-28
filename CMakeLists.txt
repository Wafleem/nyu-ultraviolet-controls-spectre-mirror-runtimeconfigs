cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Spectre_Controls)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/ST/ARM/DSP/Lib
)

# Add module sources
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    modules/imu/imu.c
    modules/can/can.c
    modules/can/can_manager.c
    modules/can/motor_registry.c
    modules/fifo/fifo.c
    modules/referee/referee.c
    modules/referee/ref_structs.c
    modules/referee/ref_crc.c
    modules/remote/remote_control.c
    modules/message_center/message_center.c
    modules/debug_print/printing.c
    modules/logger/logger.c
    modules/motor/motor_driver.c
    modules/algorithm/kalman_filter.c
    modules/algorithm/pid.c
    modules/algorithm/QuaternionEKF.c
    modules/vision_comm/vision_comm.c
    modules/vision_comm/crc8.c
    modules/vision_comm/crc16.c
    modules/vision_comm/seasky_protocol.c
    config/robot_config.c
    modules/tests/tests.c
)

# Add module include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    modules/imu
    modules/can
    modules/fifo
    modules/referee
    modules/remote
    modules/motor
    modules/message_center
    modules/debug_print
    modules/logger
    modules/algorithm
    modules/vision_comm
    modules/tests
    config
    Middlewares/ST/ARM/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_FREERTOS=1
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    arm_cortexM7lfsp_math
)
