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
set(CMAKE_PROJECT_NAME spectre2_can_usb_sensor)

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
    # Add user defined library search paths
)

# Add module sources
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # IMU
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/imu/imu.c

    # CAN (existing test-level driver + new CAN manager)
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/can/can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/can/can_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/can/motor_registry.c

    # Remote control
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/remote/ibus.c

    # Message center
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/message_center/message_center.c

    # Debug printing
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/debug_print/printing.c

    # Config
    ${CMAKE_CURRENT_SOURCE_DIR}/config/robot_config.c

    # Tests
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/tests/tests.c
)

# Add module include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/can
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/remote
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/message_center
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/debug_print
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/tests
    ${CMAKE_CURRENT_SOURCE_DIR}/config
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

    # Add user defined libraries
)
