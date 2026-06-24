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
set(CMAKE_PROJECT_NAME Flight_Hal)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    FreeRTOS/croutine.c
    FreeRTOS/event_groups.c
    FreeRTOS/list.c
    FreeRTOS/queue.c
    FreeRTOS/stream_buffer.c
    FreeRTOS/tasks.c
    FreeRTOS/timers.c
    FreeRTOS/portable/port.c
    FreeRTOS/portable/heap_4.c
    Common/common_debug.c
    Application/Application.c
    Interface/Int_IP5305T.c
    Interface/Int_Motor.c
    Interface/Int_LED.c
    Interface/Int_SI24R1.c
    Common/common_config.c
    Application/Application_receive.c
    Interface/Int_MPU6050.c
    Application/Application_flight.c
    Common/common_filter.c
    Common/common_imu.c
    Common/common_pid.c
    Interface/fix_height/VL53L1X_calibration.c
    Interface/fix_height/VL53L1X_api.c
    Interface/fix_height/vl53l1_platform.c
    Interface/Int_VL53L1X.c
    Interface/Int_BAT_ADC.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    FreeRTOS/include
    FreeRTOS/portable
    FreeRTOS
    Common
    Application
    Interface
    Interface/fix_height
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
