cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

include(pico_extras_import_optional.cmake)

project(autopilot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        -O2
        )

if (TARGET tinyusb_device)
    add_executable(autopilot
            autopilot.cpp
            LoopRate.cpp LoopRate.hpp IMU.cpp IMU.hpp EKF.cpp EKF.hpp Datalogger.cpp Datalogger.hpp)

    # pull in common dependencies
    target_link_libraries(autopilot pico_stdlib hardware_i2c)

    target_include_directories(autopilot PRIVATE
            /Users/sammy/Dropbox/robotics/autopilot/eigen/_install/include)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(autopilot 1)
    pico_enable_stdio_uart(autopilot 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(autopilot)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building autopilot because TinyUSB submodule is not initialized in the SDK")
endif()

