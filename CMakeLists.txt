cmake_minimum_required(VERSION 3.12)


# Project name
set(NAME rp2040_glitch)

# Retrieve the value of the PICO_SDK_PATH environment variable
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
if(NOT EXISTS ${PICO_SDK_PATH})
  message(FATAL_ERROR "PICO_SDK_PATH does not point to a valid directory. Please set it to the path of your Pico SDK installation. For example: /usr/share/pico-sdk")
endif()

if(NOT DEFINED ENV{PICO_BOARD})
  message(FATAL_ERROR "PICO_BOARD environment variable is not set. Please set it to the desired board type. You can find the names of the board in you ${PICO_SDK_PATH}/src/boards/include/boards/. For example: PICO_BOARD=pico for the standard pico or PICO_BOARD=waveshare_rp2040_zero. You can use PICO_BOARD=none for the MCH2022.")
endif()

set(PICO_BOARD $ENV{PICO_BOARD})

message("Build type: \"${CMAKE_BUILD_TYPE}\"")
message("PICO_SDK_PATH set to: ${PICO_SDK_PATH}")
message("PICO_BOARD set to: ${PICO_BOARD}")

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.3.1")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.3.1 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

option(PICO_USE_FASTEST_SUPPORTED_CLOCK "Use the SDK's fastest supported default clock" ON)
pico_sdk_init()

set(GLITCH_SYS_CLOCK_KHZ SYS_CLK_KHZ CACHE STRING "System clock in kHz (default: SDK selection)")

# Firmware
add_executable(${NAME}
    main.c
    usb_control.c
    executor.c
    sequence.c
    usb_descriptors.c
    protocol.c
    i2c_bus.c
    i2c_hw.c
    timing.c
    timing_hw.c
)

# Keep the complete execution path, including SDK helpers, out of XIP flash.
pico_set_binary_type(${NAME} copy_to_ram)

target_include_directories(${NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${NAME}
    pico_stdlib
    pico_unique_id
    hardware_dma
    hardware_clocks
    pico_multicore
    hardware_pio
    hardware_i2c
    tinyusb_device
    tinyusb_board
    cmsis_core
)

target_compile_definitions(${NAME} PRIVATE GLITCH_SYS_CLOCK_KHZ=${GLITCH_SYS_CLOCK_KHZ})
pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/timing.pio)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/timing.pio.json
    COMMAND pioasm -o json -v 0 ${CMAKE_CURRENT_SOURCE_DIR}/timing.pio ${CMAKE_CURRENT_BINARY_DIR}/timing.pio.json
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/timing.pio pioasm)
add_custom_target(timing_model_input DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/timing.pio.json)
add_dependencies(${NAME} timing_model_input)

pico_add_extra_outputs(${NAME})
