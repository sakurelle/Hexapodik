# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Initialise pico_sdk from installed location.
# This block matches the layout expected by the official Raspberry Pi Pico
# VS Code extension and is intentionally kept before pico_sdk_import.cmake.
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(RP2040RobotController C CXX ASM)

pico_sdk_init()

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(ROBOT_PARAMS_TXT ${CMAKE_CURRENT_LIST_DIR}/config/robot_params.txt)
set(ROBOT_PARAMS_HPP ${CMAKE_CURRENT_BINARY_DIR}/generated/robot_params.hpp)

add_custom_command(
    OUTPUT ${ROBOT_PARAMS_HPP}
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_LIST_DIR}/tools/generate_robot_params.py
        --input ${ROBOT_PARAMS_TXT}
        --output ${ROBOT_PARAMS_HPP}
    DEPENDS
        ${ROBOT_PARAMS_TXT}
        ${CMAKE_CURRENT_LIST_DIR}/tools/generate_robot_params.py
    COMMENT "Generating robot_params.hpp"
    VERBATIM
)

add_custom_target(generate_robot_params DEPENDS ${ROBOT_PARAMS_HPP})

add_executable(RP2040RobotController
    src/main.cpp
    src/app/startup_controller.cpp
    src/control/drive_controller.cpp
    src/input/rc_pwm_input.cpp
    src/protocol/uart_protocol.cpp
    src/robot/gait_controller.cpp
    src/robot/motion_controller.cpp
    src/servo/pio_servo_driver.cpp
    src/servo/servo_calibration.cpp
    src/servo/servo_frame_scheduler.cpp
    src/storage/config_storage.cpp
    src/storage/crc32.cpp
)

add_dependencies(RP2040RobotController generate_robot_params)

pico_generate_pio_header(RP2040RobotController ${CMAKE_CURRENT_LIST_DIR}/pio/servo_14ch.pio)
pico_generate_pio_header(RP2040RobotController ${CMAKE_CURRENT_LIST_DIR}/pio/servo_4ch.pio)

target_include_directories(RP2040RobotController PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_compile_definitions(RP2040RobotController PRIVATE
    PICO_STDIO_USB=1
    PICO_STDIO_UART=0
)

target_link_libraries(RP2040RobotController
    pico_stdlib
    hardware_uart
    hardware_pio
    hardware_dma
    hardware_flash
    hardware_sync
)

pico_enable_stdio_usb(RP2040RobotController 1)
pico_enable_stdio_uart(RP2040RobotController 0)
pico_add_extra_outputs(RP2040RobotController)
