cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(RP2040RobotController C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_library(hexapod_core
    src/config/robot_config.cpp
    src/config/servo_config.cpp

    src/robot/kinematics.cpp
    src/robot/robot_geometry.cpp
    src/robot/workspace.cpp

    src/locomotion/gait_generator.cpp
    src/locomotion/contact_adaptation.cpp
    src/locomotion/support_polygon.cpp
    src/locomotion/locomotion_controller.cpp
)

target_include_directories(hexapod_core PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_options(hexapod_core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(RP2040RobotController
    src/main.cpp

    src/drivers/servo_controller.cpp
    src/drivers/rc_input.cpp
    src/drivers/contact_sensors.cpp
)

target_include_directories(RP2040RobotController PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(RP2040RobotController
    hexapod_core

    pico_stdlib
    hardware_gpio
    hardware_pio
    hardware_dma
    hardware_timer
)

pico_generate_pio_header(
    RP2040RobotController
    ${CMAKE_CURRENT_LIST_DIR}/pio/servo_output.pio
)

pico_enable_stdio_usb(RP2040RobotController 1)
pico_enable_stdio_uart(RP2040RobotController 0)

pico_add_extra_outputs(RP2040RobotController)