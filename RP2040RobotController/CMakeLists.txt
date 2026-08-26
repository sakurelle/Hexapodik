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
cmake_minimum_required(VERSION 3.16)

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(RP2040RobotController C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

target_compile_features(hexapod_core PUBLIC cxx_std_17)

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

pico_generate_pio_header(RP2040RobotController
    ${CMAKE_CURRENT_LIST_DIR}/pio/servo_output.pio
)

target_link_libraries(RP2040RobotController PRIVATE
    hexapod_core
    pico_stdlib
    hardware_dma
    hardware_gpio
    hardware_pio
)

target_compile_options(RP2040RobotController PRIVATE
    -Wall
    -Wextra
)

pico_enable_stdio_usb(RP2040RobotController 1)
pico_enable_stdio_uart(RP2040RobotController 0)
pico_add_extra_outputs(RP2040RobotController)
