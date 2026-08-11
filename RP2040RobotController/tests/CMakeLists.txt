cmake_minimum_required(VERSION 3.13)
project(RP2040RobotControllerTests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rp2040_robot_controller_tests
    test_main.cpp
    ../src/protocol/uart_protocol.cpp
    ../src/servo/servo_calibration.cpp
    ../src/servo/servo_frame_scheduler.cpp
    ../src/storage/config_storage.cpp
    ../src/storage/crc32.cpp
)

target_include_directories(rp2040_robot_controller_tests PRIVATE
    ../include
)
