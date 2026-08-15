cmake_minimum_required(VERSION 3.13)
project(RP2040RobotControllerTests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(ROBOT_PARAMS_TXT ${CMAKE_CURRENT_LIST_DIR}/../config/robot_params.txt)
set(ROBOT_PARAMS_HPP ${CMAKE_CURRENT_BINARY_DIR}/generated/robot_params.hpp)

add_custom_command(
    OUTPUT ${ROBOT_PARAMS_HPP}
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_LIST_DIR}/../tools/generate_robot_params.py
        --input ${ROBOT_PARAMS_TXT}
        --output ${ROBOT_PARAMS_HPP}
    DEPENDS
        ${ROBOT_PARAMS_TXT}
        ${CMAKE_CURRENT_LIST_DIR}/../tools/generate_robot_params.py
    COMMENT "Generating test robot_params.hpp"
    VERBATIM
)

add_custom_target(generate_robot_params_tests DEPENDS ${ROBOT_PARAMS_HPP})

add_executable(rp2040_robot_controller_tests
    test_main.cpp
    ../src/control/drive_controller.cpp
    ../src/protocol/uart_protocol.cpp
    ../src/robot/gait_generator.cpp
    ../src/robot/cartesian_validation.cpp
    ../src/robot/kinematics.cpp
    ../src/robot/motion_controller.cpp
    ../src/app/startup_controller.cpp
    ../src/servo/servo_calibration.cpp
    ../src/servo/servo_frame_scheduler.cpp
    ../src/storage/config_storage.cpp
    ../src/storage/crc32.cpp
)

add_dependencies(rp2040_robot_controller_tests generate_robot_params_tests)

target_include_directories(rp2040_robot_controller_tests PRIVATE
    ../include
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

enable_testing()
add_test(NAME rp2040_robot_controller_tests COMMAND rp2040_robot_controller_tests)
add_test(
    NAME robot_params_parser
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/../tools/generate_robot_params.py --self-test
)
