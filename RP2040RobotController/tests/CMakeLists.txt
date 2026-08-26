cmake_minimum_required(VERSION 3.16)

project(hexapod_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(hexapod_core_host
    ../src/config/robot_config.cpp
    ../src/config/servo_config.cpp
    ../src/robot/kinematics.cpp
    ../src/robot/robot_geometry.cpp
    ../src/robot/workspace.cpp
    ../src/locomotion/gait_generator.cpp
    ../src/locomotion/contact_adaptation.cpp
    ../src/locomotion/support_polygon.cpp
    ../src/locomotion/locomotion_controller.cpp
    ../src/drivers/servo_controller.cpp
    ../src/drivers/rc_input.cpp
    ../src/drivers/contact_sensors.cpp
)

target_include_directories(hexapod_core_host PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../src
)

add_executable(hexapod_host_tests
    test_main.cpp
)

target_link_libraries(hexapod_host_tests PRIVATE hexapod_core_host)

if(MSVC)
    target_compile_options(hexapod_core_host PRIVATE /W4)
    target_compile_options(hexapod_host_tests PRIVATE /W4)
else()
    target_compile_options(hexapod_core_host PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(hexapod_host_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_test(NAME hexapod_host_tests COMMAND hexapod_host_tests)
