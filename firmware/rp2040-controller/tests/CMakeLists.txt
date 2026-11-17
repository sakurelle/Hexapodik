cmake_minimum_required(VERSION 3.16)

project(rp2040_controller_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(rp2040_controller_host_tests
    test_main.cpp
    ../src/controller/low_level_controller.cpp
    ../src/drivers/contact_sensors.cpp
    ../src/drivers/servo_controller.cpp
    ../src/protocol/cobs.cpp
    ../src/protocol/crc.cpp
    ../src/protocol/protocol.cpp
)

target_include_directories(rp2040_controller_host_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../src
)

if(MSVC)
    target_compile_options(rp2040_controller_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(rp2040_controller_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -Werror -UNDEBUG
    )
endif()

enable_testing()
add_test(NAME rp2040_controller_host_tests COMMAND rp2040_controller_host_tests)
