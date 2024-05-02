cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PIMORONI_PICO_PATH}/pimoroni_pico_import.cmake)

set(NAME pico_micro_ros_example)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)
add_executable(${NAME}
    src/main.cpp
    src/pico_uart_transport.c
)

target_include_directories(${NAME} PRIVATE include)
#target_include_directories(${NAME} PRIVATE robot)

target_include_directories(${NAME} PUBLIC
libmicroros/include
)

include(drivers/button/button)
include(drivers/pwm/pwm)
include(drivers/motor/motor)
include(drivers/encoder/encoder)
include(drivers/pid/pid)
include(drivers/plasma/plasma)
include(robot/robot.cmake)

target_link_libraries(${NAME}
    pico_stdlib
    microros
    button
    motor
    encoder
    pid
    plasma
    robot
)
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(${NAME})
