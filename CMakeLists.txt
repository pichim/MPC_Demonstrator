cmake_minimum_required(VERSION 3.16)

project(uart_host LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(serial_port STATIC lib/SerialPort/serial_port.cpp)
target_include_directories(serial_port PUBLIC lib/SerialPort)

find_package(Threads REQUIRED)
add_executable(uart_host src/main.cpp)
target_include_directories(uart_host PRIVATE include)
target_link_libraries(uart_host PRIVATE serial_port Threads::Threads)
