cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(pid C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(pid
    src/pid.c
)

target_include_directories(pid PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(test_pid
    tests/test_pid.c
)

target_link_libraries(test_pid PRIVATE
    pid
)
