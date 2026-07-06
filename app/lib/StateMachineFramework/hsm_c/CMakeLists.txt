cmake_minimum_required(VERSION 3.10)
project(hsm_c C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(hsm_c src/State.c src/Logger.c src/StateMachineEngine.c)
target_include_directories(hsm_c PUBLIC include)

add_executable(hsm_c_demo examples/main.c)
target_link_libraries(hsm_c_demo PRIVATE hsm_c)
