cmake_minimum_required(VERSION 3.10)
project(hsm CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(hsm src/StateMachineEngine.cpp)
target_include_directories(hsm PUBLIC include)

add_executable(hsm_demo examples/main.cpp)
target_link_libraries(hsm_demo PRIVATE hsm)
