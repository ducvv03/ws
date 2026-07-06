cmake_minimum_required(VERSION 3.13)
project(worker_looper CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Standalone Looper library (no libutils / AOSP dependencies).
add_library(looper STATIC Looper.cpp)
target_include_directories(looper PUBLIC include)
find_package(Threads REQUIRED)
target_link_libraries(looper PUBLIC Threads::Threads)

# Demo / smoke test.
add_executable(looper_demo looper_demo.cpp)
target_link_libraries(looper_demo PRIVATE looper)
