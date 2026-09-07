cmake_minimum_required(VERSION 3.8)
project(head_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- find dependencies ---------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# The OpenArm CAN library. Its CMake package name is "OpenArmCAN" and it
# exports the imported target "OpenArmCAN::openarm_can" (a static lib that
# already carries its own include directories).
find_package(OpenArmCAN REQUIRED)

# --- shared library ------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/head_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# Link the OpenArm CAN library. This also propagates its include dirs, so
# #include <openarm/can/socket/openarm.hpp> works from our sources.
target_link_libraries(${PROJECT_NAME}
  OpenArmCAN::openarm_can
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Register the ros2_control plugin with pluginlib. The first argument is the
# base package that owns the plugin interface (hardware_interface), the second
# is the plugin description file listing <class ... type="head_hardware::HeadHw">.
pluginlib_export_plugin_description_file(hardware_interface head_hardware.xml)

# --- install -------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# --- export so other packages can link against libhead_hardware.so -------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  OpenArmCAN
)

ament_package()
