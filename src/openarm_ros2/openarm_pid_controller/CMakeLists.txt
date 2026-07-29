cmake_minimum_required(VERSION 3.16)
project(openarm_pid_controller)

# Inlined from ros2_control_cmake (not present in this binary install).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(openarm_pid_controller_parameters
  src/openarm_pid_controller.yaml
)

add_library(openarm_pid_controller SHARED
  src/openarm_pid_controller.cpp
)
target_compile_features(openarm_pid_controller PUBLIC cxx_std_17)
target_include_directories(openarm_pid_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/openarm_pid_controller>
)
target_link_libraries(openarm_pid_controller PUBLIC
                      openarm_pid_controller_parameters
                      angles::angles
                      control_toolbox::control_toolbox
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${std_srvs_TARGETS}
                      ${control_msgs_TARGETS}
                      ${std_srvs_TARGETS})

pluginlib_export_plugin_description_file(controller_interface openarm_pid_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/openarm_pid_controller
)

install(TARGETS
    openarm_pid_controller
    openarm_pid_controller_parameters
  EXPORT export_openarm_pid_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_openarm_pid_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
