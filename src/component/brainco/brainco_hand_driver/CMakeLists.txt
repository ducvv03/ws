# =============================================================================
# BrainCo Hand Driver - CMakeLists.txt
# =============================================================================
# BrainCo 机械手驱动程序构建配置

cmake_minimum_required(VERSION 3.16)
project(brainco_hand_driver)

# -----------------------------------------------------------------------------
# 编译选项
# -----------------------------------------------------------------------------
option(ENABLE_CANFD "Enable CAN FD support (requires ZLG USB-CAN FD libraries)" OFF)
# SocketCAN needs no vendor library, only the Linux CAN headers, so it defaults ON.
option(ENABLE_SOCKETCAN "Enable CAN FD support over the Linux SocketCAN stack" ON)

if(ENABLE_SOCKETCAN AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(WARNING "SocketCAN is Linux-only; disabling ENABLE_SOCKETCAN on ${CMAKE_SYSTEM_NAME}.")
  set(ENABLE_SOCKETCAN OFF)
endif()

# -----------------------------------------------------------------------------
# 编译器警告设置
# -----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# BrainCo Stark SDK 配置
# SDK 由 brainco_stark_sdk_vendor 包在构建时下载并安装到 colcon install 目录，
# 不再依赖本包 vendor/ 目录，也不需要预先运行 download_sdk.sh。
# 该包导出 imported target: brainco_stark_sdk::brainco_stark_sdk
# -----------------------------------------------------------------------------
find_package(brainco_stark_sdk_vendor REQUIRED)

# -----------------------------------------------------------------------------
# ZLG USB-CAN FD 驱动库（可选）
# 该库随仓库 vendor 目录一同发布，详细请见 https://manual.zlg.cn/web/#/146 
# 仅在 ENABLE_CANFD=ON 时启用
# -----------------------------------------------------------------------------
if(ENABLE_CANFD)
  set(ZLG_CANFD_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/usbcanfd_libusb_x64_1.0.12_251029")
  set(ZLG_CANFD_LIB_PATH "${ZLG_CANFD_VENDOR_DIR}/libusbcanfd.so.1.0.12")
  set(ZLG_ZUDS_LIB_PATH   "${ZLG_CANFD_VENDOR_DIR}/libzuds.so")

  foreach(_zlg_lib IN ITEMS ${ZLG_CANFD_LIB_PATH} ${ZLG_ZUDS_LIB_PATH})
    if(NOT EXISTS ${_zlg_lib})
      message(FATAL_ERROR "ZLG USB-CAN FD library not found: ${_zlg_lib}. Please place vendor package under ${ZLG_CANFD_VENDOR_DIR} or set ENABLE_CANFD=OFF to disable CAN FD support.")
    endif()
  endforeach()

  add_library(zlg_usbcanfd SHARED IMPORTED GLOBAL)
  set_target_properties(zlg_usbcanfd PROPERTIES
    IMPORTED_LOCATION ${ZLG_CANFD_LIB_PATH}
  )

  add_library(zlg_zuds SHARED IMPORTED GLOBAL)
  set_target_properties(zlg_zuds PROPERTIES
    IMPORTED_LOCATION ${ZLG_ZUDS_LIB_PATH}
  )
  
  message(STATUS "CAN FD support: ENABLED")
else()
  message(STATUS "CAN FD support: DISABLED")
endif()

# -----------------------------------------------------------------------------
# ROS2 包依赖查找
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)

# -----------------------------------------------------------------------------
# 创建 BrainCo 机械手驱动库
# -----------------------------------------------------------------------------
set(${PROJECT_NAME}_SOURCES
  src/brainco_hand_api.cpp
  src/brainco_hand_hardware.cpp
  src/transport_session_base.cpp
  src/modbus_session.cpp
  src/sdk_helpers.cpp
)

# 条件性地添加 CAN FD 源文件
if(ENABLE_CANFD)
  list(APPEND ${PROJECT_NAME}_SOURCES src/canfd_session.cpp)
endif()

# SocketCAN 传输源文件（无需 vendor 库）
if(ENABLE_SOCKETCAN)
  list(APPEND ${PROJECT_NAME}_SOURCES src/socketcan_session.cpp)
  message(STATUS "SocketCAN support: ENABLED")
else()
  message(STATUS "SocketCAN support: DISABLED")
endif()

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

# 设置编译定义（必须在 add_library 之后）
if(ENABLE_CANFD)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_CANFD=1)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_CANFD=0)
endif()

if(ENABLE_SOCKETCAN)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_SOCKETCAN=1)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_SOCKETCAN=0)
endif()

# -----------------------------------------------------------------------------
# 设置编译特性
# C99 和 C++17 标准
# -----------------------------------------------------------------------------
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

# -----------------------------------------------------------------------------
# 设置包含目录
# BUILD_INTERFACE: 构建时包含目录
# INSTALL_INTERFACE: 安装后包含目录
# -----------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# 设置 ROS2 包依赖
# ament_target_dependencies 自动处理依赖的包含目录和链接库
# -----------------------------------------------------------------------------
ament_target_dependencies(
  ${PROJECT_NAME}
  "pluginlib"
  "rclcpp"
  "rclcpp_lifecycle"
  "hardware_interface"
)

# -----------------------------------------------------------------------------
# 链接外部依赖
# 将 BrainCo Stark SDK 链接到主项目
# -----------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}
  brainco_stark_sdk::brainco_stark_sdk
)

# 条件性地链接 CAN FD 库
if(ENABLE_CANFD)
  target_link_libraries(${PROJECT_NAME}
    zlg_usbcanfd
    zlg_zuds
  )
endif()

# 运行时确保可定位到共享库（构建阶段与安装后）
# Stark SDK 不再列入 RPATH：它由 brainco_stark_sdk_vendor 安装到 install 目录，
# colcon 的环境钩子会把该目录加入 LD_LIBRARY_PATH。以前这里写的是源码树中的
# vendor/ 绝对路径，源码树一挪动就变成悬空引用（ldd 报 not found）。
set(_brainco_build_rpath "")

if(ENABLE_CANFD)
  list(APPEND _brainco_build_rpath "${ZLG_CANFD_VENDOR_DIR}")
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${_brainco_build_rpath}"
  INSTALL_RPATH "\$ORIGIN/.."
)

# -----------------------------------------------------------------------------
# 插件系统配置
# 导出硬件接口插件描述文件，使 ROS2 能发现并加载此硬件接口插件
# -----------------------------------------------------------------------------
pluginlib_export_plugin_description_file(hardware_interface brainco_hand_driver_plugins.xml)

# -----------------------------------------------------------------------------
# 安装配置
# -----------------------------------------------------------------------------
# 安装头文件目录
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装随包分发的第三方共享库（仅 ZLG CANFD，如果启用）
# Stark SDK 由 brainco_stark_sdk_vendor 自行安装，此处不再重复安装，
# 避免同一个 .so 在 install 目录中出现两份而版本不一致。
set(BRAINCO_VENDOR_SHARED_LIBS "")

if(ENABLE_CANFD)
  list(APPEND BRAINCO_VENDOR_SHARED_LIBS
    ${ZLG_CANFD_LIB_PATH}
    ${ZLG_ZUDS_LIB_PATH}
  )
endif()

if(BRAINCO_VENDOR_SHARED_LIBS)
  install(
    FILES ${BRAINCO_VENDOR_SHARED_LIBS}
    DESTINATION lib
  )
endif()

# 安装主项目库和可执行文件
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}        # 导出目标用于其他包依赖
  ARCHIVE DESTINATION lib             # 静态库
  LIBRARY DESTINATION lib             # 动态库
  RUNTIME DESTINATION bin             # 可执行文件
)

# 安装插件配置文件
install(FILES
  brainco_hand_driver_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 安装运行时所需的文件和目录
install(DIRECTORY
  launch      # 启动文件
  config      # 配置文件
  scripts     # 脚本文件
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS  # 保持原始文件权限
)

# -----------------------------------------------------------------------------
# 包导出配置
# 让其他 ROS2 包能够找到并使用此包
# -----------------------------------------------------------------------------
# 导出包含目录
ament_export_include_directories(
  include
)

# 导出库
ament_export_libraries(
  ${PROJECT_NAME}
)

# 导出 CMake 目标
ament_export_targets(
  export_${PROJECT_NAME}
)

# 导出依赖关系
ament_export_dependencies(
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)

# -----------------------------------------------------------------------------
# 测试配置
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# 包声明结束
# 必须是 ROS2 CMake 包的最后一个命令
# -----------------------------------------------------------------------------
ament_package()
