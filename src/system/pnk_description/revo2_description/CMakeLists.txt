cmake_minimum_required(VERSION 3.5)
project(revo2_description)

find_package(ament_cmake REQUIRED)

# 安装资源目录
install(DIRECTORY
  urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

