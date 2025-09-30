cmake_minimum_required(VERSION 3.5)
project(npam_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()