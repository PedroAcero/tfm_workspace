cmake_minimum_required(VERSION 3.8)
project(npam_robot_driver)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()