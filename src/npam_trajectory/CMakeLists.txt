cmake_minimum_required(VERSION 3.5)
project(npam_trajectory)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# MoveIt2
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem)


# ============================================
# EJECUTABLE 1: joint_planner
# ============================================
add_executable(joint_planner src/joint_planner.cpp)

ament_target_dependencies(joint_planner
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  moveit_kinematics
  moveit_planners_ompl
)

target_link_libraries(joint_planner
  Boost::system
  Boost::filesystem
)

# ============================================
# EJECUTABLE 2: cartesian_planner
# ============================================
add_executable(cartesian_planner src/cartesian_planner.cpp)
ament_target_dependencies(cartesian_planner
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  moveit_kinematics
  moveit_planners_ompl
)
target_link_libraries(cartesian_planner
  Boost::system
  Boost::filesystem
)

# ============================================
# INSTALACIÓN
# ============================================

install(TARGETS
  joint_planner
  cartesian_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
