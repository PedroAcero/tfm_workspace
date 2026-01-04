cmake_minimum_required(VERSION 3.8)
project(npam_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable for plan_logger
add_executable(plan_logger src/plan_logger.cpp)

ament_target_dependencies(plan_logger
  rclcpp
  moveit_msgs
  trajectory_msgs
  std_msgs
)

# Install targets
install(TARGETS
  plan_logger
  DESTINATION lib/${PROJECT_NAME}
)

# Install data directory
install(DIRECTORY
  data/
  DESTINATION share/${PROJECT_NAME}/data
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()