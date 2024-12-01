cmake_minimum_required(VERSION 3.10)
project(cola2_stonefish)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cola2_msgs REQUIRED)
find_package(stonefish_ros2 REQUIRED)  # Add this line

# Include directories
include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${cola2_msgs_INCLUDE_DIRS}
  ${stonefish_ros2_INCLUDE_DIRS}  # Add this line
)

# Add the executable
add_executable(cola2_stonefish_node src/cola2_stonefish_node.cpp)

# Link against dependencies
ament_target_dependencies(
  cola2_stonefish_node
  rclcpp
  std_msgs
  cola2_msgs
  stonefish_ros2  # Add this line
)

# Install the executable
install(TARGETS cola2_stonefish_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/logitechF310teleop.py
    scripts/odom2tf.py
    scripts/bluerov2_logitechF310teleop.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install data and scenarios folders
install(DIRECTORY data scenarios
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing and install test files
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package the project
ament_package()

