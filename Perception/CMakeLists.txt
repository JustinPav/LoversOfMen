cmake_minimum_required(VERSION 3.5)
project(perception)

# Use C++17 (or whatever you need)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add your node
add_executable(find_block_coordinate src/findBlockCoordinate.cpp)
ament_target_dependencies(find_block_coordinate rclcpp)

# Install
install(TARGETS
  find_block_coordinate
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

