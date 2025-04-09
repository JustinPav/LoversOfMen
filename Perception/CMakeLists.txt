cmake_minimum_required(VERSION 3.5)
project(perception)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Executable
add_executable(find_block_coordinate src/findBlockCoordinate.cpp)

# Link dependencies
ament_target_dependencies(find_block_coordinate
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(find_block_coordinate PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(find_block_coordinate
  ${OpenCV_LIBRARIES}
)

# Install target
install(TARGETS
  find_block_coordinate
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
