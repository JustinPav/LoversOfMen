cmake_minimum_required(VERSION 3.5)
project(perception LANGUAGES CXX)

# Enforce C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ament and ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Executable: find_block_coordinates
add_executable(find_block_coordinates src/findBlockCoordinates.cpp)
ament_target_dependencies(find_block_coordinates
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_link_libraries(find_block_coordinates
  ${OpenCV_LIBRARIES}
)

# Executable: refdet
add_executable(refdet src/refdet.cpp)
ament_target_dependencies(refdet
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_link_libraries(refdet
  ${OpenCV_LIBRARIES}
)

# Install executables
install(
  TARGETS
    find_block_coordinates
    refdet
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()