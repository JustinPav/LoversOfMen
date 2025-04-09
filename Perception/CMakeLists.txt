cmake_minimum_required(VERSION 3.5)
project(perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(find_block_coordinates src/findBlockCoordinates.cpp)

ament_target_dependencies(find_block_coordinates
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(find_block_coordinates PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(find_block_coordinates
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  find_block_coordinates
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
