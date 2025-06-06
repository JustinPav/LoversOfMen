cmake_minimum_required(VERSION 3.5)
project(perception)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find ament and ROS2 components
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# include dirs
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# executable
add_executable(refdet src/refdet.cpp)

# link against ROS2 and OpenCV
ament_target_dependencies(refdet
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  image_transport
  tf2_geometry_msgs
  tf2_ros
)

# install the node
install(TARGETS refdet
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
