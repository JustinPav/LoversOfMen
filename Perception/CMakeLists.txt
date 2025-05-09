cmake_minimum_required(VERSION 3.5)
project(perception)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

<<<<<<< HEAD
# find ament and ROS2 components
=======
# Find dependencies
>>>>>>> 89f9c3bbb5a0dfe12bee0bb73fd3ac8adf4ba92b
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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
)

# install the node
install(TARGETS refdet
  DESTINATION lib/${PROJECT_NAME}
)

<<<<<<< HEAD
ament_package()
=======
# Generate custom messages
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/Coordinates.msg"
#  DEPENDENCIES builtin_interfaces
#)

# Ensure dependencies are exported
ament_export_dependencies(
  rosidl_default_runtime
  rosidl_default_generators
  builtin_interfaces
)

ament_package()
>>>>>>> 89f9c3bbb5a0dfe12bee0bb73fd3ac8adf4ba92b
