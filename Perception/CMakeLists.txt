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
#find_package(rosidl_default_generators REQUIRED)


# Include OpenCV headers
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Add your existing include directories here
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include # Ensure this points to the directory containing perception/msg/coordinates.hpp
  )

# Executable: find_block_coordinates
#add_executable()#find_block_coordinates src/findBlockCoordinates.cpp)
#ament_target_dependencies(find_block_coordinates
#  rclcpp
#  sensor_msgs
#  cv_bridge
#  image_transport
#  OpenCV
#  rosidl_default_runtime
#)
#target_link_libraries(find_block_coordinates
#  ${OpenCV_LIBRARIES}
#)

# Executable: refdet
add_executable(refdet src/refdet.cpp)
ament_target_dependencies(refdet
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  rosidl_default_runtime
)



target_link_libraries(refdet
  ${OpenCV_LIBRARIES}
)

# Install executables
install(
  TARGETS
    #find_block_coordinates
    refdet
  DESTINATION lib/${PROJECT_NAME}
)

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

