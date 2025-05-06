cmake_minimum_required(VERSION 3.5)
project(perception)

# Enforce C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
  "msg/Coordinates.msg"
)

# Generate interface
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

  # Include directories
  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
  )

  add_executable(refdet src/refdet.cpp)

  ament_target_dependencies(refdet
    rclcpp
    sensor_msgs
    cv_bridge
    image_transport
    OpenCV
  )

  target_link_libraries(refdet ${OpenCV_LIBRARIES})

  install(TARGETS
    refdet
    DESTINATION lib/${PROJECT_NAME}
  )


ament_package()
