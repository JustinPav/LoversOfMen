cmake_minimum_required(VERSION 3.8)
project(Integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Generate message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Coordinates.msg"
)

## This must be listed so the generated messages are usable
ament_export_dependencies(rosidl_default_runtime)

## Ensure message generation happens before building executables
add_executable(coordinates_publisher src/coordinates_publisher.cpp)

## Depend on rclcpp and generated message interfaces
ament_target_dependencies(coordinates_publisher
  rclcpp
)

## Link generated message interfaces to the executable
rosidl_target_interfaces(coordinates_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Install (optional, if you want to make the node runnable after sourcing)
install(TARGETS
  coordinates_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
