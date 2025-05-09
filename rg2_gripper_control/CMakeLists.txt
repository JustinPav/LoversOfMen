cmake_minimum_required(VERSION 3.8)
project(rg2_gripper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 

# Define executable
add_executable(rg2_control_node src/rg2_control_node.cpp)

# Link against ROS 2 dependencies
ament_target_dependencies(rg2_control_node rclcpp std_msgs sensor_msgs)

# Require C++17
target_compile_features(rg2_control_node PUBLIC cxx_std_17)

# Install the executable
install(TARGETS rg2_control_node
  DESTINATION lib/${PROJECT_NAME})

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()