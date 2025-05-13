cmake_minimum_required(VERSION 3.8)
project(integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)  # Add this line to find the tf2 package

# Create executables
add_executable(pose_publisher src/PosePublisher.cpp)
add_executable(pose_subscriber src/PoseSubscriber.cpp)
add_executable(letter_sort src/LetterSort.cpp)

# Link dependencies
ament_target_dependencies(pose_publisher rclcpp geometry_msgs tf2)
ament_target_dependencies(pose_subscriber rclcpp geometry_msgs tf2)

# Install the executables
install(TARGETS
  pose_publisher
  pose_subscriber
  letter_sort
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
