cmake_minimum_required(VERSION 3.16)
project(vtr_testing_lidar)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_navigation REQUIRED)
find_package(vtr_lidar REQUIRED)

## C++ Libraries

## Executables
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# odometry
add_executable(${PROJECT_NAME}_odometry src/odometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_odometry
  rclcpp rosbag2 rosbag2_cpp
  vtr_common vtr_logging vtr_navigation
)

# localization
add_executable(${PROJECT_NAME}_localization src/localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_localization
  rclcpp rosbag2 rosbag2_cpp
  vtr_common vtr_logging vtr_navigation
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_odometry
    ${PROJECT_NAME}_localization
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # C++ node tests

  # Python node tests

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
