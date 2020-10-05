cmake_minimum_required(VERSION 3.16)
project(vtr_testing)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread) # TODO required by mission_planning, check if we can drop this dependency
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_navigation REQUIRED)

# Libraries

# Executables
add_executable(module_vo src/module_vo.cpp)
ament_target_dependencies(module_vo
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  vtr_common
  vtr_logging
  vtr_navigation
)
target_include_directories(module_vo
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(
  TARGETS
    module_vo
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
