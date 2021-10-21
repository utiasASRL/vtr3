cmake_minimum_required(VERSION 3.16)
project(vtr_testing_boreas)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)

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

# boreas
add_executable(${PROJECT_NAME}_odometry_ros src/odometry_ros.cpp)
ament_target_dependencies(${PROJECT_NAME}_odometry_ros vtr_navigation)
add_executable(${PROJECT_NAME}_odometry_direct src/odometry_direct.cpp)
ament_target_dependencies(${PROJECT_NAME}_odometry_direct vtr_navigation)
add_executable(${PROJECT_NAME}_intra_exp_merging src/intra_exp_merging.cpp)
ament_target_dependencies(${PROJECT_NAME}_intra_exp_merging vtr_navigation)
add_executable(${PROJECT_NAME}_dynamic_detection src/dynamic_detection.cpp)
ament_target_dependencies(${PROJECT_NAME}_dynamic_detection vtr_navigation)

install(
  TARGETS
    ${PROJECT_NAME}_odometry_ros
    ${PROJECT_NAME}_odometry_direct
    ${PROJECT_NAME}_intra_exp_merging
    ${PROJECT_NAME}_dynamic_detection
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install launch files.
install(
  DIRECTORY launch config script
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
