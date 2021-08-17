cmake_minimum_required(VERSION 3.16)
project(vtr_navigation)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)

find_package(PROJ REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)  # load frames
find_package(tf2_ros REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_lgmath_extensions REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_pose_graph REQUIRED)
find_package(vtr_tactic REQUIRED)

## TODO make these two optional (depending on which pipeline to use)
find_package(vtr_lidar REQUIRED)
find_package(vtr_vision REQUIRED)

## C++ Libraries
file(GLOB_RECURSE SRC
  src/map_projector.cpp
  src/navigator.cpp
)
add_library(${PROJECT_NAME}_navigator ${SRC})
ament_target_dependencies(${PROJECT_NAME}_navigator
  PROJ
  rclcpp tf2 tf2_ros
  sensor_msgs  # for visualization only
  vtr_common vtr_lgmath_extensions vtr_logging vtr_pose_graph vtr_tactic
  vtr_lidar vtr_vision
)
target_include_directories(${PROJECT_NAME}_navigator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  PROJ
  rclcpp tf2 tf2_ros
  sensor_msgs  # for visualization only
  vtr_common vtr_lgmath_extensions vtr_logging vtr_pose_graph vtr_tactic
  vtr_lidar vtr_vision
)

# main
add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs
  vtr_common
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_navigator)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_navigator
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install launch files.
install(
  DIRECTORY launch config
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
