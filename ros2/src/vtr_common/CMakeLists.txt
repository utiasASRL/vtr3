cmake_minimum_required(VERSION 3.16)
project(vtr_common)

## Common setup for vtr packages
include("vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(date REQUIRED)
find_package(lgmath REQUIRED)
find_package(vtr_messages REQUIRED)

# Libraries
# utils
file(GLOB_RECURSE UTILS_SRC src/utils/*.cpp)
add_library(${PROJECT_NAME} ${UTILS_SRC})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# timing
file(GLOB_RECURSE TIMING_SRC src/timing/*.cpp)
add_library(${PROJECT_NAME}_timing ${TIMING_SRC})
ament_target_dependencies(${PROJECT_NAME}_timing
  date
  vtr_messages
  rclcpp
)
target_include_directories(${PROJECT_NAME}_timing
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# rosutils
file(GLOB_RECURSE ROSUTILS_SRC src/rosutils/*.cpp)
add_library(${PROJECT_NAME}_rosutils ${ROSUTILS_SRC})
ament_target_dependencies(${PROJECT_NAME}_rosutils
  tf2_ros
  tf2_geometry_msgs
  lgmath
)
target_include_directories(${PROJECT_NAME}_rosutils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  tf2_ros
  tf2_geometry_msgs
  date
  vtr_messages
  lgmath
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_timing
    ${PROJECT_NAME}_rosutils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
