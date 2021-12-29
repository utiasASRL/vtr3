###### HEADER #####
cmake_minimum_required(VERSION 3.16)
project(vtr_path_planning)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_tactic REQUIRED)

# C++ Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

file(GLOB_RECURSE SRC src/base_path_planner.cpp)
add_library(${PROJECT_NAME}_base ${SRC})
ament_target_dependencies(${PROJECT_NAME}_base vtr_common vtr_logging)

file(GLOB_RECURSE SRC src/mpc/*.cpp)
add_library(${PROJECT_NAME}_mpc ${SRC})
target_link_libraries(${PROJECT_NAME}_mpc ${PROJECT_NAME}_base)
ament_target_dependencies(${PROJECT_NAME}_mpc rclcpp)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp vtr_common vtr_logging)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_base
    ${PROJECT_NAME}_mpc
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
