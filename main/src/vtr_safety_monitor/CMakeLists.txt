cmake_minimum_required(VERSION 3.16)
project(vtr_safety_monitor)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)

## Libraries
file(GLOB_RECURSE SRC src/safety_monitor/*.cpp)
add_library(${PROJECT_NAME} ${SRC})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs sensor_msgs
  vtr_common vtr_logging
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


# Executables
add_executable(safety_monitor_node src/main.cpp)
ament_target_dependencies(safety_monitor_node
  rclcpp vtr_common vtr_logging
)
target_link_libraries(safety_monitor_node ${PROJECT_NAME})
target_include_directories(safety_monitor_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp vtr_common vtr_logging)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  TARGETS
    safety_monitor_node
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

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()

