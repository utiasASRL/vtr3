cmake_minimum_required(VERSION 3.16)
project(vtr_testing_utils)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_tactic REQUIRED)
find_package(vtr_pose_graph REQUIRED)


## C++ Libraries

## Executables
# Calculate totoal distance of a graph
add_executable(${PROJECT_NAME}_calculate_distance src/calculate_distance.cpp)
ament_target_dependencies(${PROJECT_NAME}_calculate_distance
  rclcpp sensor_msgs pcl_conversions
  vtr_common vtr_tactic vtr_pose_graph vtr_logging
)
# target_link_libraries(${PROJECT_NAME}_calculate_distance)
target_include_directories(${PROJECT_NAME}_calculate_distance
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_calculate_distance
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Python Package
# ament_python_install_package(${PROJECT_NAME})

# Python Scripts
# file(
#   GLOB_RECURSE PYTHON_SCRIPTS
#   scripts/*.py
# )
# install(
#   PROGRAMS ${PYTHON_SCRIPTS}
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files.
# install(
#   DIRECTORY
#     launch
#     config
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(state_machine_tests test/state_machine_tests.cpp)
  # target_include_directories(state_machine_tests PUBLIC test)
  # target_link_libraries(state_machine_tests ${PROJECT_NAME}_state_machine)

  # C++ node tests

  # Python node tests

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
