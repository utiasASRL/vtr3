cmake_minimum_required(VERSION 3.16)
project(vtr_testing_lidar)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(rclpy REQUIRED)

find_package(PCL COMPONENTS common REQUIRED)
find_package(pcl_conversions REQUIRED)

# find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_navigation REQUIRED)
find_package(vtr_pose_graph REQUIRED)

## C++ Libraries

## Executables
# odometry
add_executable(${PROJECT_NAME}_odometry src/odometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_odometry
  rclcpp sensor_msgs
  vtr_common vtr_navigation vtr_logging
)
# target_link_libraries(${PROJECT_NAME}_odometry)
target_include_directories(${PROJECT_NAME}_odometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# localization
add_executable(${PROJECT_NAME}_localization src/localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_localization
  rclcpp sensor_msgs
  vtr_common vtr_navigation vtr_logging vtr_pose_graph
)
# target_link_libraries(${PROJECT_NAME}_localization)
target_include_directories(${PROJECT_NAME}_localization
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
    ${PROJECT_NAME}_odometry
    ${PROJECT_NAME}_localization
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
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}/
)

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
