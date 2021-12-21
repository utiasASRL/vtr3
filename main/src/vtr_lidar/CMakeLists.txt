cmake_minimum_required(VERSION 3.16)
project(vtr_lidar)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(perception_pcl REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(vtr_logging REQUIRED)
find_package(vtr_tactic REQUIRED)
find_package(vtr_lidar_msgs REQUIRED)

## C++ Libraries
include_directories(PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# pipelines and modules
file(GLOB_RECURSE MAIN_SRC src/*.cpp)
add_library(${PROJECT_NAME}_main ${MAIN_SRC})
ament_target_dependencies(${PROJECT_NAME}_main
  Eigen3 pcl_conversions perception_pcl pcl_ros
  lgmath steam
  vtr_logging vtr_tactic vtr_lidar_msgs
  visualization_msgs nav_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 pcl_conversions perception_pcl pcl_ros
  lgmath steam
  vtr_logging vtr_tactic vtr_lidar_msgs
  visualization_msgs nav_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_main
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Python Package
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # basic types
  ament_add_gmock(test_types test/test_types.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_types ${PROJECT_NAME}_main)

  # point cloud
  ament_add_gmock(test_point_cloud test/test_point_cloud.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_point_cloud ${PROJECT_NAME}_main)

  # point map
  ament_add_gmock(test_point_scan test/test_point_scan.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_point_scan ${PROJECT_NAME}_main)
  ament_add_gmock(test_point_map test/test_point_map.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_point_map ${PROJECT_NAME}_main)
  ament_add_gmock(test_multi_exp_point_map test/test_multi_exp_point_map.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_multi_exp_point_map ${PROJECT_NAME}_main)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
