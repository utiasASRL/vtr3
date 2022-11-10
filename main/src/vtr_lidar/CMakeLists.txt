cmake_minimum_required(VERSION 3.16)
project(vtr_lidar)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_lidar_msgs REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_path_planning REQUIRED)
find_package(vtr_tactic REQUIRED)

## C++ Libraries
include_directories(PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# components
file(GLOB_RECURSE COMPONENTS_SRC
  src/data_types/*.cpp
)
add_library(${PROJECT_NAME}_components ${COMPONENTS_SRC})
ament_target_dependencies(${PROJECT_NAME}_components
  Eigen3 pcl_conversions pcl_ros
  nav_msgs
  lgmath steam
  vtr_common vtr_logging vtr_tactic vtr_path_planning vtr_lidar_msgs
)

# pipelines and modules
file(GLOB_RECURSE PIPELINE_SRC
  src/modules/*.cpp
  src/pipeline.cpp
)
add_library(${PROJECT_NAME}_pipeline ${PIPELINE_SRC})
target_link_libraries(${PROJECT_NAME}_pipeline ${PROJECT_NAME}_components)
ament_target_dependencies(${PROJECT_NAME}_pipeline
  Eigen3 pcl_conversions pcl_ros
  nav_msgs visualization_msgs
  lgmath steam
  vtr_logging vtr_tactic vtr_lidar_msgs
)

# path planner
file(GLOB_RECURSE PATH_PLANNING_SRC
  src/path_planning/*.cpp
)
add_library(${PROJECT_NAME}_path_planning ${PATH_PLANNING_SRC})
target_link_libraries(${PROJECT_NAME}_path_planning ${PROJECT_NAME}_components)
ament_target_dependencies(${PROJECT_NAME}_path_planning
  vtr_logging vtr_path_planning vtr_lidar_msgs
  lgmath steam
)

# additional tools for experiments
file(GLOB_RECURSE TOOLS_SRC
  src/mesh2pcd/*.cpp
)
add_library(${PROJECT_NAME}_tools ${TOOLS_SRC})
ament_target_dependencies(${PROJECT_NAME}_tools
  Eigen3 pcl_conversions pcl_ros
  vtr_common vtr_logging
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 pcl_conversions pcl_ros
  nav_msgs visualization_msgs
  lgmath steam
  vtr_logging vtr_tactic vtr_path_planning vtr_lidar_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_components
    ${PROJECT_NAME}_pipeline
    # ${PROJECT_NAME}_path_planning (TODO)
    ${PROJECT_NAME}_tools
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
  target_link_libraries(test_types ${PROJECT_NAME}_pipeline)

  # point cloud
  ament_add_gmock(test_point_cloud test/test_point_cloud.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_point_cloud ${PROJECT_NAME}_pipeline)

  # point map
  ament_add_gmock(test_point_scan test/test_point_scan.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_point_scan ${PROJECT_NAME}_pipeline)
  ament_add_gmock(test_point_map test/test_point_map.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_point_map ${PROJECT_NAME}_pipeline)
  ament_add_gmock(test_multi_exp_point_map test/test_multi_exp_point_map.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_multi_exp_point_map ${PROJECT_NAME}_pipeline)

  find_package(Boost REQUIRED)
  find_package(PCL REQUIRED)
  add_executable(example_himmelsbach test/segmentation/example_himmelsbach.cpp)
  ament_target_dependencies(example_himmelsbach Boost)
  target_link_libraries(example_himmelsbach ${PCL_LIBRARIES} ${PROJECT_NAME}_pipeline)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
