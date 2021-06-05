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
# find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(lgmath REQUIRED)

find_package(vtr_logging REQUIRED)

## C++ Libraries
# npm ply
file(GLOB_RECURSE NPM_PLY_SRC
  src/npm_ply/ply_file_in.cc
  src/npm_ply/ply_file_out.cc
)
add_library(${PROJECT_NAME}_npm_ply ${NPM_PLY_SRC})
ament_target_dependencies(${PROJECT_NAME}_npm_ply Eigen3)
target_include_directories(${PROJECT_NAME}_npm_ply
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# cloud
file(GLOB_RECURSE CLOUD_SRC src/cloud/cloud.cpp)
add_library(${PROJECT_NAME}_cloud ${CLOUD_SRC})
ament_target_dependencies(${PROJECT_NAME}_cloud Eigen3)
target_link_libraries(${PROJECT_NAME}_cloud ${PROJECT_NAME}_npm_ply)
target_include_directories(${PROJECT_NAME}_cloud
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# grid_subsampling
file(GLOB_RECURSE GRID_SUBSAMPLING_SRC src/grid_subsampling/grid_subsampling.cpp)
add_library(${PROJECT_NAME}_grid_subsampling ${GRID_SUBSAMPLING_SRC})
ament_target_dependencies(${PROJECT_NAME}_grid_subsampling Eigen3)
target_link_libraries(${PROJECT_NAME}_grid_subsampling ${PROJECT_NAME}_cloud)
target_include_directories(${PROJECT_NAME}_grid_subsampling
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# polar_processing
file(GLOB_RECURSE POLAR_PROCESSING_SRC src/polar_processing/polar_processing.cpp)
add_library(${PROJECT_NAME}_polar_processing ${POLAR_PROCESSING_SRC})
ament_target_dependencies(${PROJECT_NAME}_polar_processing Eigen3)
target_link_libraries(${PROJECT_NAME}_polar_processing ${PROJECT_NAME}_cloud)
target_include_directories(${PROJECT_NAME}_polar_processing
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# pointmap
file(GLOB_RECURSE POINTMAP_SRC src/pointmap/pointmap.cpp)
add_library(${PROJECT_NAME}_pointmap ${POINTMAP_SRC})
ament_target_dependencies(${PROJECT_NAME}_pointmap Eigen3)
target_link_libraries(${PROJECT_NAME}_pointmap ${PROJECT_NAME}_cloud)
target_include_directories(${PROJECT_NAME}_pointmap
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# icp
file(GLOB_RECURSE ICP_SRC src/icp/icp.cpp src/icp/lgicp.cpp)
add_library(${PROJECT_NAME}_icp ${ICP_SRC})
ament_target_dependencies(${PROJECT_NAME}_icp Eigen3 lgmath vtr_logging)
target_link_libraries(${PROJECT_NAME}_icp ${PROJECT_NAME}_pointmap)
target_include_directories(${PROJECT_NAME}_icp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# point map slam
add_executable(${PROJECT_NAME}_slam src/point_map_slam.cpp src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_slam
  Eigen3
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs pcl_conversions PCL
)
target_link_libraries(${PROJECT_NAME}_slam
  ${PROJECT_NAME}_icp
  ${PROJECT_NAME}_polar_processing
  ${PROJECT_NAME}_grid_subsampling
)
target_include_directories(${PROJECT_NAME}_slam
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  lgmath
  vtr_logging
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_npm_ply
    ${PROJECT_NAME}_cloud
    ${PROJECT_NAME}_grid_subsampling
    ${PROJECT_NAME}_polar_processing
    ${PROJECT_NAME}_pointmap
    ${PROJECT_NAME}_icp
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_slam
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Python Package
ament_python_install_package(${PROJECT_NAME})

# Python Scripts
file(
  GLOB_RECURSE PYTHON_SCRIPTS
  scripts/*.py
)
install(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(icp_tests test/icp_tests.cpp)
  target_include_directories(icp_tests PUBLIC test)
  target_link_libraries(icp_tests ${PROJECT_NAME}_icp)

  # C++ node tests

  # Python node tests

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
