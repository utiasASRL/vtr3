cmake_minimum_required(VERSION 3.16)
project(vtr_lidar)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(vtr_logging REQUIRED)
find_package(vtr_tactic REQUIRED)

## C++ Libraries
include_directories(PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# utility functions
file(GLOB_RECURSE UTIL_SRC
  src/npm_ply/*.cc
  src/cloud/*.cpp
  src/grid_subsampling/*.cpp
  src/polar_processing/*.cpp
  src/pointmap/*.cpp
  src/utils.cpp
)
add_library(${PROJECT_NAME}_utils ${UTIL_SRC})
ament_target_dependencies(${PROJECT_NAME}_utils
  Eigen3 PCL pcl_conversions
  vtr_logging
)

# pipelines and modules
file(GLOB_RECURSE MAIN_SRC
  src/modules/*.cpp
  src/cache.cpp
  src/pipeline.cpp
)
add_library(${PROJECT_NAME}_main ${MAIN_SRC})
ament_target_dependencies(${PROJECT_NAME}_main
  Eigen3 PCL pcl_conversions
  lgmath steam
  vtr_logging vtr_tactic
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}_utils)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 PCL pcl_conversions
  lgmath steam
  vtr_logging vtr_tactic
  visualization_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}_main
  EXPORT export_${PROJECT_NAME}
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # C++ node tests

  # Python node tests

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
