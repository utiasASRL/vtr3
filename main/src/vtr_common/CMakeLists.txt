cmake_minimum_required(VERSION 3.16)
project(vtr_common)

## Common setup for vtr packages
include("vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED COMPONENTS chrono timer)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(lgmath REQUIRED)

find_package(vtr_common_msgs REQUIRED)

# Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# utils
file(GLOB_RECURSE UTILS_SRC src/utils/*.cpp)
add_library(${PROJECT_NAME}_utils ${UTILS_SRC})
# timing
file(GLOB_RECURSE TIMING_SRC src/timing/*.cpp)
add_library(${PROJECT_NAME}_timing ${TIMING_SRC})
ament_target_dependencies(${PROJECT_NAME}_timing
  Boost
)
# conversions
file(GLOB_RECURSE CONVERSIONS_SRC src/conversions/*.cpp)
add_library(${PROJECT_NAME}_conversions ${CONVERSIONS_SRC})
ament_target_dependencies(${PROJECT_NAME}_conversions
  tf2_ros tf2_geometry_msgs
  lgmath
  vtr_common_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Boost
  tf2_ros tf2_geometry_msgs
  lgmath
  vtr_common_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}_timing
    ${PROJECT_NAME}_conversions
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
