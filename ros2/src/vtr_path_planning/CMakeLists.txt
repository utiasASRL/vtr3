###### HEADER #####
cmake_minimum_required(VERSION 3.16)
project(vtr_path_planning)

# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(Boost REQUIRED COMPONENTS system) # TODO required by steam, check if we can drop these dependencies

find_package(ament_cmake REQUIRED)
find_package(vtr_pose_graph REQUIRED)

# Libraries
file(
  GLOB_RECURSE SRC
  src/*.cpp
)
add_library(${PROJECT_NAME} ${SRC})
ament_target_dependencies(${PROJECT_NAME}
  vtr_pose_graph
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

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


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
