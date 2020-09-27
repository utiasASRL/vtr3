cmake_minimum_required(VERSION 3.16)
project(vtr_navigation)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread) # TODO required by mission_planning, check if we can drop this dependency
find_package(rclcpp REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_mission_planning REQUIRED)
find_package(vtr_pose_graph REQUIRED)
# find_package(vtr_vision REQUIRED)

# Libraries
file(GLOB_RECURSE ASSEMBLY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/assemblies/*.cpp)
file(GLOB_RECURSE TACTIC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/tactics/*.cpp)
file(GLOB_RECURSE FACTORY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/factories/*.cpp )
add_library(${PROJECT_NAME}
  ${ASSEMBLY_SRC}
  ${FACTORY_SRC}
  ${TACTIC_SRC}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  vtr_common
  vtr_logging
  vtr_mission_planning
  vtr_pose_graph
  # vtr_vision
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  vtr_common
  vtr_logging
  vtr_mission_planning
  vtr_pose_graph
  # vtr_vision
)

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

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
