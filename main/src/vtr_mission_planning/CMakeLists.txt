cmake_minimum_required(VERSION 3.16)
project(vtr_mission_planning)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

find_package(vtr_logging REQUIRED)
find_package(vtr_route_planning REQUIRED)
find_package(vtr_tactic REQUIRED)

## C++ Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# state machine
file(
  GLOB_RECURSE STATE_MACHINE_SRC
  src/state_machine/*.cpp
)
add_library(${PROJECT_NAME}_state_machine ${STATE_MACHINE_SRC})
ament_target_dependencies(${PROJECT_NAME}_state_machine
  vtr_logging vtr_route_planning vtr_tactic
)

# mission server
file(
  GLOB_RECURSE SERVER_SRC
  src/mission_server/*.cpp
)
add_library(${PROJECT_NAME}_mission_server ${SERVER_SRC})
ament_target_dependencies(${PROJECT_NAME}_mission_server
  vtr_logging vtr_route_planning vtr_tactic
)
target_link_libraries(${PROJECT_NAME}_mission_server
  ${PROJECT_NAME}_state_machine
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_action
  vtr_logging vtr_route_planning vtr_tactic
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_state_machine
    ${PROJECT_NAME}_mission_server
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # state machine tests
  ament_add_gtest(test_state_machine test/state_machine/test_state_machine.cpp)
  target_link_libraries(test_state_machine ${PROJECT_NAME}_state_machine)

  # mission server tests
  ament_add_gtest(test_mission_server test/mission_server/test_mission_server.cpp)
  target_link_libraries(test_mission_server ${PROJECT_NAME}_mission_server)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
