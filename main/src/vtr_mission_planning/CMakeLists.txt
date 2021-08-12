cmake_minimum_required(VERSION 3.16)
project(vtr_mission_planning)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_path_planning REQUIRED)
find_package(vtr_pose_graph REQUIRED)

# C++ Libraries
# state machine
file(
  GLOB_RECURSE STATE_MACHINE_SRC
  src/event.cpp
  src/state_machine.cpp
  src/states/*.cpp
)
add_library(${PROJECT_NAME}_state_machine ${STATE_MACHINE_SRC})
ament_target_dependencies(${PROJECT_NAME}_state_machine
  Boost OpenMP
  vtr_common vtr_logging vtr_path_planning vtr_pose_graph
)
target_include_directories(${PROJECT_NAME}_state_machine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# mission planning server
file(GLOB_RECURSE SERVER_SRC src/ros_mission_server.cpp)
add_library(${PROJECT_NAME}_server ${SERVER_SRC})
ament_target_dependencies(${PROJECT_NAME}_server
  Boost OpenMP
  rclcpp rclcpp_action
  vtr_common vtr_logging vtr_path_planning vtr_pose_graph
)
target_link_libraries(${PROJECT_NAME}_server ${PROJECT_NAME}_state_machine)
target_include_directories(${PROJECT_NAME}_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Boost OpenMP
  rclcpp rclcpp_action
  vtr_common vtr_logging vtr_path_planning vtr_pose_graph
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_state_machine
    ${PROJECT_NAME}_server
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
  ament_add_gtest(state_machine_tests test/state_machine_tests.cpp)
  target_include_directories(state_machine_tests PUBLIC test)
  target_link_libraries(state_machine_tests ${PROJECT_NAME}_state_machine)
  ament_add_gtest(mission_server_tests test/mission_server_tests.cpp)
  target_include_directories(mission_server_tests PUBLIC test)
  target_link_libraries(mission_server_tests ${PROJECT_NAME}_server)

  # C++ node tests
  file(GLOB_RECURSE CPP_NODE_TEST_SRC test/mission_server_node.cpp)
  add_executable(mission_server_node ${CPP_NODE_TEST_SRC})
  target_link_libraries(mission_server_node
    ${PROJECT_NAME}_server
    ${PROJECT_NAME}_state_machine
  )
  target_include_directories(mission_server_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  install(
    TARGETS mission_server_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
  )

  # Python node tests
  file(
    GLOB_RECURSE PYTHON_NODE_TEST_SRC
    test/*.py
  )
  install(
    PROGRAMS ${PYTHON_NODE_TEST_SRC}
    DESTINATION lib/${PROJECT_NAME}
  )

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
