cmake_minimum_required(VERSION 3.16)
project(vtr_mission_planning)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_path_planning REQUIRED)
find_package(vtr_pose_graph REQUIRED)

# Libraries
# state machine
file(
  GLOB_RECURSE STATE_MACHINE_SRC
  src/event.cpp
  src/state_machine.cpp
  src/states/*.cpp
)
add_library(${PROJECT_NAME}_state_machine ${STATE_MACHINE_SRC})
ament_target_dependencies(${PROJECT_NAME}_state_machine
  Boost
  OpenMP
  vtr_common
  vtr_logging
  vtr_path_planning
  vtr_pose_graph
)
target_include_directories(${PROJECT_NAME}_state_machine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(PACKAGE)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_state_machine
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(state_machine_tests test/state_machine_tests.cpp)
  # ament_target_dependencies(state_machine_tests ${PROJECT_NAME}_index) # Does not work yet
  target_include_directories(state_machine_tests PUBLIC test)
  target_link_libraries(state_machine_tests ${PROJECT_NAME}_state_machine)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
