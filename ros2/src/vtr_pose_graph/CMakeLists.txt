cmake_minimum_required(VERSION 3.16)
project(vtr_pose_graph)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(lgmath REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)
find_package(vtr_storage REQUIRED)

# Libraries
# interface
file(
  GLOB_RECURSE INTERFACE_SRC
  src/interface/*.cpp
)
add_library(${PROJECT_NAME}_interface ${INTERFACE_SRC})
ament_target_dependencies(${PROJECT_NAME}_interface
  lgmath
  vtr_common
  vtr_logging
  vtr_messages
  vtr_storage
)
target_include_directories(${PROJECT_NAME}_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# index
file(
  GLOB_RECURSE INDEX_SRC
  src/index/*.cpp
  src/simple_graph/*.cpp
  src/evaluator/*.cpp
)
add_library(${PROJECT_NAME}_index ${INDEX_SRC})
ament_target_dependencies(${PROJECT_NAME}_index
  lgmath
  vtr_common
  vtr_logging
  vtr_messages
  vtr_storage
)
target_link_libraries(${PROJECT_NAME}_index ${PROJECT_NAME}_interface)
target_include_directories(${PROJECT_NAME}_index
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lgmath)
ament_export_dependencies(vtr_common)
ament_export_dependencies(vtr_messages)
ament_export_dependencies(vtr_storage)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_interface
    ${PROJECT_NAME}_index
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(simple_graph_tests test/simple_graph_tests.cpp)
  # ament_target_dependencies(simple_graph_tests ${PROJECT_NAME}_index) # Does not work yet
  target_link_libraries(simple_graph_tests ${PROJECT_NAME}_index)
  ament_add_gtest(id_tests test/id_tests.cpp)
  target_link_libraries(id_tests ${PROJECT_NAME}_index)
  ament_add_gtest(read_write_tests test/read_write_tests.cpp)
  target_link_libraries(read_write_tests ${PROJECT_NAME}_index)
  ament_add_gtest(evaluator_tests test/evaluator_tests.cpp)
  target_link_libraries(evaluator_tests ${PROJECT_NAME}_index)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
